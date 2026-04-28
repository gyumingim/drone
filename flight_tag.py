"""
flight_tag.py — 이륙 1m → AprilTag 위 속도 제어 호버링

동작:
  1. UWB origin 확정 대기
  2. ARM → 이륙 1m
  3. UWB → VPE (common._vision_loop 가 EKF 위치 참조 공급)
  4. Tag 감지 시: vx = Kp*n, vy = Kp*e 속도 명령
     Tag 미감지: vx=vy=0 (제자리 유지)
  5. Ctrl+C → 착륙

좌표계:
  tag_reader.py 반환 (north, east, down, yaw):
    north > 0 → tag이 드론 북쪽 → 드론을 북으로 이동해야 tag 위
    east  > 0 → tag이 드론 동쪽 → 드론을 동으로 이동해야 tag 위
"""
import math
import time
import threading

from pymavlink import mavutil
from uwb_reader import UWBReader
from tag_reader import TagReader
from common import connect, do_takeoff, do_land, ts, TAKEOFF_M

HOVER_ALT = TAKEOFF_M
CTRL_HZ = 10        # 제어 루프 주기 (Hz)
KP = 0.4            # 비례 게인 (m/s per m)
MAX_VEL = 0.5       # 최대 허용 속도 (m/s)
TAG_TOL_M = 0.10    # 도달 판정 반경 (m)

# velocity-only mask: pos xyz 무시, vx/vy 사용, vz/가속/yaw 무시
_VEL_MASK = 0b11111100111  # 0x7E7


def _send_velocity(c, vx, vy):
    """NED 속도 명령 (vz=0, 고도 유지)."""
    c.mav.set_position_target_local_ned_send(
        0, c.target_system, c.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        _VEL_MASK,
        0, 0, -HOVER_ALT,
        vx, vy, 0,
        0, 0, 0,
        0, 0)


def _control_loop(c, tag, stop):
    """Tag 감지 시 속도 명령, 미감지 시 vx=vy=0."""
    dt = 1.0 / CTRL_HZ
    while not stop.is_set():
        pose = tag.get_pose()
        if pose:
            n, e, d, _ = pose
            dist = math.hypot(n, e)
            vx = max(-MAX_VEL, min(MAX_VEL, KP * n))
            vy = max(-MAX_VEL, min(MAX_VEL, KP * e))
            _send_velocity(c, vx, vy)
            print(f'[CTRL] {ts()} tag N={n:+.2f} E={e:+.2f} '
                  f'→ vx={vx:+.2f} vy={vy:+.2f} '
                  f'dist={dist:.2f}m'
                  f'{" ★ 도달" if dist < TAG_TOL_M else ""}')
        else:
            _send_velocity(c, 0, 0)
        time.sleep(dt)


def main():
    uwb = UWBReader()
    uwb.start()
    print(f'[UWB] {ts()} origin 확정 대기...')
    while uwb.get_xy() is None:
        time.sleep(0.2)
    print(f'[UWB] {ts()} origin 확정: {uwb.get_xy()}')

    tag = TagReader()
    tag.start()
    print(f'[TAG] {ts()} 카메라 초기화...')
    time.sleep(1)

    # start_vision=True: UWB→VPE는 common._vision_loop 담당
    c, stop, cache, lock = connect(uwb, start_vision=True)
    if c is None:
        return

    if not do_takeoff(c, stop, cache, lock):
        do_land(c, stop, cache)
        return

    print(f'[HOVER] {ts()} 이륙 완료 — tag 감지 대기 (10Hz 속도 제어)')

    threading.Thread(
        target=_control_loop,
        args=(c, tag, stop),
        daemon=True,
    ).start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass

    do_land(c, stop, cache)


if __name__ == '__main__':
    main()
