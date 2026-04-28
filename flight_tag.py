"""
flight_tag.py — 이륙 1m → AprilTag 위 VPE 전환 + go_to 호버링

동작:
  1. UWB origin 확정 대기
  2. ARM → 이륙 1m
  3. _vision_loop (20Hz):
     - Tag 미감지: UWB → VPE (cov=0.05)
     - Tag 감지:   Tag → VPE (cov=0.002) + go_to(tag_world)
       최초 감지 시 tag_world = ekf_pos + (n, e) 로 anchor
  4. Ctrl+C → 착륙

참조: github.com/stephendade/aprilmav (VPE 방식),
      ardupilot.org/dev/docs/mavlink-nongps-position-estimation.html
"""
import time
import threading

from uwb_reader import UWBReader
from tag_reader import TagReader
from common import connect, do_takeoff, do_land, go_to, ts, TAKEOFF_M

HOVER_ALT = TAKEOFF_M

# VISION_POSITION_ESTIMATE covariance (21-element upper-triangular)
# cov[0]=xx, cov[6]=yy, cov[11]=zz
_COV_UWB = [0.0] * 21
_COV_UWB[0] = _COV_UWB[6] = 0.05
_COV_UWB[11] = 9999.0   # z 무시 (UWB z 신뢰도 낮음)

_COV_TAG = [0.0] * 21
_COV_TAG[0] = _COV_TAG[6] = 0.002
_COV_TAG[11] = 0.002    # tag는 z(고도)도 신뢰


def _vision_loop(c, uwb, tag, cache, lock, stop):
    """20Hz VPE 전송.
    Tag 감지: tag 기준 드론 world 위치 → VPE(cov=0.002) + go_to(tag_world).
    Tag 미감지: UWB → VPE(cov=0.05), tag_world 리셋.
    """
    tag_world = None  # (tn, te): tag의 NED world 좌표 (최초 감지 시 anchor)

    while not stop.is_set():
        pose = tag.get_pose()
        now = time.time()

        if pose:
            n, e, d, yaw = pose

            # tag world 좌표 최초 anchor (EKF 위치 + 현재 상대 오프셋)
            if tag_world is None:
                with lock:
                    pos = cache['local_pos']
                if pos:
                    tag_world = (pos.x + n, pos.y + e)
                    print(f'[TAG] {ts()} tag world anchor '
                          f'({tag_world[0]:.2f}, {tag_world[1]:.2f})')

            if tag_world:
                # 드론 world 위치 = tag_world - 현재 상대값
                drone_n = tag_world[0] - n
                drone_e = tag_world[1] - e

                c.mav.vision_position_estimate_send(
                    int(now * 1e6),
                    drone_n, drone_e, -HOVER_ALT,
                    0.0, 0.0, yaw,
                    _COV_TAG)

                # tag 정중앙 위로 position setpoint 갱신
                go_to(c, tag_world[0], tag_world[1], -HOVER_ALT)

        else:
            tag_world = None  # tag 놓치면 다음 감지 때 재anchor
            xy = uwb.get_xy()
            if xy:
                with lock:
                    att = cache['attitude']
                yaw = att.yaw if att else 0.0
                c.mav.vision_position_estimate_send(
                    int(now * 1e6),
                    xy[0], xy[1], 0.0,
                    0.0, 0.0, yaw,
                    _COV_UWB)

        time.sleep(0.05)  # 20Hz


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

    # start_vision=False: VPE는 _vision_loop에서 직접 관리
    c, stop, cache, lock = connect(uwb, start_vision=False)
    if c is None:
        return

    if not do_takeoff(c, stop, cache, lock):
        do_land(c, stop, cache)
        return

    print(f'[HOVER] {ts()} 이륙 완료 — tag 감지 대기')

    threading.Thread(
        target=_vision_loop,
        args=(c, uwb, tag, cache, lock, stop),
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
