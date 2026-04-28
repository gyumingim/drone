"""
flight_tag.py — 이륙 1m → AprilTag 위 호버링

동작:
  1. UWB origin 확정 대기 (10~30번째 평균)
  2. ARM → 이륙 1m
  3. VPE 전환:
     - Tag 미감지: UWB → VPE (cov 0.05)
     - Tag 감지:   Tag → VPE (cov 0.002)  EKF가 Tag 우선 신뢰
  4. Tag 감지되면 tag 정중앙 위로 go_to 보정
  5. Ctrl+C → 착륙

좌표계:
  tag_reader.py 에서 (north, east, down, yaw) 반환
  north>0 = tag이 드론 북쪽에 있음 → 드론이 북으로 이동해야 tag 위
  east>0  = tag이 드론 동쪽에 있음 → 드론이 동으로 이동해야 tag 위
"""
import math
import time
import threading

from pymavlink import mavutil
from uwb_reader import UWBReader
from tag_reader import TagReader
from common import (
    connect, do_takeoff, do_land,
    go_to, ts, TAKEOFF_M,
)

HOVER_ALT   = TAKEOFF_M       # 호버링 고도 (m)
CTRL_HZ     = 5               # tag 보정 주기
TAG_TOL_M   = 0.10            # tag 중심 도달 판정 반경 (m)

# VPE covariance
_COV_UWB = [0.0] * 21
_COV_UWB[0] = _COV_UWB[6] = 0.05
_COV_UWB[11] = 9999.0

_COV_TAG = [0.0] * 21
_COV_TAG[0] = _COV_TAG[6] = 0.002
_COV_TAG[11] = 0.002


def _vision_loop(c, uwb, tag, cache, lock, stop):
    """Tag 감지 여부에 따라 VPE 소스 전환.
    Tag 최초 감지 시 tag world 위치 고정 → 이후 tag 기준 드론 위치 계산.
    """
    tag_world = None   # (tn, te): tag의 NED world 좌표

    while not stop.is_set():
        pose = tag.get_pose()
        now  = time.time()

        if pose:
            n, e, d, yaw = pose

            # tag world 위치 초기화 (최초 1회)
            if tag_world is None:
                with lock:
                    pos = cache['local_pos']
                if pos:
                    # tag는 드론에서 (n, e) 만큼 떨어진 곳
                    tag_world = (pos.x + n, pos.y + e)
                    print(f'[TAG] {ts()} tag world 확정 '
                          f'({tag_world[0]:.2f}, {tag_world[1]:.2f})')

            if tag_world:
                # 드론 위치 = tag world - 현재 tag 상대값
                drone_n = tag_world[0] - n
                drone_e = tag_world[1] - e
                c.mav.vision_position_estimate_send(
                    int(now * 1e6),
                    drone_n, drone_e, -HOVER_ALT,
                    0.0, 0.0, yaw,
                    _COV_TAG)
        else:
            # Tag 없음 — UWB fallback
            tag_world = None   # tag 놓치면 다음 감지 때 재초기화
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

        time.sleep(0.05)   # 20Hz


def _control_loop(c, tag, cache, lock, stop):
    """Tag 감지되면 tag 정중앙 위로 go_to 보정."""
    while not stop.is_set():
        pose = tag.get_pose()
        if pose:
            n, e, d, yaw = pose
            dist = math.hypot(n, e)
            with lock:
                pos = cache['local_pos']
            if pos:
                # tag 위로 이동: 현재 위치에서 (n, e) 만큼 이동
                go_to(c, pos.x + n, pos.y + e, -HOVER_ALT)
                print(f'[CTRL] {ts()} tag N={n:+.2f} E={e:+.2f} '
                      f'D={d:.2f} dist={dist:.2f}m '
                      f'{"★ 도달" if dist < TAG_TOL_M else ""}')
        time.sleep(1.0 / CTRL_HZ)


def main():
    # UWB origin 확정 대기
    uwb = UWBReader()
    uwb.start()
    print(f'[UWB] {ts()} origin 확정 대기...')
    while uwb.get_xy() is None:
        time.sleep(0.2)
    print(f'[UWB] {ts()} origin 확정: {uwb.get_xy()}')

    # 카메라 + tag 감지 시작
    tag = TagReader()
    tag.start()
    print(f'[TAG] {ts()} 카메라 초기화...')
    time.sleep(1)

    # FC 연결 (start_vision=False → 직접 VPE 전송)
    c, stop, cache, lock = connect(uwb, start_vision=False)
    if c is None:
        return

    # 이륙
    if not do_takeoff(c, stop, cache, lock):
        do_land(c, stop, cache)
        return

    print(f'[HOVER] {ts()} 이륙 완료 — tag 감지 대기')

    # VPE + 제어 루프 시작
    threading.Thread(
        target=_vision_loop,
        args=(c, uwb, tag, cache, lock, stop),
        daemon=True,
    ).start()
    threading.Thread(
        target=_control_loop,
        args=(c, tag, cache, lock, stop),
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
