"""
flight_tag.py — 이륙 1m → AprilTag 위 호버링

VPE 전략:
  - Tag 감지: tag=(0,0) origin, 드론=(-n,-e,-alt) 로 VPE 전송
              go_to(0, 0, -alt) → 태그 정중앙 위로 수렴
  - Tag 미감지: UWB VPE 전송 + go_to(uwb_pos) 로 현재 위치 홀드
  - 프레임 전환 시 reset_counter 증가 → EKF가 새 좌표계 즉시 수용

참조: ardupilot.org/dev/docs/mavlink-nongps-position-estimation.html
      mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE
"""
import time
import threading

from uwb_reader import UWBReader
from tag_reader import TagReader
from common import connect, do_takeoff, do_land, go_to, ts, TAKEOFF_M

HOVER_ALT = TAKEOFF_M

# VISION_POSITION_ESTIMATE covariance (21-element upper-triangular, m²)
# [0]=xx [6]=yy [11]=zz  — variance = (예상오차m)²
_COV_UWB = [0.0] * 21
_COV_UWB[0] = _COV_UWB[6] = 0.25   # UWB ±50cm → 0.5²
_COV_UWB[11] = 9999.0               # z 무시

_COV_TAG = [0.0] * 21
_COV_TAG[0] = _COV_TAG[6] = 0.002  # tag ±4.5cm → 0.045²
_COV_TAG[11] = 0.002


def _vision_loop(c, uwb, tag, cache, lock, stop):
    """20Hz VPE 전송 + 프레임 전환 관리.

    Tag frame:  태그=(0,0) origin, 드론=(-n,-e)
    UWB frame:  UWB origin 기준 절대 좌표
    프레임 전환 시 reset_counter 증가 → EKF 즉시 수렴
    """
    prev_source = None
    reset_cnt = 0

    while not stop.is_set():
        pose = tag.get_pose()
        now = time.time()

        if pose:
            n, e, d, yaw = pose

            if prev_source != 'tag':
                reset_cnt = (reset_cnt + 1) % 256
                print(f'[VPE] {ts()} UWB→TAG 전환 (reset={reset_cnt})')

            # 태그=(0,0) origin, 드론 위치=(-n, -e)
            c.mav.vision_position_estimate_send(
                int(now * 1e6),
                -n, -e, -HOVER_ALT,
                0.0, 0.0, yaw,
                _COV_TAG, reset_cnt)

            go_to(c, 0, 0, -HOVER_ALT)
            prev_source = 'tag'

        else:
            xy = uwb.get_xy()
            if xy:
                if prev_source != 'uwb':
                    reset_cnt = (reset_cnt + 1) % 256
                    print(f'[VPE] {ts()} TAG→UWB 전환 (reset={reset_cnt})')
                    # 현재 UWB 위치에서 홀드
                    go_to(c, xy[0], xy[1], -HOVER_ALT)

                with lock:
                    att = cache['attitude']
                yaw = att.yaw if att else 0.0
                c.mav.vision_position_estimate_send(
                    int(now * 1e6),
                    xy[0], xy[1], 0.0,
                    0.0, 0.0, yaw,
                    _COV_UWB, reset_cnt)

                prev_source = 'uwb'

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
