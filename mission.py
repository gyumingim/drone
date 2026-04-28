"""
mission.py — 이륙 → 북/동/남/서 각 0.5m 왕복 → 착륙

NED 프레임 기준:
  x = 북 (North)
  y = 동 (East)
  z = 아래 (Down, 음수=위쪽)
초기화 시점 드론 헤딩이 '북'으로 설정됨.
"""
import time
from pymavlink import mavutil
from uwb_reader import UWBReader
from common import (
    connect, do_takeoff, do_land,
    go_to, wait_pos, cmd,
    ts, TAKEOFF_M,
)

SPEED_MS  = 0.3           # 순항 속도 (m/s)
CRUISE_Z  = -TAKEOFF_M    # 비행 고도 NED (음수=위쪽)

# (x_north, y_east, 레이블) — 각 방향 0.5m 후 원점 복귀
WAYPOINTS = [
    ( 0.5,  0.0, '북 0.5m'),
    ( 0.0,  0.0, '원점'),
    ( 0.0,  0.5, '동 0.5m'),
    ( 0.0,  0.0, '원점'),
    (-0.5,  0.0, '남 0.5m'),
    ( 0.0,  0.0, '원점'),
    ( 0.0, -0.5, '서 0.5m'),
    ( 0.0,  0.0, '원점'),
]


def main():
    uwb = UWBReader()
    uwb.start()
    print(f'[UWB] {ts()} origin 대기...')
    while uwb.get_xy() is None:
        time.sleep(0.2)
    print(f'[UWB] {ts()} origin 확정: {uwb.get_xy()}')

    c, stop = connect(uwb)
    if c is None:
        return

    if not do_takeoff(c, stop):
        do_land(c, stop)
        return

    time.sleep(1)  # 이륙 후 안정화

    # 속도 제한
    cmd(c, mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 1, SPEED_MS, -1, 0, 0, 0, 0)
    print(f'[NAV] {ts()} 순항 속도 {SPEED_MS}m/s 설정')

    for x, y, label in WAYPOINTS:
        print(f'[NAV] {ts()} → {label}  ({x:.1f}N, {y:.1f}E)')
        go_to(c, x, y, CRUISE_Z)
        if not wait_pos(c, x, y):
            print(f'[NAV] {ts()} {label} 도달 실패 — 착륙')
            do_land(c, stop)
            return
        print(f'[NAV] {ts()} {label} 도달!')
        time.sleep(1)

    do_land(c, stop)


if __name__ == '__main__':
    main()
