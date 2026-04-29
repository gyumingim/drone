"""
mission.py — UWB 기반 웨이포인트 비행 (tag 미사용)

동작 순서:
  1. UWB origin 확정
  2. FC 연결 → EKF → GUIDED → ARM
  3. 이륙 1m
  4. 북 0.5m → 동 0.5m → 남 0.5m → 서 0.5m → 원점 순서로 이동
  5. 원점 도달 후 착륙

좌표계: NED (North-East-Down)
  x = 북(North), y = 동(East), z = 아래(Down) — 위쪽은 음수
  이륙 시점 드론 위치가 (0, 0) origin (UWB 앵커 기준 상대좌표)

용도: AprilTag 없이 UWB만으로 waypoint 비행 기능 검증
"""
import time
from pymavlink import mavutil
from lib_uwb_reader import UWBReader
from lib_common import (
    connect, do_takeoff, do_land,
    go_to, wait_pos, cmd,
    ts, TAKEOFF_M,
)

SPEED_MS  = 0.3         # 순항 속도 (m/s) — 실내 기준 보수적 값
CRUISE_Z  = -TAKEOFF_M  # 비행 고도 NED (음수=위쪽, 예: -1.0 = 지면 위 1m)

# 웨이포인트 리스트: (x_north, y_east, 설명)
# UWB origin(이륙 위치) 기준 상대 좌표 (m)
WAYPOINTS = [
    ( 0.5,  0.0, '북 0.5m'),
    ( 0.0,  0.5, '동 0.5m'),
    (-0.5,  0.0, '남 0.5m'),
    ( 0.0, -0.5, '서 0.5m'),
    ( 0.0,  0.0, '원점'),    # 출발점으로 복귀
]


def main():
    # ── UWB origin 확정 ──────────────────────────────────────────────────────
    # origin 확정 전에는 get_xy()가 None → EKF에 VPE 전송 불가 → 비행 불가
    uwb = UWBReader()
    uwb.start()
    print(f'[UWB] {ts()} origin 대기...')
    while uwb.get_xy() is None:
        time.sleep(0.2)
    print(f'[UWB] {ts()} origin 확정: {uwb.get_xy()}')

    # ── FC 연결 및 준비 ───────────────────────────────────────────────────────
    # connect() 내부: _vision_loop(UWB VPE 20Hz) 자동 시작
    c, stop, cache, lock = connect(uwb)
    if c is None:
        return

    # ── 이륙 ─────────────────────────────────────────────────────────────────
    if not do_takeoff(c, stop, cache, lock):
        do_land(c, stop, cache)
        return

    # 이륙 직후 EKF/VPE 안정화 대기
    # 목표 고도 도달 직후 바로 이동 명령을 내리면 VPE 기반 위치가 아직 수렴 중일 수 있음
    time.sleep(1)

    # ── 순항 속도 설정 ────────────────────────────────────────────────────────
    # MAV_CMD_DO_CHANGE_SPEED 파라미터:
    #   param1=1: ground speed 타입 (ArduCopter는 실제로 param1 무시, param2만 사용)
    #   param2=SPEED_MS: 목표 속도 (m/s)
    #   param3=-1: throttle 변경 없음
    # 참조: ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
    cmd(c, mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 1, SPEED_MS, -1, 0, 0, 0, 0)
    print(f'[NAV] {ts()} 순항 속도 {SPEED_MS}m/s 설정')

    # ── 웨이포인트 순차 이동 ─────────────────────────────────────────────────
    for x, y, label in WAYPOINTS:
        print(f'[NAV] {ts()} → {label}  ({x:.1f}N, {y:.1f}E)')

        # SET_POSITION_TARGET_LOCAL_NED로 목표 위치 전송
        # EKF가 UWB VPE로 현재 위치를 알고 있어야 go_to가 올바르게 동작
        go_to(c, x, y, CRUISE_Z)

        # LOCAL_POSITION_NED 기반 도달 판정 (허용오차 0.3m, 타임아웃 20s)
        if not wait_pos(c, cache, lock, x, y):
            print(f'[NAV] {ts()} {label} 도달 실패 — 착륙')
            do_land(c, stop, cache)
            return

        print(f'[NAV] {ts()} {label} 도달!')
        time.sleep(1)  # 각 웨이포인트에서 1초 정지

    # ── 착륙 ─────────────────────────────────────────────────────────────────
    do_land(c, stop, cache)


if __name__ == '__main__':
    main()
