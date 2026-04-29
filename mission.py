"""
mission.py — UWB 기반 웨이포인트 비행 (tag 미사용)

동작 순서:
  1. UWB origin 확정
  2. FC 연결 → EKF → GUIDED → ARM
  3. 이륙 1m
  4. 1초 위치 안정화 (이륙 지점 고정)
  5. 북쪽 0.5m 이동
  6. 도달 확인 후 1초 호버
  7. 착륙

좌표계: NED (North-East-Down)
  x = 북(North), y = 동(East), z = 아래(Down) — 위쪽은 음수
  이륙 시점 드론 위치가 (0, 0) origin (UWB 앵커 기준 상대좌표)

용도: AprilTag 없이 UWB만으로 waypoint 비행 기능 검증
"""
import time
from loguru import logger
from pymavlink import mavutil
from lib_uwb_reader import UWBReader
from lib_common import (
    connect, do_takeoff, do_land,
    go_to, wait_pos, cmd,
    TAKEOFF_M,
)

SPEED_MS = 0.3          # 순항 속도 (m/s) — 실내 기준 보수적 값
CRUISE_Z = -TAKEOFF_M   # 비행 고도 NED (음수=위쪽, 예: -1.0 = 지면 위 1m)

WAYPOINTS = [
    (0.5, 0.0, '북 0.5m'),
]


def _hover(c, uwb, x, y, secs):
    """지정 좌표를 secs초 동안 2Hz go_to로 위치 고정.

    hold_position()은 VelAccel(속도=0)만 유지해 바람에 밀려도 복귀 안 함.
    명시적 position target을 반복 전송해야 PosVelAccel 서브모드 유지 → 원위치 복귀.
    패킷 손실 시 재전송 효과도 있음.
    """
    deadline = time.time() + secs
    while time.time() < deadline:
        if uwb.get_xy() is None:
            logger.warning('[SAFE] UWB 끊김 — 착륙')
            return False
        go_to(c, x, y, CRUISE_Z)
        time.sleep(0.5)
    return True


def main():
    # ── UWB origin 확정 ──────────────────────────────────────────────────────
    uwb = UWBReader()
    uwb.start()
    logger.info('[UWB] origin 대기...')
    while uwb.get_xy() is None:
        time.sleep(0.2)
    logger.info('[UWB] origin 확정: {}', uwb.get_xy())

    # ── FC 연결 및 준비 ───────────────────────────────────────────────────────
    c, stop, cache, lock = connect(uwb)
    if c is None:
        return

    # ── 이륙 ─────────────────────────────────────────────────────────────────
    if not do_takeoff(c, stop, cache, lock):
        do_land(c, stop, cache)
        return

    # ── 이륙 직후 안정화 1초 ─────────────────────────────────────────────────
    # do_takeoff() 완료 시점엔 ArduPilot이 hold_position()(VelAccel) 상태.
    # go_to(0,0)을 반복 전송해 PosVelAccel 서브모드로 전환 → 이륙 지점 고정.
    logger.info('[NAV] 이륙 안정화...')
    if not _hover(c, uwb, 0, 0, 1.0):
        do_land(c, stop, cache)
        return

    # ── 순항 속도 설정 ────────────────────────────────────────────────────────
    cmd(c, mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 1, SPEED_MS, -1, 0, 0, 0, 0)
    logger.info('[NAV] 순항 속도 {}m/s 설정', SPEED_MS)

    # ── 웨이포인트 순차 이동 ─────────────────────────────────────────────────
    for x, y, label in WAYPOINTS:
        logger.info('[NAV] → {}  ({:.1f}N, {:.1f}E)', label, x, y)
        go_to(c, x, y, CRUISE_Z)

        if not wait_pos(c, cache, lock, x, y):
            logger.warning('[NAV] {} 도달 실패 — 착륙', label)
            do_land(c, stop, cache)
            return

        logger.info('[NAV] {} 도달!', label)

        # 도달 후 1초 호버 — 직전 go_to가 PosVelAccel 유지 중이므로
        # 추가 go_to 없이 sleep해도 위치 고정됨. 단 UWB 끊김은 감시.
        if not _hover(c, uwb, x, y, 1.0):
            do_land(c, stop, cache)
            return

    # ── 착륙 ─────────────────────────────────────────────────────────────────
    do_land(c, stop, cache)


if __name__ == '__main__':
    main()
