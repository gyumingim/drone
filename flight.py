"""
flight.py — UWB 기반 제자리 이착륙 (tag 미사용)

동작 순서:
  1. UWB origin 확정 (약 10~30초 소요)
  2. FC 연결 → EKF 준비 → GUIDED 모드 → ARM
  3. 이륙 1m
  4. HOVER_S초 제자리 호버 (UWB 끊기면 즉시 착륙)
  5. 착륙

용도: 카메라/AprilTag 없이 UWB만으로 기본 이착륙 검증용
"""
import time
from loguru import logger
from lib_uwb_reader import UWBReader
from lib_tag_reader import TagReader
from lib_common import connect, do_takeoff, do_land, go_to, start_depth_sender, HOVER_S, TAKEOFF_M


def main():
    # ── 카메라 초기화 (depth 고도 소스) ─────────────────────────────────────
    tag = TagReader()
    tag.start()
    logger.info('[TAG] 카메라 초기화...')
    time.sleep(1)

    # ── UWB origin 확정 ──────────────────────────────────────────────────────
    uwb = UWBReader()
    uwb.start()
    logger.info('[UWB] origin 대기...')
    while uwb.get_xy() is None:
        time.sleep(0.2)
    logger.info('[UWB] origin 확정: {}', uwb.get_xy())

    # ── FC 연결 및 준비 ───────────────────────────────────────────────────────
    # connect() 내부에서:
    #   - _reader_loop: MAVLink 수신 스레드
    #   - _vision_loop: UWB → VPE 20Hz 전송 스레드 (start_vision=True 기본값)
    #   - _hb_loop: GCS heartbeat 1Hz 전송 (FC watchdog 방지)
    #   - _rc_override_loop: throttle failsafe 방지용 RC override
    c, stop, cache, lock = connect(uwb)
    if c is None:
        return

    # ── depth DISTANCE_SENSOR 전송 시작 ──────────────────────────────────────
    start_depth_sender(c, tag, stop)

    # ── 이륙 ─────────────────────────────────────────────────────────────────
    # do_takeoff 실패 시 (ACK 거부 or 고도 미달) 즉시 착륙 후 종료
    if not do_takeoff(c, stop, cache, lock):
        do_land(c, stop, cache)
        return

    # ── 호버 ─────────────────────────────────────────────────────────────────
    # 2Hz로 go_to(0,0) 전송 — PosVelAccel 서브모드 활성화
    # hold_position()은 VelAccel(속도=0)만 유지해 바람에 밀려도 복귀 안 함.
    # 명시적 위치 목표를 주기적으로 전송해야 EKF 오차를 보정하며 원위치 복귀.
    # 참조: ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
    logger.info('[HOVR] 호버 {}s (이륙 지점 위치 고정)', HOVER_S)
    deadline = time.time() + HOVER_S
    while time.time() < deadline:
        # UWB 신호 유실 감지 → 위치 추정 불가 → 즉시 착륙
        # VPE가 끊기면 EKF가 위치 추정을 잃고 드론이 drift할 수 있어 안전 착륙
        if uwb.get_xy() is None:
            logger.warning('[SAFE] UWB 끊김 — 착륙')
            break
        go_to(c, 0, 0, -TAKEOFF_M)  # 이륙 지점(UWB origin) 고정, 10Hz
        time.sleep(0.1)

    # ── 착륙 ─────────────────────────────────────────────────────────────────
    do_land(c, stop, cache)


if __name__ == '__main__':
    main()
