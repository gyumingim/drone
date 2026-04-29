"""
flight_tag.py — AprilTag + UWB 융합 호버링 (메인 비행 코드)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
동작 순서
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  1. UWB origin 확정 (앵커 기준 절대좌표 계산)
  2. 카메라 초기화 (RealSense 파이프라인 시작)
  3. FC 연결 → EKF 준비 → GUIDED 모드 → ARM
  4. _vision_loop 시작 → UWB VPE 20Hz 전송 (이륙 중 EKF 위치 유지)
  5. 이륙 1m
  6. Tag 감지 시 Tag VPE + go_to 명령으로 수렴
  7. Ctrl+C → 착륙

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
VPE(VISION_POSITION_ESTIMATE) 전략
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  [Tag 감지 시]
    - 좌표계: 태그=(0,0) origin, 드론 위치=(-n, -e)
    - EKF에 태그 기준 드론 위치 전송 → go_to(0,0) → 태그 정중앙 수렴
    - covariance 낮음(0.002) → EKF가 tag 위치를 높은 신뢰도로 수용

  [Tag 미감지 시]
    - UWB 절대좌표를 VPE로 전송 → go_to(uwb_pos) → 현재 위치 홀드
    - covariance 높음(0.25) → EKF가 UWB를 낮은 신뢰도로 수용

  [Tag↔UWB 전환 시]
    - reset_counter 증가 → EKF에게 "좌표계 바뀜" 신호
    - EKF가 이전 위치 추정을 버리고 새 좌표계를 즉시 수용

  [둘 다 없을 시]
    - 마지막 위치를 cov=9999(매우 낮은 신뢰도)로 재전송
    - EKF timeout(약 5초) 방지 → timeout 발생 시 failsafe 가능성

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
참조
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  ardupilot.org/dev/docs/mavlink-nongps-position-estimation.html
  mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE
"""
import time
import threading

from loguru import logger
from lib_uwb_reader import UWBReader
from lib_tag_reader import TagReader
from lib_common import connect, do_takeoff, do_land, go_to, TAKEOFF_M

# 이륙 고도와 호버 고도를 동일하게 유지 (1m)
HOVER_ALT = TAKEOFF_M

# ── VPE covariance 행렬 (21-element 상삼각 행렬, 단위 m²) ────────────────────
# 6×6 공분산 행렬의 상삼각 요소를 행 우선으로 나열
# 인덱스: [0]=xx, [6]=yy, [11]=zz (나머지는 0 = 상관관계 없음)
# variance = (예상오차)² 로 설정

# UWB: ±50cm 오차 → variance = 0.5² = 0.25
# z는 EK3_SRC1_POSZ=1(Baro)이 이미 무시하므로 cov[11]을 9999로 올릴 필요 없음.
# ArduPilot GCS_Common.cpp: posErr = sqrt(cov[0]+cov[6]+cov[11]) 합산 계산 →
# cov[11]=9999이면 posErr≈100m이 되어 EKF가 XY까지 사실상 무시함 (버그).
_COV_UWB = [0.0] * 21
_COV_UWB[0] = _COV_UWB[6] = _COV_UWB[11] = 0.25

# TAG: ±4.5cm 오차 → variance = 0.045² ≈ 0.002
# 실제 EKF에 들어가는 posErr = sqrt(0.002+0.002+0.002) = 0.0775m
# → VISO_POS_M_NSE 하한(기본 0.1m)에 clamp → EKF posErr = 0.1m
# z(고도)는 EK3_SRC1_POSZ=1(Baro)이 이미 처리 → cov[11] 값은 posErr 합산에만 영향
_COV_TAG = [0.0] * 21
_COV_TAG[0] = _COV_TAG[6] = _COV_TAG[11] = 0.002


def _vision_loop(c, uwb, tag, cache, lock, stop):
    """20Hz VPE 전송 루프 — Tag/UWB 소스 전환 및 EKF reset_counter 관리.

    이 함수는 별도 스레드에서 실행되며 FC 연결 직후(이륙 전)부터 시작됨.
    이륙 중에도 UWB VPE를 전송해 EKF가 위치를 잃지 않도록 유지.
    """
    prev_source = None  # 직전 VPE 소스 ('tag' | 'uwb' | None)
    reset_cnt = 0       # EKF frame reset 신호 (0~255 순환, 변경 시 EKF 좌표계 리셋)
    last_vpe = None     # (x, y, z) — tag+UWB 둘 다 없을 때 stale VPE 재전송용

    while not stop.is_set():
        pose = tag.get_pose()  # (north, east, down, yaw) or None
        now = time.time()

        # FC에서 현재 yaw를 읽어 VPE yaw 필드로 돌려줌
        # EK3_SRC1_YAW=6(ExternalNav)이므로 EKF가 실제로 사용함
        # 독립 yaw 센서가 없으므로 FC 자신의 gyro 적분값을 확인하는 역할
        with lock:
            att = cache['attitude']
        drone_yaw = att.yaw if att else 0.0

        if pose:
            # ── Tag 감지: tag origin 기준 VPE ──────────────────────────────
            n, e, d, tag_yaw = pose  # tag_yaw: 태그 정북 정렬 가정 → NED yaw로 사용
            drone_yaw = tag_yaw      # gyro 적분값을 tag yaw로 동기화 → TAG→UWB 전환 시 연속성 유지

            # 소스 전환 감지: UWB→TAG 전환 시 좌표계가 바뀌므로 reset_counter 증가
            # reset_counter가 바뀌면 EKF가 이전 위치 추정을 버리고 새 값을 즉시 수용
            if prev_source != 'tag':
                reset_cnt = (reset_cnt + 1) % 256
                logger.info('[VPE] UWB→TAG 전환 (reset={})', reset_cnt)

            # 태그=(0,0) origin 좌표계에서 드론 위치 = (-n, -e)
            # 예: 태그가 드론 북쪽 0.3m → n=0.3 → 드론은 태그 기준 south(-0.3)
            # go_to(0,0)으로 태그 origin(=태그 바로 위)을 향해 이동
            # yaw: tag 기반 (드리프트 없음) / FC gyro yaw는 tag 미감지 시에만 사용
            c.mav.vision_position_estimate_send(
                int(now * 1e6),          # 타임스탬프 (μs)
                -n, -e, -HOVER_ALT,      # x,y: 태그 기준 드론 위치 / z: EK3_SRC1_POSZ=Baro라 무시됨
                0.0, 0.0, tag_yaw,       # roll=0, pitch=0 / yaw: tag 기반 NED yaw (EKF가 사용)
                _COV_TAG, reset_cnt)

            last_vpe = (-n, -e, -HOVER_ALT)
            go_to(c, 0, 0, -HOVER_ALT)  # 태그 정중앙 위로 이동 (NED z 음수=위)
            prev_source = 'tag'

        else:
            # ── Tag 미감지: UWB 절대좌표 VPE ───────────────────────────────
            xy = uwb.get_xy()
            if xy:
                switching = (prev_source != 'uwb')

                # TAG→UWB 전환 시 좌표계 변경 알림
                if switching:
                    reset_cnt = (reset_cnt + 1) % 256
                    logger.info('[VPE] TAG→UWB 전환 (reset={})', reset_cnt)

                # UWB 절대좌표를 VPE로 전송 (앵커 기준 절대좌표)
                c.mav.vision_position_estimate_send(
                    int(now * 1e6),
                    xy[0], xy[1], 0.0,   # UWB x, y 좌표 (z는 신뢰 안 함)
                    0.0, 0.0, drone_yaw,
                    _COV_UWB, reset_cnt)

                last_vpe = (xy[0], xy[1], 0.0)

                # 전환 직후 한 루프는 go_to 생략 — EKF가 새 좌표계를 수렴할 시간(50ms) 확보
                # 수렴 전에 go_to를 보내면 EKF가 잘못된 위치 기준으로 명령을 해석할 수 있음
                if not switching:
                    go_to(c, xy[0], xy[1], -HOVER_ALT)

                prev_source = 'uwb'

            elif last_vpe:
                # ── Tag+UWB 둘 다 없음: stale VPE 전송 ────────────────────
                # VPE를 완전히 끊으면 EKF timeout(~5초) 발생 → position lost failsafe
                # 마지막 알려진 위치를 cov=9999로 전송해 timeout만 방지.
                # posErr = sqrt(9999×3) = 173m → 100m에 clamp →
                # EKF Kalman gain≈0 → 위치 추정 사실상 갱신 안 됨 → IMU+baro만으로 버팀
                # (UWB[11]=9999 버그와 달리 여기는 XY도 무시되는 게 의도임)
                _cov_stale = [0.0] * 21
                _cov_stale[0] = _cov_stale[6] = _cov_stale[11] = 9999.0
                c.mav.vision_position_estimate_send(
                    int(now * 1e6),
                    last_vpe[0], last_vpe[1], last_vpe[2],
                    0.0, 0.0, drone_yaw,
                    _cov_stale, reset_cnt)
                logger.debug('[VPE] 소스 없음 — 마지막 위치 유지 (stale)')

        # ── DISTANCE_SENSOR 전송 (EK3_SRC1_POSZ=3 Rangefinder 고도 소스) ──
        # MAV_SENSOR_ROTATION_PITCH_270(25) = 하향 — ArduPilot이 RNGFND1로 인식
        depth_alt = tag.get_depth_alt()
        if depth_alt:
            c.mav.distance_sensor_send(
                0,                      # timestamp (ArduPilot이 무시)
                10,                     # min_distance cm
                1000,                   # max_distance cm (D435i 최대 10m)
                int(depth_alt * 100),   # current_distance cm
                0,                      # type (무시)
                0,                      # id (무시)
                25,                     # MAV_SENSOR_ROTATION_PITCH_270 (하향)
                0,                      # covariance (무시)
            )

        time.sleep(0.05)   # 20Hz (50ms)


def main():
    # ── UWB origin 확정 ──────────────────────────────────────────────────────
    # 이륙 후 UWB 폴백이 필요하므로 비행 전 origin 확정 필수
    uwb = UWBReader()
    uwb.start()
    logger.info('[UWB] origin 확정 대기...')
    while uwb.get_xy() is None:
        time.sleep(0.2)
    logger.info('[UWB] origin 확정: {}', uwb.get_xy())

    # ── 카메라 초기화 ─────────────────────────────────────────────────────────
    # RealSense 파이프라인은 start() 내부 스레드에서 비동기로 시작됨
    # 1초 대기: 파이프라인 초기화 + 첫 프레임 수신까지 시간 확보
    tag = TagReader()
    tag.start()
    logger.info('[TAG] 카메라 초기화...')
    time.sleep(1)

    # ── FC 연결 및 준비 ───────────────────────────────────────────────────────
    # start_vision=False: connect() 내장 UWB VPE 루프 미사용
    # 대신 _vision_loop(아래)가 Tag+UWB 융합 VPE를 직접 담당
    c, stop, cache, lock = connect(uwb, start_vision=False)
    if c is None:
        return

    # ── VPE 루프 시작 (이륙 전) ──────────────────────────────────────────────
    # 이륙 중에도 UWB VPE가 전송돼야 EKF가 수평 위치를 잃지 않음.
    # 이 루프를 do_takeoff() 뒤에 두면 이륙 중 EKF가 위치 추정을 잃을 수 있음.
    threading.Thread(
        target=_vision_loop,
        args=(c, uwb, tag, cache, lock, stop),
        daemon=True,
    ).start()

    # ── 이륙 ─────────────────────────────────────────────────────────────────
    if not do_takeoff(c, stop, cache, lock):
        do_land(c, stop, cache)
        return

    logger.info('[HOVER] 이륙 완료 — tag 감지 대기')

    # ── 메인 루프: Ctrl+C 대기 ───────────────────────────────────────────────
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass

    # ── 착륙 ─────────────────────────────────────────────────────────────────
    do_land(c, stop, cache)


if __name__ == '__main__':
    main()
