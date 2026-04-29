"""
flight_tag.py — AprilTag + UWB 융합 호버링 (메인 비행 코드)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
동작 순서
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  1. UWB origin 확정 (앵커 기준 절대좌표 계산)
  2. 카메라 초기화 (RealSense 파이프라인 시작)
  3. FC 연결 → EKF 준비 → GUIDED 모드 → ARM
  4. 이륙 1m (UWB VPE 없이 baro+IMU로 고도 제어)
  5. _vision_loop 시작 → Tag/UWB VPE 20Hz 전송 + go_to 명령
  6. Ctrl+C → 착륙

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

from lib_uwb_reader import UWBReader
from lib_tag_reader import TagReader
from lib_common import connect, do_takeoff, do_land, go_to, ts, TAKEOFF_M

# 이륙 고도와 호버 고도를 동일하게 유지 (1m)
HOVER_ALT = TAKEOFF_M

# ── VPE covariance 행렬 (21-element 상삼각 행렬, 단위 m²) ────────────────────
# 6×6 공분산 행렬의 상삼각 요소를 행 우선으로 나열
# 인덱스: [0]=xx, [6]=yy, [11]=zz (나머지는 0 = 상관관계 없음)
# variance = (예상오차)² 로 설정

# UWB: ±50cm 오차 → variance = 0.5² = 0.25
# z는 무시 (고도는 baro가 담당) → 9999로 설정해 EKF가 z 값을 신뢰하지 않게 함
_COV_UWB = [0.0] * 21
_COV_UWB[0] = _COV_UWB[6] = 0.25
_COV_UWB[11] = 9999.0

# TAG: ±4.5cm 오차 → variance = 0.045² ≈ 0.002
# tag는 z(고도)도 신뢰할 수 있으므로 z variance도 낮게 설정
_COV_TAG = [0.0] * 21
_COV_TAG[0] = _COV_TAG[6] = 0.002
_COV_TAG[11] = 0.002


def _vision_loop(c, uwb, tag, cache, lock, stop):
    """20Hz VPE 전송 루프 — Tag/UWB 소스 전환 및 EKF reset_counter 관리.

    이 함수는 별도 스레드에서 실행되며 이륙 완료 후 시작됨.
    이륙 중(baro 기반 고도 제어)에는 실행되지 않음.
    """
    prev_source = None   # 직전 VPE 소스 ('tag' | 'uwb' | None)
    reset_cnt   = 0      # EKF frame reset 신호 (0~255 순환, 변경 시 EKF 좌표계 리셋)
    last_vpe    = None   # (x, y, z) — tag+UWB 둘 다 없을 때 stale VPE 재전송용

    while not stop.is_set():
        pose = tag.get_pose()   # (north, east, down, yaw) or None
        now  = time.time()

        # 드론 현재 heading (나침반 없으면 gyro 적분값)
        # VPE yaw 필드에 넣어 EKF yaw 추정에 활용
        with lock:
            att = cache['attitude']
        drone_yaw = att.yaw if att else 0.0

        if pose:
            # ── Tag 감지: tag origin 기준 VPE ──────────────────────────────
            n, e, d, _ = pose   # tag yaw 미사용 (드론 heading과 독립)

            # 소스 전환 감지: UWB→TAG 전환 시 좌표계가 바뀌므로 reset_counter 증가
            # reset_counter가 바뀌면 EKF가 이전 위치 추정을 버리고 새 값을 즉시 수용
            if prev_source != 'tag':
                reset_cnt = (reset_cnt + 1) % 256
                print(f'[VPE] {ts()} UWB→TAG 전환 (reset={reset_cnt})')

            # 태그=(0,0) origin 좌표계에서 드론 위치 = (-n, -e)
            # 예: 태그가 드론 북쪽 0.3m → n=0.3 → 드론은 태그 기준 south(-0.3)
            # go_to(0,0)으로 태그 origin(=태그 바로 위)을 향해 이동
            c.mav.vision_position_estimate_send(
                int(now * 1e6),          # 타임스탬프 (μs)
                -n, -e, -HOVER_ALT,      # 드론 위치 (태그 기준, NED)
                0.0, 0.0, drone_yaw,     # roll=0, pitch=0, yaw=드론heading
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
                    print(f'[VPE] {ts()} TAG→UWB 전환 (reset={reset_cnt})')

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
                # 마지막 알려진 위치를 cov=9999(신뢰도 없음)로 전송해 timeout만 방지
                # EKF는 이 값을 사실상 무시하고 IMU+baro만으로 버팀
                _cov_stale = [0.0] * 21
                _cov_stale[0] = _cov_stale[6] = _cov_stale[11] = 9999.0
                c.mav.vision_position_estimate_send(
                    int(now * 1e6),
                    last_vpe[0], last_vpe[1], last_vpe[2],
                    0.0, 0.0, drone_yaw,
                    _cov_stale, reset_cnt)
                print(f'[VPE] {ts()} 소스 없음 — 마지막 위치 유지 (stale)')

        time.sleep(0.05)   # 20Hz (50ms)


def main():
    # ── UWB origin 확정 ──────────────────────────────────────────────────────
    # 이륙 후 UWB 폴백이 필요하므로 비행 전 origin 확정 필수
    uwb = UWBReader()
    uwb.start()
    print(f'[UWB] {ts()} origin 확정 대기...')
    while uwb.get_xy() is None:
        time.sleep(0.2)
    print(f'[UWB] {ts()} origin 확정: {uwb.get_xy()}')

    # ── 카메라 초기화 ─────────────────────────────────────────────────────────
    # RealSense 파이프라인은 start() 내부 스레드에서 비동기로 시작됨
    # 1초 대기: 파이프라인 초기화 + 첫 프레임 수신까지 시간 확보
    tag = TagReader()
    tag.start()
    print(f'[TAG] {ts()} 카메라 초기화...')
    time.sleep(1)

    # ── FC 연결 및 준비 ───────────────────────────────────────────────────────
    # start_vision=False: 이륙 전 VPE 전송 안 함
    # 이유: 이륙은 baro+IMU로 충분, _vision_loop는 이륙 완료 후 시작
    #       (이륙 중 VPE가 없어도 ARMING_CHECK=0이면 ARM/TAKEOFF 정상 동작)
    c, stop, cache, lock = connect(uwb, start_vision=False)
    if c is None:
        return

    # ── 이륙 ─────────────────────────────────────────────────────────────────
    if not do_takeoff(c, stop, cache, lock):
        do_land(c, stop, cache)
        return

    print(f'[HOVER] {ts()} 이륙 완료 — VPE 루프 시작, tag 감지 대기')

    # ── VPE 루프 시작 ─────────────────────────────────────────────────────────
    # 이륙 완료 후 시작 — Tag+UWB 융합 위치 추정 + go_to 명령
    threading.Thread(
        target=_vision_loop,
        args=(c, uwb, tag, cache, lock, stop),
        daemon=True,
    ).start()

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
