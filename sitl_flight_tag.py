"""
sitl_flight_tag.py — SITL 테스트용 flight_tag (카메라/UWB/depth 모두 fake)

flight_tag.py와 로직 동일. 차이점:
  - FC: /dev/ttyACM0 → tcp:127.0.0.1:5760 (ArduPilot SITL 기본 TCP)
  - UWB: DWM1001 시리얼 → FakeUWB (항상 (0,0) 반환)
  - 카메라: FakeTagReader (Tag 미감지, depth=1.0m 고정)
  - QGC: sim_vehicle.py 실행 시 MAVProxy가 UDP 14550 자동 출력 → QGC 자동 연결

실행 전 준비:
  1. ArduPilot SITL 시작:
       cd ~/ardupilot && sim_vehicle.py -v ArduCopter --console
  2. SITL 파라미터 설정 (MAVProxy 콘솔):
       param set ARMING_CHECK 0 (없으면 arm throttle force 사용)
       param set VISO_TYPE 1
       param set EK3_SRC1_POSXY 6
       param set EK3_SRC1_YAW 6
       param set EK3_SRC1_POSZ 1
  3. QGC 실행 → 자동으로 UDP 14550 연결됨
  4. 이 스크립트 실행:
       python3 sitl_flight_tag.py

검증 목적:
  - connect → EKF → ARM → TAKEOFF → VPE(UWB fake) 전송 → LAND 흐름
  - DISTANCE_SENSOR fake depth 20Hz 전송 여부
  - QGC에서 드론 위치/자세/고도 실시간 확인
"""
import time
import threading

from loguru import logger
import lib_common                       # FC_PORT 오버라이드를 위해 모듈로 임포트
lib_common.FC_PORT = 'udpin:0.0.0.0:14551'  # MAVProxy UDP 출력 포트 (14550=QGC, 14551=스크립트)

from lib_common import connect, do_takeoff, do_land, go_to, TAKEOFF_M

HOVER_ALT = TAKEOFF_M

_COV_UWB = [0.0] * 21
_COV_UWB[0] = _COV_UWB[6] = _COV_UWB[11] = 0.25

_COV_TAG = [0.0] * 21
_COV_TAG[0] = _COV_TAG[6] = _COV_TAG[11] = 0.002


class FakeUWB:
    """SITL용 가짜 UWB — 항상 origin (0, 0) 반환."""
    def start(self): pass
    def get_xy(self): return (0.0, 0.0)


class FakeTagReader:
    """SITL용 가짜 카메라 — Tag 미감지, depth 1.0m 고정."""
    def start(self): pass
    def get_pose(self): return None          # 항상 Tag 미감지 → UWB fallback
    def get_depth_alt(self): return 1.0      # 고정 1m (EK3_SRC1_POSZ=Baro면 무시됨)
    def get_frame(self): return None
    def get_latency(self): return (0.0, 0.0, 0.0)
    def get_depth_latency(self): return (0.0, 0.0)


def _vision_loop(c, uwb, tag, cache, lock, stop):
    """flight_tag._vision_loop와 동일. uwb만 FakeUWB 사용."""
    prev_source = None
    reset_cnt = 0
    last_vpe = None

    while not stop.is_set():
        pose = tag.get_pose()
        now = time.time()
        with lock:
            att = cache['attitude']
        drone_yaw = att.yaw if att else 0.0

        if pose:
            n, e, d, tag_yaw = pose
            drone_yaw = tag_yaw
            if prev_source != 'tag':
                reset_cnt = (reset_cnt + 1) % 256
                logger.info('[VPE] UWB→TAG 전환 (reset={})', reset_cnt)
            c.mav.vision_position_estimate_send(
                int(now * 1e6),
                -n, -e, -HOVER_ALT,
                0.0, 0.0, tag_yaw,
                _COV_TAG, reset_cnt)
            last_vpe = (-n, -e, -HOVER_ALT)
            go_to(c, 0, 0, -HOVER_ALT)
            prev_source = 'tag'

        else:
            xy = uwb.get_xy()
            if xy:
                switching = (prev_source != 'uwb')
                if switching:
                    reset_cnt = (reset_cnt + 1) % 256
                    logger.info('[VPE] TAG→UWB 전환 (reset={})', reset_cnt)
                c.mav.vision_position_estimate_send(
                    int(now * 1e6),
                    xy[0], xy[1], 0.0,
                    0.0, 0.0, drone_yaw,
                    _COV_UWB, reset_cnt)
                last_vpe = (xy[0], xy[1], 0.0)
                if not switching:
                    go_to(c, xy[0], xy[1], -HOVER_ALT)
                prev_source = 'uwb'

            elif last_vpe:
                _cov_stale = [0.0] * 21
                _cov_stale[0] = _cov_stale[6] = _cov_stale[11] = 9999.0
                c.mav.vision_position_estimate_send(
                    int(now * 1e6),
                    last_vpe[0], last_vpe[1], last_vpe[2],
                    0.0, 0.0, drone_yaw,
                    _cov_stale, reset_cnt)
                logger.debug('[VPE] 소스 없음 — stale 유지')

        depth_alt = tag.get_depth_alt()
        if depth_alt:
            c.mav.distance_sensor_send(
                0, 10, 1000,
                int(depth_alt * 100),
                0, 0, 25, 0)

        time.sleep(0.05)


def main():
    uwb = FakeUWB()
    logger.info('[UWB] FakeUWB 사용 — origin (0, 0) 고정')

    tag = FakeTagReader()
    tag.start()
    logger.info('[TAG] FakeTagReader 사용 — Tag 미감지 / depth=1.0m 고정')

    c, stop, cache, lock = connect(uwb, start_vision=True, force_arm=True)
    if c is None:
        return

    threading.Thread(
        target=_vision_loop,
        args=(c, uwb, tag, cache, lock, stop),
        daemon=True,
    ).start()

    if not do_takeoff(c, stop, cache, lock):
        do_land(c, stop, cache)
        return

    logger.info('[HOVER] 이륙 완료 — tag 감지 대기. Ctrl+C로 착륙')
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass

    do_land(c, stop, cache)


if __name__ == '__main__':
    main()
