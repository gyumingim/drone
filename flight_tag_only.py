"""
flight_tag_only.py — Tag only 호버링 (UWB 없음)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
동작 순서
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  1. 카메라 초기화 (RealSense 파이프라인 시작)
  2. FC 연결 → EKF 준비 → GUIDED 모드 → ARM
  3. _vision_loop 시작 → tag 없을 시 cov=9999 VPE 전송 (EKF timeout 방지)
  4. 이륙 1m
  5. Tag 감지 시 Tag VPE (cov=0.002) + go_to(0,0) 으로 수렴
  6. Ctrl+C → 착륙

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
VPE(VISION_POSITION_ESTIMATE) 전략
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  [Tag 감지 시]
    - 좌표계: 태그=(0,0) origin, 드론 위치=(-n, -e)
    - EKF에 태그 기준 드론 위치 전송 → go_to(0,0) → 태그 정중앙 수렴
    - covariance 낮음(0.002) → EKF가 tag 위치를 높은 신뢰도로 수용

  [Tag 미감지 시]
    - cov=9999 VPE 전송 → posErr=100m(clamp) → EKF Kalman gain≈0 → 무시
    - IMU+baro만으로 유지, XY drift 허용
    - EKF timeout(약 5초) 방지용

  [Tag 전환 시]
    - reset_counter 증가 → EKF에게 "좌표계 바뀜" 신호
    - EKF가 이전 위치 추정을 버리고 새 좌표계를 즉시 수용

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
참조
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  ardupilot.org/dev/docs/mavlink-nongps-position-estimation.html
  mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE
  AP_VisualOdom_MAV.cpp: posErr = constrain_float(posErr, VISO_POS_M_NSE, 100.0)
"""
import math
import time
import threading

import cv2
import numpy as np
from loguru import logger
from lib_tag_reader import TagReader
from lib_common import connect, do_takeoff, do_land, go_to, interpret_flight, TAKEOFF_M

HOVER_ALT = TAKEOFF_M

# TAG: ±4.5cm 오차 → variance = 0.045² ≈ 0.002
_COV_TAG = [0.0] * 21
_COV_TAG[0] = _COV_TAG[6] = _COV_TAG[11] = 0.002

# tag 없을 때: posErr=100m(clamp) → EKF Kalman gain≈0 → XY 무시
_COV_STALE = [0.0] * 21
_COV_STALE[0] = _COV_STALE[6] = _COV_STALE[11] = 9999.0

_WIN = 'flight_tag_only — camera'


class _NullUWB:
    """UWB 없는 환경에서 lib_common connect()에 전달하는 stub."""
    def get_xy(self): return (0.0, 0.0)
    def start(self): pass


def _put(img, text, y, color=(200, 200, 200), scale=0.52):
    cv2.putText(img, text, (10, y),
                cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


def _draw_hud(frame, pose, latency, depth_alt, depth_lat, cache, lock):
    h, w = frame.shape[:2]
    PANEL = 230
    detect_ms, total_ms, full_ms = latency
    depth_proc_ms, depth_full_ms = depth_lat

    with lock:
        srv      = cache['servo']
        airborne = cache['airborne']

    intent   = interpret_flight(srv)
    dep_str  = f'{depth_alt:.2f}m' if depth_alt else '0.00m'
    arm_str  = 'AIRBORNE' if airborne else 'GROUND'

    roi = frame[h - PANEL:h]
    cv2.addWeighted(np.zeros_like(roi), 0.55, roi, 0.45, 0, roi)
    frame[h - PANEL:h] = roi

    cx, cy_ref = w // 2, (h - PANEL) // 2

    if pose:
        n, e, d, yaw = pose
        dist = math.hypot(n, e)
        scale_px = min(w, h - PANEL) * 0.28
        ax = int(cx + e * scale_px)
        ay = int(cy_ref - n * scale_px)
        ax = max(20, min(w - 20, ax))
        ay = max(20, min(h - PANEL - 20, ay))
        cv2.arrowedLine(frame, (cx, cy_ref), (ax, ay), (0, 165, 255), 3, tipLength=0.25)
        cv2.circle(frame, (cx, cy_ref), 6, (255, 255, 255), -1)
        cv2.putText(frame, '● TAG', (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.78, (0, 220, 80), 2, cv2.LINE_AA)
    else:
        cv2.putText(frame, 'o NO TAG', (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.78, (60, 60, 230), 2, cv2.LINE_AA)

    cv2.putText(frame, f'{intent}  depth={dep_str}  [{arm_str}]',
                (w - 320, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                (0, 255, 255) if airborne else (180, 180, 180), 1, cv2.LINE_AA)

    y0, dy = h - PANEL + 22, 27
    if pose:
        n, e, d, yaw = pose
        dist = math.hypot(n, e)
        _put(frame, f'Pose   N={n:+.3f}m  E={e:+.3f}m  D={d:.3f}m  yaw={math.degrees(yaw):+.1f}deg',
             y0, (0, 255, 255))
        _put(frame, f'dist2d {dist:.3f}m', y0 + dy, (0, 220, 180))
    else:
        _put(frame, 'Pose   N=---  E=---  D=---', y0, (80, 80, 180))
        _put(frame, 'dist2d ---', y0 + dy, (80, 80, 180))

    dep_str = (f'alt={depth_alt:.3f}m  proc={depth_proc_ms:.1f}ms  lag={depth_full_ms:.0f}ms'
               if depth_alt else f'---  proc={depth_proc_ms:.1f}ms')
    _put(frame, f'Depth  {dep_str}', y0 + dy * 2, (180, 255, 130))

    if srv:
        avg = (srv.servo1_raw + srv.servo2_raw + srv.servo3_raw + srv.servo4_raw) / 4
        _put(frame,
             f'Servo  {srv.servo1_raw} {srv.servo2_raw} {srv.servo3_raw} {srv.servo4_raw}'
             f'  avg={avg:.0f}  → {intent}',
             y0 + dy * 3, (255, 200, 100))
    else:
        _put(frame, 'Servo  ---', y0 + dy * 3, (100, 100, 100))

    _put(frame, f'detect={detect_ms:.1f}ms  total={total_ms:.1f}ms  full={full_ms:.1f}ms',
         y0 + dy * 4, (120, 120, 120))

    return frame


def _display_loop(tag, cache, lock, stop):
    cv2.namedWindow(_WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(_WIN, 640, 480)
    while not stop.is_set():
        frame = tag.get_frame()
        if frame is not None:
            _draw_hud(frame, tag.get_pose(), tag.get_latency(),
                      tag.get_depth_alt(), tag.get_depth_latency(),
                      cache, lock)
            cv2.imshow(_WIN, frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            stop.set()
            break
        time.sleep(0.033)
    cv2.destroyAllWindows()


def _vision_loop(c, tag, cache, lock, stop):
    """20Hz VPE 전송 루프 — Tag 감지 시 low-cov, 미감지 시 cov=9999(EKF timeout 방지).

    UWB 없으므로 tag 미감지 구간은 IMU+baro drift 허용.
    """
    prev_source = 'none'
    reset_cnt = 0
    _srv_tick = 0

    while not stop.is_set():
        pose = tag.get_pose()
        now = time.time()

        with lock:
            att      = cache['attitude']
            airborne = cache['airborne']
        drone_yaw = att.yaw if att else 0.0

        if pose:
            # ── Tag 감지: tag origin 기준 VPE ──────────────────────────────
            n, e, d, tag_yaw = pose
            drone_yaw = tag_yaw

            if prev_source != 'tag':
                reset_cnt = (reset_cnt + 1) % 256
                logger.info('[VPE] →TAG 전환 (reset={})', reset_cnt)

            c.mav.vision_position_estimate_send(
                int(now * 1e6),
                -n, -e, -HOVER_ALT,
                0.0, 0.0, tag_yaw,
                _COV_TAG, reset_cnt)

            if airborne:
                go_to(c, 0, 0, -HOVER_ALT)
            prev_source = 'tag'

        else:
            # ── Tag 미감지: cov=9999 → posErr=100m(clamp) → EKF 사실상 무시 ──
            # VPE를 끊으면 EKF timeout(~5초) → position lost failsafe 가능
            if prev_source == 'tag':
                reset_cnt = (reset_cnt + 1) % 256
                logger.info('[VPE] TAG 소실 → stale (reset={})', reset_cnt)

            c.mav.vision_position_estimate_send(
                int(now * 1e6),
                0.0, 0.0, -HOVER_ALT,
                0.0, 0.0, drone_yaw,
                _COV_STALE, reset_cnt)
            prev_source = 'none'

        # ── DISTANCE_SENSOR 전송 (EK3_SRC1_POSZ=2 Rangefinder 고도 소스) ──
        depth_alt = tag.get_depth_alt()
        d_cm = int(depth_alt * 100) if depth_alt else 10
        c.mav.distance_sensor_send(0, 10, 1000, d_cm, 0, 0, 25, 0)
        with lock:
            cache['depth'] = depth_alt

        # ── 2Hz 상태 출력 ──────────────────────────────────────────────────
        _srv_tick += 1
        if _srv_tick >= 10:
            _srv_tick = 0
            with lock:
                srv = cache['servo']
            intent = interpret_flight(srv)
            src = 'TAG' if prev_source == 'tag' else '---'
            dep = f'{depth_alt:.2f}m' if depth_alt else '0.00m'
            if srv:
                logger.info('[STATUS] {} | src={} depth={} | srv={} {} {} {}',
                            intent, src, dep,
                            srv.servo1_raw, srv.servo2_raw,
                            srv.servo3_raw, srv.servo4_raw)
            else:
                logger.info('[STATUS] {} | src={} depth={}', intent, src, dep)

        time.sleep(0.05)  # 20Hz


def main():
    # ── 카메라 초기화 ─────────────────────────────────────────────────────────
    tag = TagReader()
    tag.start()
    logger.info('[TAG] 카메라 초기화...')
    time.sleep(1)

    # ── FC 연결 및 준비 ───────────────────────────────────────────────────────
    # _NullUWB: lib_common _vision_loop이 VPE(0,0, cov=1.0)를 잠깐 전송 → EKF 초기화
    c, stop, cache, lock = connect(_NullUWB(), start_vision=True)
    if c is None:
        return

    # ── VPE 루프 교체 ─────────────────────────────────────────────────────────
    with lock:
        cache['vision_pause'] = True
    threading.Thread(
        target=_vision_loop,
        args=(c, tag, cache, lock, stop),
        daemon=True,
    ).start()
    threading.Thread(
        target=_display_loop,
        args=(tag, cache, lock, stop),
        daemon=True,
    ).start()

    # ── 이륙 ─────────────────────────────────────────────────────────────────
    if not do_takeoff(c, stop, cache, lock):
        do_land(c, stop, cache)
        return

    with lock:
        cache['airborne'] = True
    logger.info('[HOVER] 이륙 완료 — tag 감지 대기')

    # ── 메인 루프: Ctrl+C 대기 ───────────────────────────────────────────────
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass

    do_land(c, stop, cache)


if __name__ == '__main__':
    main()
