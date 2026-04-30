"""
flight_tag_airborne.py — 이미 공중에 있는 드론에서 AprilTag 단독 호버링

do_takeoff() 없이 시작, UWB 불필요.
  - tag 인식 → VPE 전송 + go_to(0,0) 수렴
  - tag 미인식 → stale VPE 유지 (EKF timeout 방지)
  - Ctrl+C → 착륙
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


class _UWBStub:
    """connect() 내부 EKF init 중 pos_rel 충족용 정적 VPE 소스."""
    def get_xy(self):
        return (0.0, 0.0)

_COV_TAG = [0.0] * 21
_COV_TAG[0] = _COV_TAG[6] = _COV_TAG[11] = 0.002

_WIN = 'flight_tag_airborne — camera'


def _put(img, text, y, color=(200, 200, 200), scale=0.52):
    cv2.putText(img, text, (10, y),
                cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


def _draw_hud(frame, pose, latency, depth_alt, depth_lat, cache, lock):
    h, w = frame.shape[:2]
    PANEL = 230
    detect_ms, total_ms, full_ms = latency
    depth_proc_ms, depth_full_ms = depth_lat

    with lock:
        srv  = cache['servo']

    intent  = interpret_flight(srv)
    dep_str = f'{depth_alt:.2f}m' if depth_alt else '0.00m'

    roi = frame[h - PANEL:h]
    cv2.addWeighted(np.zeros_like(roi), 0.55, roi, 0.45, 0, roi)
    frame[h - PANEL:h] = roi

    cx, cy_ref = w // 2, (h - PANEL) // 2

    if pose:
        n, e, d, yaw = pose
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

    cv2.putText(frame, f'{intent}  depth={dep_str}  [AIRBORNE]',
                (w - 320, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                (0, 255, 255), 1, cv2.LINE_AA)

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
    prev_source = None
    reset_cnt   = 0
    last_vpe    = None
    _srv_tick   = 0

    while not stop.is_set():
        pose = tag.get_pose()
        now  = time.time()

        with lock:
            att = cache['attitude']
        drone_yaw = att.yaw if att else 0.0

        if pose:
            n, e, d, tag_yaw = pose
            drone_yaw = tag_yaw

            if prev_source != 'tag':
                reset_cnt = (reset_cnt + 1) % 256
                logger.info('[VPE] →TAG (reset={})', reset_cnt)

            c.mav.vision_position_estimate_send(
                int(now * 1e6),
                -n, -e, -HOVER_ALT,
                0.0, 0.0, tag_yaw,
                _COV_TAG, reset_cnt)

            last_vpe = (-n, -e, -HOVER_ALT)
            go_to(c, 0, 0, -HOVER_ALT)
            prev_source = 'tag'

        elif last_vpe:
            _cov_stale = [0.0] * 21
            _cov_stale[0] = _cov_stale[6] = _cov_stale[11] = 9999.0
            c.mav.vision_position_estimate_send(
                int(now * 1e6),
                last_vpe[0], last_vpe[1], last_vpe[2],
                0.0, 0.0, drone_yaw,
                _cov_stale, reset_cnt)
            logger.debug('[VPE] stale (태그 없음)')

        # DISTANCE_SENSOR (rangefinder 고도 소스)
        depth_alt = tag.get_depth_alt()
        d_cm = int(depth_alt * 100) if depth_alt else 10
        c.mav.distance_sensor_send(0, 10, 1000, d_cm, 0, 0, 25, 0)
        with lock:
            cache['depth'] = depth_alt

        # 2Hz 상태 로그
        _srv_tick += 1
        if _srv_tick >= 10:
            _srv_tick = 0
            with lock:
                srv  = cache['servo']
            intent = interpret_flight(srv)
            src = 'TAG' if prev_source == 'tag' else '---'
            dep = f'{depth_alt:.2f}m' if depth_alt else '0.00m'
            if srv:
                logger.info('[STATUS] {} | src={} depth={} | srv={} {} {} {}',
                            intent, src, dep,
                            srv.servo1_raw, srv.servo2_raw,
                            srv.servo3_raw, srv.servo4_raw)
            else:
                logger.info('[STATUS] {} | src={} depth={}',
                            intent, src, dep)

        time.sleep(0.05)  # 20Hz


def main():
    tag = TagReader()
    tag.start()
    logger.info('[TAG] 카메라 초기화...')
    time.sleep(1)

    # EKF init 중 pos_rel 충족을 위해 stub VPE(0,0) 전송
    # connect() 반환 후 vision_pause=True로 중단하고 tag 루프로 교체
    c, stop, cache, lock = connect(_UWBStub(), start_vision=True)
    if c is None:
        return

    with lock:
        cache['vision_pause'] = True  # lib_common _vision_loop 중단

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

    # FC flying state 전환 — 이미 공중이면 EKF z 즉시 목표 도달 판정
    if not do_takeoff(c, stop, cache, lock):
        do_land(c, stop, cache)
        return

    with lock:
        cache['airborne'] = True
    logger.info('[AIRBORNE] tag 추적 시작')

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass

    do_land(c, stop, cache)


if __name__ == '__main__':
    main()
