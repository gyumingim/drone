"""dbg_tag_test.py — AprilTag 감지 테스트 + flight_tag 투입값 시각화

화면 구성:
  - 태그 코너/중심 (tag_reader 기본 오버레이)
  - 이미지 중심 → 태그 방향 화살표 (드론이 이동해야 할 방향)
  - 하단 패널: flight_tag에서 실제 사용되는 모든 값
"""
import math
import time
import cv2
import numpy as np
from loguru import logger
from lib_tag_reader import TagReader, TAG_SIZE_M

DEADBAND_M = 0.08  # flight_tag.py 와 동일
COV_TAG = 0.002
COV_UWB = 0.05

logger.info('[TEST] tag_size={}m  |  q 키로 종료', TAG_SIZE_M)
reader = TagReader()
reader.start()

logger.info('[TEST] 카메라 초기화 대기...')
time.sleep(2)


def _put(img, text, y, color=(200, 200, 200), scale=0.52):
    cv2.putText(img, text, (10, y),
                cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


def draw_hud(frame, pose, latency, depth_alt, depth_lat):
    h, w = frame.shape[:2]
    PANEL = 202
    detect_ms, total_ms, full_ms = latency
    depth_proc_ms, depth_full_ms = depth_lat

    # 하단 반투명 패널
    roi = frame[h - PANEL:h]
    cv2.addWeighted(np.zeros_like(roi), 0.55, roi, 0.45, 0, roi)
    frame[h - PANEL:h] = roi

    cx, cy_ref = w // 2, (h - PANEL) // 2

    if pose:
        n, e, d, yaw = pose
        dist = math.hypot(n, e)
        in_db = dist < DEADBAND_M

        # 방향 화살표: 이미지 중심 → 드론이 이동해야 할 방향 (NED → 픽셀)
        scale_px = min(w, h - PANEL) * 0.28   # 1m = 화면 28%
        ax = int(cx + e * scale_px)
        ay = int(cy_ref - n * scale_px)
        ax = max(20, min(w - 20, ax))
        ay = max(20, min(h - PANEL - 20, ay))
        color_arrow = (0, 255, 255) if in_db else (0, 165, 255)
        cv2.arrowedLine(frame, (cx, cy_ref), (ax, ay),
                        color_arrow, 3, tipLength=0.25)
        cv2.circle(frame, (cx, cy_ref), 6, (255, 255, 255), -1)
        cv2.putText(frame, 'drone', (cx + 8, cy_ref - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 255, 255), 1)

        # 상태 텍스트 (좌상단)
        label = '★ DEADBAND' if in_db else '● DETECTED'
        cv2.putText(frame, label, (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.78,
                    (0, 255, 255) if in_db else (0, 220, 80), 2, cv2.LINE_AA)

        # 하단 패널
        y0, dy = h - PANEL + 22, 27
        _put(frame,
             f'Pose    N={n:+.3f}m  E={e:+.3f}m  D={d:.3f}m  '
             f'yaw={math.degrees(yaw):+.1f}deg',
             y0, (0, 255, 255))
        _put(frame,
             f'dist2d  {dist:.3f}m  '
             f'{"→ vx=vy=0 (deadband)" if in_db else "→ moving"}',
             y0 + dy, (0, 220, 180))
        _put(frame,
             f'VPE     cov={COV_TAG} (TAG active)',
             y0 + dy * 2, (180, 255, 180))
        _put(frame,
             f'go_to   dN={n:+.3f}m  dE={e:+.3f}m  (tag world 기준)',
             y0 + dy * 3)
        _put(frame,
             f'Depth   alt={depth_alt:.3f}m  proc={depth_proc_ms:.1f}ms  lag={depth_full_ms:.0f}ms'
             if depth_alt else f'Depth   ---  proc={depth_proc_ms:.1f}ms  lag={depth_full_ms:.0f}ms',
             y0 + dy * 4, (180, 255, 130))
        _put(frame,
             f'detect={detect_ms:.1f}ms  total={total_ms:.1f}ms  full={full_ms:.1f}ms',
             y0 + dy * 5, (120, 120, 120))
    else:
        cv2.putText(frame, 'o NOT DETECTED', (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.78, (60, 60, 230), 2,
                    cv2.LINE_AA)

        y0, dy = h - PANEL + 22, 27
        _put(frame, 'Pose    N=---  E=---  D=---  yaw=---', y0, (80, 80, 180))
        _put(frame, 'dist2d  ---', y0 + dy, (80, 80, 180))
        _put(frame,
             f'VPE     cov={COV_UWB} (UWB fallback)',
             y0 + dy * 2, (180, 180, 100))
        _put(frame, 'go_to   --- (tag 미감지)', y0 + dy * 3)
        _put(frame,
             f'Depth   alt={depth_alt:.3f}m  proc={depth_proc_ms:.1f}ms  lag={depth_full_ms:.0f}ms'
             if depth_alt else f'Depth   ---  proc={depth_proc_ms:.1f}ms  lag={depth_full_ms:.0f}ms',
             y0 + dy * 4, (180, 255, 130))
        _put(frame,
             f'detect={detect_ms:.1f}ms  total={total_ms:.1f}ms  full={full_ms:.1f}ms',
             y0 + dy * 5, (120, 120, 120))

    return frame


WIN = 'AprilTag Test — flight_tag preview'
cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
cv2.resizeWindow(WIN, 640, 480)

try:
    while True:
        frame = reader.get_frame()
        pose = reader.get_pose()

        if frame is not None:
            draw_hud(frame, pose, reader.get_latency(), reader.get_depth_alt(), reader.get_depth_latency())
            cv2.imshow(WIN, frame)

        if pose:
            n, e, d, yaw = pose
            logger.info('[TAG] N={:+.3f}m  E={:+.3f}m  D={:.3f}m'
                        '  yaw={:+.1f}deg  dist2d={:.3f}m',
                        n, e, d, math.degrees(yaw), math.hypot(n, e))
        else:
            logger.debug('[TAG] 미감지')

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        time.sleep(0.1)

except KeyboardInterrupt:
    pass
finally:
    cv2.destroyAllWindows()
    logger.info('[TEST] 종료')
