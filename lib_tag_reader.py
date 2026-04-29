"""
tag_reader.py — RealSense D435i + AprilTag pose 추출

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
카메라 물리 장착 정보
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  위치: 드론 하단, 렌즈가 지면을 향하도록 아래로 장착
  회전: 드론 기준 180° 회전 장착 (수평면 기준)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
180° 장착 보정 전략 — 이미지 선회전(pre-rotation)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  카메라가 180° 돌아가 있으므로 raw 이미지도 180° 뒤집혀 있음.
  pose_t를 수식으로 보정하는 대신, 감지 전에 이미지를 180° 돌려
  detector가 항상 "표준 0° 장착"과 동일한 이미지를 보게 만듦.

  장점:
    - NED 변환 수식이 단순해짐 (축 매핑이 직관적)
    - 장착 각도가 바뀌어도 cv2.ROTATE_* 상수 하나만 바꾸면 됨
    - 시각화(tag_test.py)에서 보이는 방향도 실제 드론 방향과 일치

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
표준 0° 장착 기준 축 매핑 (이미지 회전 후 적용)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  회전된 이미지에서 detector가 반환하는 pose_t = (tx, ty, tz):
    tx: 이미지 우측(+X_cam) 방향 거리 → East 방향  → east  = +tx
    ty: 이미지 하단(+Y_cam) 방향 거리 → South 방향 → north = -ty
    tz: depth(카메라→태그)            → Down 방향  → down  = +tz

  결과적으로 반환하는 pose (NED, 태그 기준 드론 상대 위치):
    north = -ty  (태그가 드론 북쪽에 있으면 north > 0)
    east  = +tx  (태그가 드론 동쪽에 있으면 east  > 0)
    down  = +tz  (드론이 태그 위 1m에 있으면 down ≈ 1.0)
    yaw   = atan2(R[1,0], R[0,0])  (이미지 회전 후 추가 보정 불필요)

  → 드론이 태그 정중앙 위에 있으면 north=0, east=0
  → 실물 확인: tag_test.py 실행 후 드론을 태그 북쪽에 놓으면 north>0 인지 검증
"""
import math
import time
import threading

import cv2
import numpy as np
import pyrealsense2 as rs
from pupil_apriltags import Detector
from loguru import logger

TAG_FAMILY = 'tag36h11'
TAG_SIZE_M = 0.16  # 태그 한 변 실제 크기 (m) — 실물 측정값으로 교정 필수


class TagReader:
    """백그라운드 스레드에서 RealSense 카메라를 읽어 AprilTag pose를 갱신.

    get_pose()  → (north, east, down, yaw_rad) or None
    get_frame() → BGR numpy array or None  (tag_test.py 시각화용)

    스레드 안전: get_pose/get_frame은 lock을 통해 원자적으로 읽힘.
    카메라 연결 끊김 시 자동 재시도 (3초 대기 후 pipeline 재시작).
    """

    def __init__(self, tag_size=TAG_SIZE_M, tag_id=None):
        self._tag_size = tag_size
        self._tag_id = tag_id      # None이면 ID 무관하게 가장 신뢰도 높은 태그 사용
        self._lock = threading.Lock()
        self._pose = None          # 태그 미감지 시 None — flight_tag가 UWB로 전환
        self._frame = None         # 오버레이된 BGR 이미지 (tag_test.py 시각화 전용)
        self._latency = (0.0, 0.0) # (detect_ms, total_ms) — 최근 프레임 측정값
        self._detector = Detector(families=TAG_FAMILY)

    def start(self):
        """백그라운드 캡처 스레드 시작. daemon=True이므로 메인 종료 시 자동 종료."""
        threading.Thread(target=self._run, daemon=True).start()

    def get_pose(self):
        """최신 pose를 스레드 안전하게 반환. 태그 미감지 시 None."""
        with self._lock:
            return self._pose

    def get_frame(self):
        """최신 오버레이 프레임 복사본 반환. 프레임 없으면 None."""
        with self._lock:
            return self._frame.copy() if self._frame is not None else None

    def get_latency(self):
        """(detect_ms, total_ms) 반환 — detect: detector.detect() 소요, total: 프레임→pose 전체."""
        with self._lock:
            return self._latency

    def _run(self):
        """카메라 캡처 루프. 예외 발생 시 pipeline을 재시작해 자동 복구."""
        while True:
            # pipeline을 try 바깥에서 생성 → finally에서 항상 stop() 호출 가능
            pipeline = rs.pipeline()
            try:
                cfg = rs.config()
                # 컬러 스트림만 활성화 (depth 스트림 불필요 — 고도는 pose_t.z로 계산)
                cfg.enable_stream(
                    rs.stream.color, 640, 480, rs.format.bgr8, 30)
                profile = pipeline.start(cfg)

                # 카메라 내부 파라미터 (intrinsics) 추출
                # fx, fy: 초점거리(pixel), cx, cy: 주점(principal point, pixel)
                # detector에 전달해야 미터 단위 pose_t를 계산할 수 있음
                intr = (profile.get_stream(rs.stream.color)
                               .as_video_stream_profile()
                               .get_intrinsics())
                fx, fy = intr.fx, intr.fy
                cx, cy = intr.ppx, intr.ppy
                h_px, w_px = 480, 640  # 스트림 해상도

                # 180° 회전 후 주점 보정
                # cv2.rotate(ROTATE_180): 픽셀(x,y) → (w-1-x, h-1-y)
                # 따라서 주점도 동일하게 변환해야 detector가 올바른 pose를 계산함
                # (RealSense cx≈320, cy≈240이라 차이는 ~1px지만 명시적으로 보정)
                cx_rot = (w_px - 1) - cx
                cy_rot = (h_px - 1) - cy
                logger.info(
                    '[TAG] 카메라 연결  fx={:.1f} fy={:.1f}'
                    '  cx={:.1f}→{:.1f}  cy={:.1f}→{:.1f}',
                    fx, fy, cx, cx_rot, cy, cy_rot)

                _frame_count = 0
                while True:
                    t_frame = time.time()
                    frames = pipeline.wait_for_frames(timeout_ms=2000)
                    color = frames.get_color_frame()
                    if not color:
                        continue

                    img = np.asanyarray(color.get_data())  # BGR, (480, 640, 3)

                    # ── 180° 이미지 선회전 (pre-rotation) ───────────────────────
                    # 카메라가 물리적으로 180° 돌아 있으므로 raw 이미지도 뒤집혀 있음.
                    # detector 실행 전에 보정하면 pose_t가 표준 0° 장착과 동일하게 나옴.
                    # → 이후 NED 변환에서 별도 축 변환 수식이 필요 없어짐.
                    # 장착 각도 변경 시 이 상수만 바꾸면 됨:
                    #   0°       → 회전 없음 (이 줄 제거)
                    #   90° CW   → cv2.ROTATE_90_CLOCKWISE
                    #   90° CCW  → cv2.ROTATE_90_COUNTERCLOCKWISE
                    #   180°     → cv2.ROTATE_180  ← 현재
                    img = cv2.rotate(img, cv2.ROTATE_180)
                    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

                    # AprilTag 감지 + pose 추정
                    # cx_rot, cy_rot: 회전된 이미지 기준의 주점 사용
                    t_detect = time.time()
                    dets = self._detector.detect(
                        gray,
                        estimate_tag_pose=True,
                        camera_params=[fx, fy, cx_rot, cy_rot],
                        tag_size=self._tag_size,
                    )
                    detect_ms = (time.time() - t_detect) * 1000
                    total_ms = (time.time() - t_frame) * 1000
                    with self._lock:
                        self._latency = (detect_ms, total_ms)
                    _frame_count += 1
                    if _frame_count % 30 == 1:  # 30프레임마다 1회 출력 (~1초)
                        logger.debug('[TAG] latency  detect={:.1f}ms  total={:.1f}ms',
                                     detect_ms, total_ms)

                    # 특정 ID만 추적할 경우 필터링 (tag_id=None이면 전체 허용)
                    if self._tag_id is not None:
                        dets = [d for d in dets if d.tag_id == self._tag_id]

                    if not dets:
                        # 태그 미감지: pose None 유지 → flight_tag가 UWB VPE로 전환
                        with self._lock:
                            self._pose = None
                            self._frame = img
                        continue

                    # 여러 태그 감지 시 decision_margin(신뢰도)이 가장 높은 것 선택
                    det = max(dets, key=lambda d: d.decision_margin)

                    # pose_t: 카메라 원점 기준 태그까지의 변위 (미터, 카메라 프레임)
                    # pose_R: 카메라 프레임 기준 태그의 회전행렬 (3×3)
                    tx, ty, tz = det.pose_t.flatten()
                    R = det.pose_R

                    # ── 카메라 프레임 → NED 프레임 변환 ────────────────────────
                    # 이미지를 180° 회전했으므로 detector는 표준 0° 장착으로 인식.
                    # 표준 0° 기준 축 매핑 (다운워드 카메라):
                    #   +X_cam (이미지 우) = East  → east  = +tx
                    #   +Y_cam (이미지 하) = South → north = -ty  (South = -North)
                    #   +Z_cam (depth)    = Down  → down  = +tz
                    #
                    # 직관적 검증:
                    #   드론이 태그 북쪽 0.3m에 있다면
                    #   태그는 드론 기준 South → 회전 이미지 하단에 위치 → ty > 0
                    #   north = -ty = -0.3 < 0  (태그가 드론 남쪽) → 맞음 ✓
                    north = -ty
                    east = tx
                    down = tz  # 드론 고도 ≈ tz (지면 태그 기준, 양수)

                    # yaw: 드론의 NED yaw (태그가 정북 정렬돼 있으면 NED 기준과 일치)
                    # atan2(R[1,0], R[0,0]): 이미지 회전 후 태그 Z축 회전각 = 드론 heading
                    yaw = math.atan2(R[1, 0], R[0, 0])

                    # ── 시각화 오버레이 (tag_test.py 전용) ──────────────────────
                    for corner in det.corners.astype(int):
                        cv2.circle(img, tuple(corner), 6, (0, 255, 0), -1)
                    pcx, pcy = int(det.center[0]), int(det.center[1])
                    cv2.drawMarker(img, (pcx, pcy), (0, 0, 255),
                                   cv2.MARKER_CROSS, 20, 2)
                    # flight_tag에 실제 전달되는 NED 값 표시
                    cv2.putText(img,
                                f'N={north:.2f} E={east:.2f} D={down:.2f} '
                                f'yaw={math.degrees(yaw):.1f}deg',
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                                0.6, (0, 255, 255), 2)

                    # 다른 스레드가 읽는 도중 갱신되지 않도록 lock으로 원자적 업데이트
                    with self._lock:
                        self._pose = (north, east, down, yaw)
                        self._frame = img

            except Exception as e:
                logger.error('[TAG] 에러: {} — 3초 후 재시도', e)
                with self._lock:
                    self._pose = None  # 에러 시 pose 무효화 → UWB 폴백 유도
                time.sleep(3)
            finally:
                # pipeline.start() 성공 여부와 무관하게 항상 정리
                try:
                    pipeline.stop()
                except Exception:
                    pass
