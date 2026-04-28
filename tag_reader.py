"""
tag_reader.py — RealSense D435i + AprilTag pose 추출

카메라 좌표계 (렌즈 아래 장착 기준):
  Camera X_optical = 드론 East  (오른쪽)
  Camera Y_optical = 드론 South (뒤쪽)  ← 마운트 방향 따라 조정
  Camera Z_optical = 드론 Down  (아래, tz ≈ 고도)

반환 pose (NED, tag 기준 상대값):
  north = -ty  (tag이 카메라 뒤면 +N → 드론이 tag보다 north에 있다)
  east  = +tx
  down  = +tz  (≈ 고도, 양수)
  yaw   = tag의 yaw in camera frame

  → tag 정중앙 위에 있으려면 north=0, east=0 이 되어야 함
"""
import math
import time
import threading

import cv2
import numpy as np
import pyrealsense2 as rs
from pupil_apriltags import Detector

TAG_FAMILY = 'tag36h11'
TAG_SIZE_M = 0.16    # 태그 한 변 실제 크기 (m)


class TagReader:
    """
    get_pose()  → (north, east, down, yaw_rad) or None
    get_frame() → BGR numpy array or None  (시각화용)
    """

    def __init__(self, tag_size=TAG_SIZE_M, tag_id=None):
        self._tag_size  = tag_size
        self._tag_id    = tag_id    # None = 아무 tag나 감지
        self._lock      = threading.Lock()
        self._pose      = None
        self._frame     = None
        self._detector  = Detector(families=TAG_FAMILY)

    def start(self):
        threading.Thread(target=self._run, daemon=True).start()

    def get_pose(self):
        with self._lock:
            return self._pose

    def get_frame(self):
        with self._lock:
            return self._frame.copy() if self._frame is not None else None

    def _run(self):
        while True:
            try:
                pipeline = rs.pipeline()
                cfg      = rs.config()
                cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
                profile  = pipeline.start(cfg)

                intr = (profile.get_stream(rs.stream.color)
                               .as_video_stream_profile()
                               .get_intrinsics())
                fx, fy, cx, cy = intr.fx, intr.fy, intr.ppx, intr.ppy
                print(f'[TAG] 카메라 연결  fx={fx:.1f} fy={fy:.1f}')

                while True:
                    frames = pipeline.wait_for_frames(timeout_ms=2000)
                    color  = frames.get_color_frame()
                    if not color:
                        continue

                    img  = np.asanyarray(color.get_data())
                    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

                    dets = self._detector.detect(
                        gray,
                        estimate_tag_pose=True,
                        camera_params=[fx, fy, cx, cy],
                        tag_size=self._tag_size,
                    )

                    if self._tag_id is not None:
                        dets = [d for d in dets if d.tag_id == self._tag_id]

                    if not dets:
                        with self._lock:
                            self._pose  = None
                            self._frame = img
                        continue

                    det = max(dets, key=lambda d: d.decision_margin)
                    tx, ty, tz = det.pose_t.flatten()
                    R           = det.pose_R

                    # NED 변환
                    north = -ty
                    east  =  tx
                    down  =  tz
                    yaw   = math.atan2(R[1, 0], R[0, 0])

                    # 오버레이
                    for corner in det.corners.astype(int):
                        cv2.circle(img, tuple(corner), 6, (0, 255, 0), -1)
                    pcx, pcy = int(det.center[0]), int(det.center[1])
                    cv2.drawMarker(img, (pcx, pcy), (0, 0, 255),
                                   cv2.MARKER_CROSS, 20, 2)
                    cv2.putText(img,
                                f'N={north:.2f} E={east:.2f} D={down:.2f} '
                                f'yaw={math.degrees(yaw):.1f}deg',
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                                0.6, (0, 255, 255), 2)

                    with self._lock:
                        self._pose  = (north, east, down, yaw)
                        self._frame = img

            except Exception as e:
                print(f'[TAG] 에러: {e} — 3초 후 재시도')
                with self._lock:
                    self._pose = None
                time.sleep(3)
