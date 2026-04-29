"""
tag_reader.py — RealSense D435i + AprilTag pose 추출

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
카메라 물리 장착 정보
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  위치: 드론 하단, 렌즈가 지면을 향하도록 아래로 장착
  회전: 드론 기준 180° 회전 장착 (수평면 기준, Z축 회전)

  회전 전(0° 표준):  X_cam(이미지 우) → East,  Y_cam(이미지 하) → South
  180° 회전 후:      X_cam(이미지 우) → West,  Y_cam(이미지 하) → North
                     Z_cam(depth)     → Down   (회전 무관하게 항상 아래)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
카메라 프레임 → NED 프레임 변환 (180° 장착 기준)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  pupil_apriltags가 반환하는 pose_t = (tx, ty, tz):
    tx: 카메라 기준 태그까지의 X_cam 방향 거리 (이미지 우측 방향)
    ty: 카메라 기준 태그까지의 Y_cam 방향 거리 (이미지 아래 방향)
    tz: 카메라 기준 태그까지의 Z_cam 방향 거리 (depth, 지면 방향)

  180° 회전 시 축 매핑:
    X_cam → West(-East) 방향이므로 east = -tx
    Y_cam → North       방향이므로 north = ty
    Z_cam → Down        방향이므로 down  = tz

  2D 회전행렬(180°) 검증:
    표준: X_cam=(1,0)→East,  Y_cam=(0,-1)→South
    R(180°) * (1,0)  = (-1,0) = West  → east  = -tx  ✓
    R(180°) * (0,-1) = (0,1)  = North → north = ty   ✓

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
반환 pose (NED, 태그 기준 드론 상대 위치)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  north =  ty   (태그가 드론 북쪽에 있으면 north > 0)
  east  = -tx   (태그가 드론 동쪽에 있으면 east  > 0)
  down  =  tz   (고도, 드론이 태그 위 1m면 down ≈ 1.0)
  yaw   = atan2(R[1,0], R[0,0]) + π  (180° 장착 보정)

  → 드론이 태그 정중앙 위에 있으면 north=0, east=0
  → 실물 확인: tag_test.py 실행 후 드론을 태그 북쪽에 놓으면 north>0인지 반드시 검증
"""
import math
import time
import threading

import cv2
import numpy as np
import pyrealsense2 as rs
from pupil_apriltags import Detector

TAG_FAMILY = 'tag36h11'
TAG_SIZE_M = 0.16    # 태그 한 변 실제 크기 (m) — 실물 측정값으로 교정 필수


class TagReader:
    """백그라운드 스레드에서 RealSense 카메라를 읽어 AprilTag pose를 갱신.

    get_pose()  → (north, east, down, yaw_rad) or None
    get_frame() → BGR numpy array or None  (tag_test.py 시각화용)

    스레드 안전: get_pose/get_frame은 lock을 통해 원자적으로 읽힘.
    카메라 연결 끊김 시 자동 재시도 (3초 대기 후 pipeline 재시작).
    """

    def __init__(self, tag_size=TAG_SIZE_M, tag_id=None):
        self._tag_size = tag_size
        self._tag_id   = tag_id   # None이면 ID 무관하게 가장 신뢰도 높은 태그 사용
        self._lock     = threading.Lock()
        self._pose     = None     # 태그 미감지 시 None 유지
        self._frame    = None     # 오버레이된 BGR 이미지 (시각화 전용)
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

    def _run(self):
        """카메라 캡처 루프. 예외 발생 시 pipeline을 재시작해 복구."""
        while True:
            # pipeline을 try 블록 바깥에서 생성해 finally에서 항상 stop 가능하게 함
            pipeline = rs.pipeline()
            try:
                cfg = rs.config()
                # 컬러 스트림만 사용 (depth 불필요 — pose_t의 tz로 고도 계산)
                cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
                profile = pipeline.start(cfg)

                # 카메라 내부 파라미터 추출 — AprilTag pose 추정에 필수
                # fx, fy: 초점거리(px),  cx, cy: 주점(이미지 중심)
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

                    # AprilTag 감지 + pose 추정
                    # camera_params와 tag_size가 있어야 미터 단위 pose_t 반환
                    dets = self._detector.detect(
                        gray,
                        estimate_tag_pose=True,
                        camera_params=[fx, fy, cx, cy],
                        tag_size=self._tag_size,
                    )

                    # 특정 ID만 추적하는 경우 필터링 (None이면 전체 허용)
                    if self._tag_id is not None:
                        dets = [d for d in dets if d.tag_id == self._tag_id]

                    if not dets:
                        # 태그 미감지: pose를 None으로 초기화
                        # flight_tag.py의 _vision_loop가 None을 보면 UWB로 전환
                        with self._lock:
                            self._pose  = None
                            self._frame = img
                        continue

                    # 여러 태그 감지 시 decision_margin(감지 신뢰도)이 가장 높은 것 선택
                    det = max(dets, key=lambda d: d.decision_margin)

                    # pose_t: 카메라 원점 기준 태그까지의 변위 (미터)
                    # pose_R: 태그의 카메라 기준 회전행렬 (3x3)
                    tx, ty, tz = det.pose_t.flatten()
                    R           = det.pose_R

                    # ── 카메라 프레임 → NED 프레임 변환 (180° 장착) ─────────────
                    # 180° 회전 장착 시 축 방향:
                    #   X_cam(이미지 우) → West(-East)  →  east  = -tx
                    #   Y_cam(이미지 하) → North         →  north = ty
                    #   Z_cam(depth)    → Down           →  down  = tz (부호 유지)
                    #
                    # 검증: 드론이 태그 북쪽 0.3m에 있을 때
                    #   태그는 드론 기준 South(-North)에 있음
                    #   → ty가 남쪽 방향 → ty < 0이 나와야 north < 0
                    #   → tag_test.py로 실물 확인 필수
                    north =  ty
                    east  = -tx
                    down  =  tz  # 드론 고도 ≈ tz (지면 태그 기준, 양수)

                    # yaw: 태그가 카메라 기준으로 얼마나 회전했는지
                    # atan2(R[1,0], R[0,0]): 카메라 프레임 기준 태그 회전각
                    # + π: 180° 장착 보정 (카메라가 180° 돌아 있으므로 역방향 보정)
                    # flight_tag.py에서 이 yaw는 현재 미사용 (드론 heading은 att.yaw 사용)
                    yaw = math.atan2(R[1, 0], R[0, 0]) + math.pi

                    # ── 시각화 오버레이 (tag_test.py 전용) ──────────────────────
                    # 태그 코너에 초록 원 표시
                    for corner in det.corners.astype(int):
                        cv2.circle(img, tuple(corner), 6, (0, 255, 0), -1)
                    # 태그 중심에 빨간 십자 표시
                    pcx, pcy = int(det.center[0]), int(det.center[1])
                    cv2.drawMarker(img, (pcx, pcy), (0, 0, 255),
                                   cv2.MARKER_CROSS, 20, 2)
                    # 변환된 NED 값 화면 출력 (flight_tag에 실제 들어가는 값)
                    cv2.putText(img,
                                f'N={north:.2f} E={east:.2f} D={down:.2f} '
                                f'yaw={math.degrees(yaw):.1f}deg',
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                                0.6, (0, 255, 255), 2)

                    # 다른 스레드가 읽는 동안 갱신되지 않도록 lock으로 원자적 업데이트
                    with self._lock:
                        self._pose  = (north, east, down, yaw)
                        self._frame = img

            except Exception as e:
                print(f'[TAG] 에러: {e} — 3초 후 재시도')
                with self._lock:
                    self._pose = None  # 에러 시 pose 무효화 → UWB 폴백 유도
                time.sleep(3)
            finally:
                # pipeline.start() 성공 여부와 무관하게 항상 정리
                # start() 실패했어도 stop() 호출해도 안전 (내부적으로 무시됨)
                try:
                    pipeline.stop()
                except Exception:
                    pass
