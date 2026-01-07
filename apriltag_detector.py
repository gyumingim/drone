"""
AprilTag Detector - Isaac Sim 카메라 이미지에서 AprilTag 감지
"""
import numpy as np
import cv2
from pupil_apriltags import Detector


class AprilTagDetector:
    """AprilTag 감지 및 상대 위치 추정"""

    def __init__(self, tag_size=0.5, camera_fov_deg=150.0, image_resolution=(1280, 720)):
        """
        초기화

        Args:
            tag_size: AprilTag 실제 크기 (미터)
            camera_fov_deg: 카메라 시야각 (도)
            image_resolution: 이미지 해상도 (width, height)
        """
        self.tag_size = tag_size
        self.img_w, self.img_h = image_resolution

        # 카메라 내부 파라미터 계산
        self.fx = self.img_w / (2 * np.tan(np.radians(camera_fov_deg / 2)))
        self.fy = self.fx
        self.cx = self.img_w / 2
        self.cy = self.img_h / 2

        # AprilTag 검출기
        self.detector = Detector(
            families="tag36h11",
            nthreads=4,
            quad_decimate=2.0,
            quad_sigma=0.0,
            refine_edges=True,
            decode_sharpening=0.25
        )

        print(f"[AprilTag] Detector initialized (FOV={camera_fov_deg}°, fx={self.fx:.1f})")

    def detect(self, image):
        """
        이미지에서 AprilTag 감지

        Args:
            image: RGBA 이미지 (numpy array)

        Returns:
            dict or None: {
                'position': np.array([x, y, z]),  # Body frame (m)
                'distance': float,  # 거리 (m)
                'detected': bool
            }
        """
        if image is None or image.size == 0:
            return {'position': np.zeros(3), 'distance': 0.0, 'detected': False}

        try:
            # RGB → Grayscale
            if len(image.shape) == 3 and image.shape[2] >= 3:
                gray = cv2.cvtColor(image[:, :, :3].astype(np.uint8), cv2.COLOR_RGB2GRAY)
            else:
                gray = image[:, :, 0].astype(np.uint8) if len(image.shape) == 3 else image.astype(np.uint8)

            # CLAHE (대비 향상)
            clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
            enhanced = clahe.apply(gray)
            enhanced = cv2.GaussianBlur(enhanced, (3, 3), 0)

            # AprilTag 검출
            tags = self.detector.detect(
                enhanced,
                estimate_tag_pose=True,
                camera_params=[self.fx, self.fy, self.cx, self.cy],
                tag_size=self.tag_size
            )

            if tags:
                tag = tags[0]  # 첫 번째 태그 사용

                # 카메라 좌표계 → 드론 Body 좌표계 변환
                # 카메라: +X=오른쪽, +Y=아래, +Z=앞
                # Body: +X=앞, +Y=오른쪽, +Z=아래 (NED)
                tag_camera = np.array([
                    tag.pose_t[2][0],   # Z(카메라 앞) → X(드론 앞)
                    tag.pose_t[0][0],   # X(카메라 오른쪽) → Y(드론 오른쪽)
                    tag.pose_t[1][0]    # Y(카메라 아래) → Z(드론 아래)
                ])

                distance = np.linalg.norm(tag_camera)

                return {
                    'position': tag_camera,
                    'distance': distance,
                    'detected': True
                }

        except Exception as e:
            print(f"[AprilTag] Detection error: {e}")

        return {'position': np.zeros(3), 'distance': 0.0, 'detected': False}
