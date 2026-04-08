"""
camera_detector.py — UWB 앵커 카메라 감지 모듈

지원 소스:
  "gz"    — Gazebo Harmonic gz-transport 하향 카메라 (SITL)
  "v4l2"  — V4L2 USB 카메라 (실기체, /dev/videoN)
  "none"  — 비활성화

의존성:
  conda install -c conda-forge opencv pillow
  (Gazebo 사용 시) python3-gz-transport13 / python3-gz-msgs10

protobuf 호환성 주의:
  pip protobuf >= 4.x 는 gz.msgs10 과 충돌함.
  PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python 으로 우회.
"""
import math
import os
import subprocess
import threading

# ── protobuf 호환성 패치 ─────────────────────────────────────────────────────
# uwb_anchor_dropper_sitl.py 최상단에서 이미 설정되지만 단독 사용 시 보장용
os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

try:
    import cv2
    import numpy as np
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

try:
    from PIL import Image as PILImage, ImageTk
    HAS_PIL = True
except ImportError:
    HAS_PIL = False

# ── Gazebo Harmonic gz-transport Python 바인딩 (버전별 자동 탐색) ──────────
HAS_GZ_TRANSPORT = False
_gz_transport = None
_GzImage      = None
_GZ_VERSION   = None

for _v in (13, 14, 12, 11):
    try:
        _gz_transport = __import__(f"gz.transport{_v}", fromlist=["Node"])
        _msgs_v = _v - 3
        _msgs = __import__(f"gz.msgs{_msgs_v}", fromlist=["image_pb2"])
        _GzImage = _msgs.image_pb2.Image
        HAS_GZ_TRANSPORT = True
        print(f"[Camera] gz.transport{_v} + gz.msgs{_msgs_v} 로드 완료")
        break
    except Exception as _e:
        print(f"[Camera] gz.transport{_v} 로드 실패: {_e}")

if not HAS_GZ_TRANSPORT:
    print("[Camera] gz-transport Python 바인딩 없음 (python3-gz-transport13 필요)")


def _gz_topic_list() -> list[str]:
    """현재 Gazebo에서 퍼블리시 중인 토픽 목록 반환."""
    try:
        r = subprocess.run(["gz", "topic", "-l"],
                           capture_output=True, text=True, timeout=3)
        return [t.strip() for t in r.stdout.splitlines() if t.strip()]
    except Exception:
        return []


def _find_image_topic(preferred: str) -> str:
    """
    preferred 토픽이 존재하면 그대로 사용.
    없으면 Gazebo 토픽 목록에서 image 토픽을 자동 탐색.
    """
    topics = _gz_topic_list()
    if preferred in topics:
        return preferred

    # 선호 토픽 없음 → 자동 탐색 (image 포함 토픽)
    image_topics = [t for t in topics if "image" in t.lower()
                    and "camera" not in t.lower().replace("/camera_link", "")]
    if image_topics:
        chosen = image_topics[0]
        print(f"[Camera] 토픽 자동 탐색: {preferred} 없음 → {chosen} 사용")
        return chosen

    # 전체 image 포함 토픽
    image_topics2 = [t for t in topics if "image" in t.lower()]
    if image_topics2:
        chosen = image_topics2[0]
        print(f"[Camera] 토픽 자동 탐색 (fallback): {chosen} 사용")
        return chosen

    print(f"[Camera] 이미지 토픽 없음. 사용 가능한 토픽:\n  " +
          "\n  ".join(topics[:15]) if topics else "  (Gazebo 미실행)")
    return preferred   # 없어도 일단 구독 시도


class AnchorDetector:
    """
    카메라 이미지에서 빨간 UWB 앵커(원통)를 감지한다.
    - Gazebo SITL: gz-transport로 /drone/downward_cam/image 구독
    - 실기체   : OpenCV VideoCapture로 V4L2 카메라 읽기
    """

    DEFAULT_GZ_TOPIC = "/drone/downward_cam/image"
    FOV_DEG          = 90.0   # 카메라 수평 시야각 (degrees)

    def __init__(self, source: str = "gz",
                 gz_topic: str = None,
                 v4l2_device: int = 0):
        self.source   = source if HAS_CV2 else "none"
        self.ok       = False
        self._frame   = None
        self._lock    = threading.Lock()
        self._running = False
        self._cap     = None

        if self.source == "gz":
            if HAS_GZ_TRANSPORT:
                topic = _find_image_topic(gz_topic or self.DEFAULT_GZ_TOPIC)
                self._init_gz(topic)
            else:
                print("[Camera] gz-transport 없음 → 비활성화")
                self.source = "none"
        elif self.source == "v4l2":
            self._init_v4l2(v4l2_device)

    # ── 초기화 ────────────────────────────────────────────────────────────

    def _init_gz(self, topic: str):
        try:
            self._node = _gz_transport.Node()
            self._node.subscribe(_GzImage, topic, self._on_gz_image)
            self.ok = True
            self._gz_topic = topic
            print(f"[Camera] Gazebo 카메라 구독: {topic}")
        except Exception as e:
            print(f"[Camera] Gazebo 초기화 실패: {e}")
            self.source = "none"

    def _init_v4l2(self, device: int):
        self._cap = cv2.VideoCapture(device)
        if not self._cap.isOpened():
            print(f"[Camera] /dev/video{device} 열기 실패")
            self.source = "none"
            return
        self._running = True
        self.ok = True
        threading.Thread(target=self._v4l2_loop, daemon=True).start()
        print(f"[Camera] V4L2 카메라 시작: /dev/video{device}")

    # ── 이미지 수신 ───────────────────────────────────────────────────────

    def _v4l2_loop(self):
        while self._running and self._cap:
            ret, frame = self._cap.read()
            if ret:
                with self._lock:
                    self._frame = frame

    def _on_gz_image(self, msg):
        """gz-transport Image 메시지 콜백."""
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            # pixel_format_type 3 = RGB_INT8 (Gazebo 기본)
            if msg.pixel_format_type == 3:
                img = arr.reshape((msg.height, msg.width, 3))
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            else:
                img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if img is not None:
                with self._lock:
                    self._frame = img
        except Exception:
            pass

    def stop(self):
        self._running = False
        if self._cap:
            self._cap.release()

    def get_frame(self):
        with self._lock:
            return self._frame.copy() if self._frame is not None else None

    # ── 앵커 감지 ─────────────────────────────────────────────────────────

    def detect_anchor(self, frame=None):
        """
        빨간 앵커(원통) 감지.
        반환: (cx, cy, img_w, img_h) 또는 None
        """
        if not HAS_CV2:
            return None
        if frame is None:
            frame = self.get_frame()
        if frame is None:
            return None

        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # 빨간색 = Hue 0-10 and 160-180
        m1   = cv2.inRange(hsv, np.array([0,   100,  80]), np.array([10,  255, 255]))
        m2   = cv2.inRange(hsv, np.array([160, 100,  80]), np.array([180, 255, 255]))
        mask = cv2.bitwise_or(m1, m2)
        k    = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        c = max(contours, key=cv2.contourArea)
        if cv2.contourArea(c) < 100:
            return None
        M = cv2.moments(c)
        if M["m00"] == 0:
            return None
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        return (cx, cy, frame.shape[1], frame.shape[0])

    def annotate(self, frame, detection=None):
        """감지 결과를 프레임에 그려서 반환."""
        if not HAS_CV2 or frame is None:
            return frame
        out = frame.copy()
        h, w = out.shape[:2]
        # 중심 십자선
        cv2.line(out, (w//2 - 20, h//2), (w//2 + 20, h//2), (0, 255, 0), 1)
        cv2.line(out, (w//2, h//2 - 20), (w//2, h//2 + 20), (0, 255, 0), 1)
        if detection:
            cx, cy = detection[0], detection[1]
            cv2.circle(out, (cx, cy), 14, (0, 100, 255), 2)
            cv2.putText(out, "ANCHOR", (cx + 8, cy - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 180, 255), 1)
            cv2.line(out, (w//2, h//2), (cx, cy), (0, 200, 255), 1)
        return out

    def get_tk_image(self, frame=None, w: int = 200, h: int = 150):
        """tkinter PhotoImage 반환 (PIL 필요). GC 방지를 위해 참조를 저장할 것."""
        if not HAS_CV2 or not HAS_PIL:
            return None
        if frame is None:
            frame = self.get_frame()
        if frame is None:
            return None
        small = cv2.resize(frame, (w, h))
        rgb   = cv2.cvtColor(small, cv2.COLOR_BGR2RGB)
        return ImageTk.PhotoImage(PILImage.fromarray(rgb))

    # ── 좌표 변환 ─────────────────────────────────────────────────────────

    @staticmethod
    def pixel_to_ned_offset(cx: int, cy: int,
                             img_w: int, img_h: int,
                             altitude: float,
                             fov_deg: float = 90.0):
        """
        카메라 이미지 픽셀 오프셋 → NED 미터 오프셋 변환.

        가정: 하향 카메라, 드론 yaw = 0 (North 정렬)
          픽셀 x+ = East+
          픽셀 y+ = South (= North-)

        altitude: 현재 고도 양수 미터 (abs 처리됨)
        """
        dx = cx - img_w / 2
        dy = cy - img_h / 2
        # 이미지 너비 기준 실제 미터/픽셀 비율
        scale = 2 * abs(altitude) * math.tan(math.radians(fov_deg / 2)) / img_w
        dn = -dy * scale   # 픽셀 y↓ = 남쪽 = North-
        de =  dx * scale   # 픽셀 x→ = East+
        return dn, de
