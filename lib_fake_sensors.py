"""
lib_fake_sensors.py — SITL용 가짜 센서

UDP port 14560으로 "north,east,alt\n" 수신 → XY/Z 실시간 제어
sitl_viz.py 슬라이더에서 전송.

FakeUWB:       noise_m > 0 이면 가우시안 노이즈 추가
FakeTagReader: detect_rate로 tag 감지 확률 조절
"""
import random
import socket
import threading

UDP_CTRL_PORT = 14560

_state = {'x': 0.0, 'y': 0.0, 'z': 1.0}
_state_lock = threading.Lock()


def start_udp_control(port=UDP_CTRL_PORT):
    """UDP 수신 스레드 시작. sitl_viz.py 슬라이더 값 반영."""
    def _recv():
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('', port))
        while True:
            try:
                data, _ = sock.recvfrom(64)
                x, y, z = map(float, data.decode().strip().split(','))
                with _state_lock:
                    _state['x'], _state['y'], _state['z'] = x, y, z
            except Exception:
                pass
    threading.Thread(target=_recv, daemon=True).start()


class FakeUWB:
    """SITL용 가짜 UWB.

    noise_m=0.0 : UDP 슬라이더 값 그대로 반환
    noise_m=0.3 : ±30cm 가우시안 노이즈 추가 (멀티패스 시뮬레이션)
    """

    def __init__(self, noise_m: float = 0.0):
        self._noise = noise_m

    def start(self): pass

    def get_xy(self):
        with _state_lock:
            x, y = _state['x'], _state['y']
        if self._noise > 0:
            return (
                x + random.gauss(0.0, self._noise),
                y + random.gauss(0.0, self._noise),
            )
        return (x, y)


class FakeTagReader:
    """SITL용 가짜 카메라.

    detect_rate=0.0 : 항상 Tag 미감지 → UWB fallback
    detect_rate=1.0 : 항상 Tag 감지 → tag 수렴 테스트
    alt_m           : 초기 고도 (UDP로 실시간 변경 가능)
    """

    def __init__(self, alt_m: float = 1.0, detect_rate: float = 0.0):
        with _state_lock:
            _state['z'] = alt_m
        self._rate = detect_rate

    def start(self): pass

    def get_pose(self):
        if self._rate > 0 and random.random() < self._rate:
            with _state_lock:
                z = _state['z']
            return (0.0, 0.0, z, 0.0)
        return None

    def get_depth_alt(self):
        with _state_lock:
            return _state['z']

    def get_frame(self): return None
    def get_latency(self): return (0.0, 0.0, 0.0)
    def get_depth_latency(self): return (0.0, 0.0)
