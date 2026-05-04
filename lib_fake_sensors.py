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
import time

UDP_CTRL_PORT = 14560

_state = {'x': 0.0, 'y': 0.0, 'z': 1.0, 'target_n': 0.0, 'target_e': 0.0, 'noise': 0.0}
_state_lock = threading.Lock()


def start_udp_control(port=UDP_CTRL_PORT):
    """UDP 수신 스레드 시작. sitl_viz.py 입력값 반영.

    메시지 포맷: "sensor_n,sensor_e,alt[,target_n,target_e]"
    target 필드는 선택적 — 없으면 이전 값 유지.
    """
    def _recv():
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('', port))
        while True:
            try:
                data, _ = sock.recvfrom(128)
                parts = list(map(float, data.decode().strip().split(',')))
                with _state_lock:
                    _state['x'], _state['y'], _state['z'] = parts[0], parts[1], parts[2]
                    if len(parts) >= 5:
                        _state['target_n'], _state['target_e'] = parts[3], parts[4]
                    if len(parts) >= 6:
                        _state['noise'] = parts[5]
            except Exception:
                pass
    threading.Thread(target=_recv, daemon=True).start()


def get_target():
    """현재 go_to 목표 위치 반환. (target_n, target_e) in NED meters."""
    with _state_lock:
        return _state['target_n'], _state['target_e']


class FakeUWB:
    """SITL용 가짜 UWB — DWM1001DEV 10Hz 출력 주기 모사.

    noise_m : 초기값. sitl_viz.py 슬라이더로 실시간 변경됨
    rate_hz : 10Hz 고정 (실제 DWM1001DEV 기본 출력 주기)
    """
    _RATE_HZ = 10.0

    def __init__(self, noise_m: float = 0.0):
        self._noise = noise_m
        self._cache = (0.0, 0.0)
        self._cache_lock = threading.Lock()

    def start(self):
        threading.Thread(target=self._sample_loop, daemon=True).start()

    def _sample_loop(self):
        interval = 1.0 / self._RATE_HZ
        while True:
            with _state_lock:
                x, y = _state['x'], _state['y']
                noise = _state['noise']
            effective_noise = max(self._noise, noise)
            if effective_noise > 0:
                val = (x + random.gauss(0.0, effective_noise),
                       y + random.gauss(0.0, effective_noise))
            else:
                val = (x, y)
            with self._cache_lock:
                self._cache = val
            time.sleep(interval)

    def get_xy(self):
        with self._cache_lock:
            return self._cache


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
