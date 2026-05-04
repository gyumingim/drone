"""
lib_fake_sensors.py — SITL용 가짜 센서

FakeUWB:       noise_m > 0 이면 가우시안 노이즈 추가
FakeTagReader: detect_rate로 tag 감지 확률 조절, alt_m으로 depth 고정
"""
import random


class FakeUWB:
    """SITL용 가짜 UWB.

    noise_m=0.0 : origin (0.0, 0.0) 고정
    noise_m=0.3 : ±30cm 가우시안 노이즈 — 멀티패스 시뮬레이션
    """

    def __init__(self, noise_m: float = 0.0):
        self._noise = noise_m

    def start(self): pass

    def get_xy(self):
        if self._noise > 0:
            return (
                random.gauss(0.0, self._noise),
                random.gauss(0.0, self._noise),
            )
        return (0.0, 0.0)


class FakeTagReader:
    """SITL용 가짜 카메라.

    detect_rate=0.0 : 항상 Tag 미감지 → UWB fallback
    detect_rate=1.0 : 항상 Tag 감지 → tag 수렴 테스트
    alt_m           : get_depth_alt() 고정 반환값
    """

    def __init__(self, alt_m: float = 1.0, detect_rate: float = 0.0):
        self._alt = alt_m
        self._rate = detect_rate

    def start(self): pass

    def get_pose(self):
        if self._rate > 0 and random.random() < self._rate:
            return (0.0, 0.0, self._alt, 0.0)  # (north, east, down, yaw)
        return None

    def get_depth_alt(self):
        return self._alt

    def get_frame(self): return None
    def get_latency(self): return (0.0, 0.0, 0.0)
    def get_depth_latency(self): return (0.0, 0.0)
