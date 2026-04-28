"""
UWB XY reader — DWM1001 lec (CSV) format, POS 필드 사용

lec format: DIST,N,AN0,id,ax,ay,az,dist,...,POS,x,y,z,qf
POS 필드: DWM1001 펌웨어가 자체 계산한 3D 위치 + quality factor (0~100)

Origin: 10~30번째 측정값 평균 (초기 9개 버림, 이후 21개 평균)
        origin 확정 전 get_xy()/get_xyz()는 None 반환 → ARM 불가
"""
import time
import threading

import serial

PORT = '/dev/ttyUSB0'
BAUD = 115200
MIN_QUALITY  = 0   # 0=필터 없음, 필요시 50 이상
ORIGIN_SKIP  = 9   # 초기 버릴 샘플 수
ORIGIN_COUNT = 50  # 평균 낼 샘플 수 (10~30번째)


def _parse_pos(line: str):
    """lec 한 줄에서 POS 필드 추출. 반환: (x, y, z, qf) or None"""
    if not line.startswith('DIST,'):
        return None
    parts = line.split(',')
    try:
        i = parts.index('POS')
        return float(parts[i+1]), float(parts[i+2]), float(parts[i+3]), int(parts[i+4])
    except (ValueError, IndexError):
        return None


class UWBReader:
    """
    UWB 태그 시리얼 리더.
    get_xy()  → (rel_x, rel_y) or None (origin 확정 전)
    get_xyz() → (rel_x, rel_y, rel_z) or None
    get_error() → 마지막 에러 문자열 or None
    """

    def __init__(self):
        self._lock        = threading.Lock()
        self._samples     = []    # origin 계산용 버퍼 (_run 전용, lock 불필요)
        self._origin      = None  # (ox, oy, oz) — 확정 후 변경 없음
        self._pos         = None  # (rel_x, rel_y, rel_z)
        self._total_count = 0
        self._last_error  = None

    def start(self):
        threading.Thread(target=self._run, daemon=True).start()

    def get_xy(self):
        with self._lock:
            return (self._pos[0], self._pos[1]) if self._pos else None

    def get_xyz(self):
        with self._lock:
            return self._pos

    def get_error(self):
        with self._lock:
            return self._last_error

    def get_stats(self):
        with self._lock:
            return {'total_count': self._total_count}

    def _run(self):
        while True:
            try:
                with self._lock:
                    self._last_error = None
                with serial.Serial(PORT, BAUD, timeout=1, dsrdtr=False, rtscts=False) as ser:
                    print(f'[UWB] {PORT} 연결')
                    while True:
                        raw = ser.readline().decode('ascii', errors='ignore').strip()
                        if not raw:
                            continue
                        result = _parse_pos(raw)
                        if result is None:
                            continue
                        x, y, z, qf = result
                        if qf < MIN_QUALITY:
                            continue

                        with self._lock:
                            self._total_count += 1
                            count = self._total_count

                        # origin 확정 전: 샘플 수집
                        if self._origin is None:
                            if count > ORIGIN_SKIP:
                                self._samples.append((x, y, z))
                                n = len(self._samples)
                                print(f'[UWB] origin 수집 중 {n}/{ORIGIN_COUNT} '
                                      f'({x:.3f}, {y:.3f}, {z:.3f})')
                                if n == ORIGIN_COUNT:
                                    ox = sum(s[0] for s in self._samples) / n
                                    oy = sum(s[1] for s in self._samples) / n
                                    oz = sum(s[2] for s in self._samples) / n
                                    with self._lock:
                                        self._origin = (ox, oy, oz)
                                    print(f'[UWB] origin 확정 ({ox:.3f}, {oy:.3f}, {oz:.3f})')
                            else:
                                print(f'[UWB] 워밍업 {count}/{ORIGIN_SKIP}')
                            continue  # origin 확정 전엔 pos 미갱신

                        with self._lock:
                            ox, oy, oz = self._origin
                            self._pos = (x - ox, y - oy, z - oz)

            except Exception as e:
                msg = str(e)
                with self._lock:
                    self._last_error = msg
                print(f'[UWB] 에러: {msg} — 3초 후 재연결')
                time.sleep(3)
