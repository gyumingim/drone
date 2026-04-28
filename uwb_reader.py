"""
UWB XY reader — DWM1001 lec (CSV) format, POS 필드 사용

lec format: DIST,N,AN0,id,ax,ay,az,dist,...,POS,x,y,z,qf
POS 필드: DWM1001 펌웨어가 자체 계산한 3D 위치 + quality factor (0~100)
"""
import time
import threading

import serial

PORT = '/dev/ttyUSB0'
BAUD = 115200
MIN_QUALITY = 0  # 0=필터 없음, 필요시 50 이상으로 올릴 것


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
    get_xy() → (rel_x, rel_y) 미터, 첫 수신 위치 기준 상대값. 없으면 None.
    get_error() → 마지막 에러 문자열. 없으면 None.
    """

    def __init__(self):
        self._lock        = threading.Lock()
        self._origin      = None
        self._pos         = None
        self._total_count = 0
        self._last_error  = None

    def start(self):
        threading.Thread(target=self._run, daemon=True).start()

    def get_xy(self):
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
                            if self._origin is None:
                                self._origin = (x, y)
                                print(f'[UWB] origin ({x:.2f}, {y:.2f})')
                            ox, oy = self._origin
                            self._pos = (x - ox, y - oy)
            except Exception as e:
                msg = str(e)
                with self._lock:
                    self._last_error = msg
                print(f'[UWB] 에러: {msg} — 3초 후 재연결')
                time.sleep(3)
