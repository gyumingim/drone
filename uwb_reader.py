"""
UWB XY reader — DWM1001 lec (CSV) format + manual trilateration

lec format: DIST,N,AN0,id,ax,ay,az,dist,...,POS,x,y,z,qf
POS(firmware) 무시 — 삼변측량으로 직접 계산.
"""
import math
import time
import threading

import serial
import numpy as np
from scipy.optimize import least_squares

PORT = '/dev/ttyUSB0'
BAUD = 115200
MIN_ANCHORS = 3


def _parse_lec(line: str):
    """DIST CSV 한 줄 파싱.
    반환: {anchor_id: {'pos': (ax,ay,az), 'dist_m': float}} or None
    """
    if not line.startswith('DIST,'):
        return None
    parts = line.split(',')
    try:
        n = int(parts[1])
        anchors = {}
        idx = 2
        for _ in range(n):
            aid = parts[idx + 1]
            ax = float(parts[idx + 2])
            ay = float(parts[idx + 3])
            az = float(parts[idx + 4])
            dist = float(parts[idx + 5])
            anchors[aid] = {'pos': (ax, ay, az), 'dist_m': dist}
            idx += 6
        return anchors if len(anchors) >= MIN_ANCHORS else None
    except (ValueError, IndexError):
        return None


def _trilaterate(anchors: dict):
    """3D 삼변측량 (z≥0 bound, 드론은 항상 바닥 위).
    반환: (x, y, z) or None
    """
    pts = np.array([v['pos'] for v in anchors.values()])
    dst = np.array([v['dist_m'] for v in anchors.values()])

    x0 = float(np.mean(pts[:, 0]))
    y0 = float(np.mean(pts[:, 1]))
    z0 = float(np.max(pts[:, 2])) + 1.0

    def residuals(p):
        return np.linalg.norm(pts - p, axis=1) - dst

    r = least_squares(
        residuals, [x0, y0, z0], method='trf',
        bounds=([-np.inf, -np.inf, 0.0], [np.inf, np.inf, np.inf])
    )
    x, y, z = r.x
    if not all(math.isfinite(v) for v in (x, y, z)):
        return None
    return float(x), float(y), float(z)


class UWBReader:
    """
    UWB 태그 시리얼 리더.
    get_xy() → (rel_x, rel_y) 미터, 첫 수신 위치 기준 상대값. 없으면 None.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._origin = None
        self._pos = None

    def start(self):
        threading.Thread(target=self._run, daemon=True).start()

    def get_xy(self):
        with self._lock:
            return self._pos

    def _run(self):
        while True:
            try:
                with serial.Serial(PORT, BAUD, timeout=1) as ser:
                    print(f'[UWB] {PORT} 연결')
                    # 현재 스트리밍 모드(lep/les) 탈출 후 lec 시작
                    ser.write(b'\r\n')
                    time.sleep(0.3)
                    ser.reset_input_buffer()
                    ser.write(b'lec\r\n')
                    last = time.time()

                    while True:
                        raw = ser.readline().decode(
                            'ascii', errors='ignore'
                        ).strip()

                        if not raw or raw in ('dwm>', 'lec', 'les', 'lep'):
                            if time.time() - last > 3:
                                ser.write(b'lec\r\n')
                                last = time.time()
                            continue

                        last = time.time()
                        anchors = _parse_lec(raw)
                        if anchors is None:
                            continue

                        pos = _trilaterate(anchors)
                        if pos is None:
                            continue

                        x, y, _ = pos
                        with self._lock:
                            if self._origin is None:
                                self._origin = (x, y)
                                print(f'[UWB] origin ({x:.2f}, {y:.2f})')
                            ox, oy = self._origin
                            self._pos = (x - ox, y - oy)

            except Exception as e:
                print(f'[UWB] 에러: {e} — 3초 후 재연결')
                time.sleep(3)
