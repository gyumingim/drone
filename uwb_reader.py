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
MAX_SPEED_MS = 10.0  # m/s — 이 속도 초과 시 측정값 버림 (UWB 노이즈 ~8m/s, 비행 이상 ~28m/s)


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
            ax  = float(parts[idx + 2])
            ay  = float(parts[idx + 3])
            # z는 옵션 — 다음 값이 마지막이거나 AN/POS 레이블이면 z=0 생략된 것
            next4 = parts[idx + 4] if idx + 4 < len(parts) else ''
            has_z = (idx + 5 < len(parts)
                     and not next4.startswith('AN')
                     and not next4.startswith('POS'))
            if has_z:
                az   = float(parts[idx + 4])
                dist = float(parts[idx + 5])
                idx += 6
            else:
                az   = 0.0
                dist = float(parts[idx + 4])
                idx += 5
            anchors[aid] = {'pos': (ax, ay, az), 'dist_m': dist}
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
        self._last_accepted = None  # (x, y, z, t) — 마지막 수락된 3D 위치+시간
        self._last_speed = 0.0     # 마지막 계산된 속도 (m/s), 거부된 것도 포함
        self._reject_count = 0
        self._total_count = 0

    def start(self):
        threading.Thread(target=self._run, daemon=True).start()

    def get_xy(self):
        with self._lock:
            return self._pos

    def get_stats(self):
        """Returns dict: speed_ms, reject_count, total_count, accept_count."""
        with self._lock:
            return {
                'speed_ms':     self._last_speed,
                'reject_count': self._reject_count,
                'total_count':  self._total_count,
                'accept_count': self._total_count - self._reject_count,
            }

    def _run(self):
        while True:
            try:
                with serial.Serial(
                    PORT, BAUD, timeout=1,
                    dsrdtr=False, rtscts=False
                ) as ser:
                    print(f'[UWB] {PORT} 연결 (lec 이미 실행 중 가정)')

                    while True:
                        raw = ser.readline().decode(
                            'ascii', errors='ignore'
                        ).strip()

                        if not raw:
                            continue
                        anchors = _parse_lec(raw)
                        if anchors is None:
                            continue

                        pos = _trilaterate(anchors)
                        if pos is None:
                            continue

                        x, y, z = pos
                        now = time.time()

                        with self._lock:
                            self._total_count += 1

                        # 속도 필터 — 이전 수락 위치 대비 3D 속도 검사
                        if self._last_accepted is not None:
                            lx, ly, lz, lt = self._last_accepted
                            dt = max(now - lt, 1e-6)
                            dist3d = math.sqrt(
                                (x - lx)**2 + (y - ly)**2 + (z - lz)**2
                            )
                            speed = dist3d / dt
                            with self._lock:
                                self._last_speed = speed
                            if speed > MAX_SPEED_MS:
                                with self._lock:
                                    self._reject_count += 1
                                print(f'[UWB] 속도필터 reject: '
                                      f'{speed:.1f}m/s '
                                      f'({dist3d:.3f}m/{dt:.3f}s)')
                                continue

                        self._last_accepted = (x, y, z, now)
                        with self._lock:
                            if self._origin is None:
                                self._origin = (x, y)
                                print(f'[UWB] origin ({x:.2f}, {y:.2f})')
                            ox, oy = self._origin
                            self._pos = (x - ox, y - oy)

            except Exception as e:
                print(f'[UWB] 에러: {e} — 3초 후 재연결')
                time.sleep(3)
