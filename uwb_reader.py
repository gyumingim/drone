"""
UWB XY reader — GrowSpace/DWM1001 les format

les 명령 한 번 전송 → 연속 스트림 시작.
스트림 중에는 les 재전송 금지. 3초 데이터 없으면 재전송.
"""
import re, time, threading, serial

PORT    = '/dev/ttyUSB0'
BAUD    = 115200
QF_MIN  = 40

# les 포맷: ... est[x,y,z,qf]
_EST = re.compile(r'est\[([-\d.]+),([-\d.]+),([-\d.]+),(\d+)\]')


def _parse_xy(line: str):
    """les / DIST 라인에서 (x, y) 반환. QF 미달 또는 파싱 실패 시 None."""
    # les 포맷
    m = _EST.search(line)
    if m and int(m[4]) >= QF_MIN:
        return float(m[1]), float(m[2])
    # DIST 포맷: DIST,N,...,POS,x,y,z[,qf]
    if line.startswith('DIST,'):
        p = line.split(',')
        try:
            i = p.index('POS')
            qf = int(p[i + 4]) if len(p) > i + 4 else 100
            if qf >= QF_MIN:
                return float(p[i + 1]), float(p[i + 2])
        except (ValueError, IndexError):
            pass
    return None


class UWBReader:
    """
    UWB 태그 시리얼 리더.
    get_xy() → (rel_x, rel_y) 미터, 첫 수신 위치 기준 상대값. 없으면 None.
    """

    def __init__(self):
        self._lock   = threading.Lock()
        self._origin = None
        self._pos    = None

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
                    ser.write(b'les\r\n')   # 스트림 시작
                    last = time.time()

                    while True:
                        raw = ser.readline().decode('ascii', errors='ignore').strip()

                        if not raw:
                            # 3초 무데이터 → 스트림 끊김, les 재전송
                            if time.time() - last > 3:
                                ser.write(b'les\r\n')
                                last = time.time()
                            continue

                        if raw in ('dwm>', 'les', 'lec', 'lep'):
                            continue

                        last = time.time()
                        xy = _parse_xy(raw)
                        if xy is None:
                            continue

                        x, y = xy
                        with self._lock:
                            if self._origin is None:
                                self._origin = (x, y)
                                print(f'[UWB] origin ({x:.2f}, {y:.2f})')
                            ox, oy = self._origin
                            self._pos = (x - ox, y - oy)

            except Exception as e:
                print(f'[UWB] 에러: {e} — 3초 후 재연결')
                time.sleep(3)
