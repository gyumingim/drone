"""
lib_uwb_reader.py — DWM1001 UWB 태그 시리얼 리더

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
DWM1001 lec 포맷
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  출력 예시:
    DIST,3,AN0,DW1234,1.23,4.56,0.00,2.10,...,POS,1.20,3.40,0.05,95

  POS 필드: DWM1001 펌웨어 내장 삼변측량 결과
    x, y, z: 앵커 좌표계 기준 태그 위치 (m)
    qf: quality factor (0~100, 높을수록 신뢰도 높음)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Origin 보정
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  초기 ORIGIN_SKIP개 샘플을 버리고 (워밍업)
  이후 ORIGIN_COUNT개 샘플의 평균을 origin으로 확정.
  이후 get_xy()/get_xyz()는 origin 기준 상대좌표를 반환.

  origin 확정 전 get_xy()는 None → 비행 코드는 이를 체크 후 대기
"""
import time
import threading

import serial

PORT         = '/dev/ttyUSB0'  # UWB 태그 시리얼 포트
BAUD         = 115200
MIN_QUALITY  = 0    # 최소 quality factor (0=필터 없음, 필요시 50 이상으로 설정)
ORIGIN_SKIP  = 9    # 초기 버릴 샘플 수 (DWM1001 부팅 직후 불안정한 값 제거)
ORIGIN_COUNT = 50   # origin 계산에 사용할 샘플 수 (평균)


def _parse_pos(line: str):
    """lec 포맷 한 줄에서 POS 필드를 파싱.

    반환: (x, y, z, qf) or None (POS 필드 없거나 파싱 실패 시)
    'POS' 토큰을 직접 검색하므로 앞선 DIST 필드 수에 무관하게 동작.
    """
    if not line.startswith('DIST,'):
        return None
    parts = line.split(',')
    try:
        i = parts.index('POS')   # 'POS' 토큰 위치 탐색
        return float(parts[i+1]), float(parts[i+2]), float(parts[i+3]), int(parts[i+4])
    except (ValueError, IndexError):
        return None


class UWBReader:
    """UWB 태그 시리얼 백그라운드 리더.

    start() 호출 후 백그라운드 스레드에서 시리얼을 읽고 위치를 갱신.
    에러 발생 시 3초 대기 후 자동 재연결.

    공개 API:
      get_xy()    → (rel_x, rel_y) or None  (origin 확정 전 None)
      get_xyz()   → (rel_x, rel_y, rel_z) or None
      get_error() → 마지막 에러 문자열 or None
      get_stats() → {'total_count': int}
    """

    def __init__(self):
        self._lock        = threading.Lock()
        self._samples     = []    # origin 계산용 버퍼 (_run 전용, lock 불필요)
        self._origin      = None  # (ox, oy, oz) — 확정 후 변경되지 않음
        self._pos         = None  # (rel_x, rel_y, rel_z) — origin 기준 상대좌표
        self._total_count = 0     # 수신된 총 샘플 수 (진단용)
        self._last_error  = None  # 마지막 예외 메시지

    def start(self):
        """백그라운드 시리얼 읽기 스레드 시작."""
        threading.Thread(target=self._run, daemon=True).start()

    def get_xy(self):
        """origin 기준 상대 XY 좌표. origin 미확정 시 None."""
        with self._lock:
            return (self._pos[0], self._pos[1]) if self._pos else None

    def get_xyz(self):
        """origin 기준 상대 XYZ 좌표. origin 미확정 시 None."""
        with self._lock:
            return self._pos

    def get_error(self):
        """마지막 에러 메시지. 정상 시 None."""
        with self._lock:
            return self._last_error

    def get_stats(self):
        """진단용 통계 딕셔너리."""
        with self._lock:
            return {'total_count': self._total_count}

    def _run(self):
        """시리얼 읽기 루프. 예외 발생 시 3초 후 재연결."""
        while True:
            try:
                with self._lock:
                    self._last_error = None

                # dsrdtr=False, rtscts=False: 하드웨어 흐름 제어 비활성화
                # DWM1001은 흐름 제어 없이 단방향 전송 → 활성화 시 수신 차단 가능
                with serial.Serial(PORT, BAUD, timeout=1, dsrdtr=False, rtscts=False) as ser:
                    print(f'[UWB] {PORT} 연결')
                    while True:
                        raw = ser.readline().decode('ascii', errors='ignore').strip()
                        if not raw:
                            continue

                        result = _parse_pos(raw)
                        if result is None:
                            continue   # POS 필드 없는 줄 (DIST만 있는 줄 등) 스킵

                        x, y, z, qf = result

                        # quality factor 필터 (MIN_QUALITY=0이면 통과)
                        if qf < MIN_QUALITY:
                            continue

                        with self._lock:
                            self._total_count += 1
                            count = self._total_count

                        if self._origin is None:
                            # ── origin 확정 전: 샘플 수집 단계 ────────────────
                            if count <= ORIGIN_SKIP:
                                # 초기 불안정 샘플 버림 (DWM1001 부팅 직후 발산 방지)
                                print(f'[UWB] 워밍업 {count}/{ORIGIN_SKIP}')
                            else:
                                self._samples.append((x, y, z))
                                n = len(self._samples)
                                print(f'[UWB] origin 수집 중 {n}/{ORIGIN_COUNT} '
                                      f'({x:.3f}, {y:.3f}, {z:.3f})')

                                if n == ORIGIN_COUNT:
                                    # ORIGIN_COUNT개 평균 → origin 확정
                                    ox = sum(s[0] for s in self._samples) / n
                                    oy = sum(s[1] for s in self._samples) / n
                                    oz = sum(s[2] for s in self._samples) / n
                                    with self._lock:
                                        self._origin = (ox, oy, oz)
                                    print(f'[UWB] origin 확정 ({ox:.3f}, {oy:.3f}, {oz:.3f})')

                            continue  # origin 확정 전엔 _pos 갱신 안 함

                        # ── origin 확정 후: 상대좌표 계산 ──────────────────────
                        # 절대좌표에서 origin을 빼서 이륙 지점 기준 상대좌표로 변환
                        with self._lock:
                            ox, oy, oz = self._origin
                            self._pos = (x - ox, y - oy, z - oz)

            except Exception as e:
                msg = str(e)
                with self._lock:
                    self._last_error = msg
                print(f'[UWB] 에러: {msg} — 3초 후 재연결')
                time.sleep(3)
