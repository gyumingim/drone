"""
dbg_uwb_debug.py — UWB 시리얼 단독 디버그

시리얼 포트를 직접 열어 DWM1001 raw 데이터를 출력하고
POS 필드 파싱 결과를 확인하는 최소 진단 도구.

용도:
  - UWB 하드웨어 연결 확인
  - lec 포맷 출력 여부 확인
  - POS 파싱 정상 동작 확인
  - lib_uwb_reader 없이 독립 실행 가능

실행 후 20줄 수신하면 자동 종료. Ctrl+C로 중간 종료 가능.
"""
import time
import serial
from lib_uwb_reader import PORT, BAUD, _parse_pos

# ── 포트 열기 ─────────────────────────────────────────────────────────────────
print(f'[DEBUG] {PORT} 열기 시도...')
try:
    # dsrdtr=False, rtscts=False: 하드웨어 흐름 제어 비활성화 (DWM1001 필수)
    ser = serial.Serial(PORT, BAUD, timeout=2, dsrdtr=False, rtscts=False)
    print(f'[DEBUG] 포트 열기 성공')
except Exception as e:
    print(f'[DEBUG] 포트 열기 실패: {e}')
    exit(1)

# ── 수신 루프 ─────────────────────────────────────────────────────────────────
print('[DEBUG] 수신 대기 (Ctrl+C 종료)...')
count = 0      # 수신된 총 줄 수
pos_count = 0  # POS 파싱 성공 횟수
try:
    while True:
        raw = ser.readline().decode('ascii', errors='ignore').strip()
        count += 1
        # !r: 비printable 문자 포함 여부 확인 (인코딩 문제 디버그)
        print(f'[RAW #{count}] {raw!r}')

        result = _parse_pos(raw)
        if result:
            pos_count += 1
            x, y, z, qf = result
            print(f'[POS #{pos_count}] x={x:.3f} y={y:.3f} z={z:.3f} qf={qf}')

        # 20줄 수신 후 자동 종료 (빠른 포트 확인 용도)
        if count >= 20:
            break
except KeyboardInterrupt:
    pass
finally:
    ser.close()

print(f'\n[SUMMARY] 총 {count}줄 수신, POS 파싱 성공: {pos_count}줄')
