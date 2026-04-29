"""uwb_debug.py — UWB 시리얼 단독 디버그 (대시보드 없음)"""
import time
import serial
from lib_uwb_reader import PORT, BAUD, _parse_pos

print(f'[DEBUG] {PORT} 열기 시도...')
try:
    ser = serial.Serial(PORT, BAUD, timeout=2, dsrdtr=False, rtscts=False)
    print(f'[DEBUG] 포트 열기 성공')
except Exception as e:
    print(f'[DEBUG] 포트 열기 실패: {e}')
    exit(1)

print('[DEBUG] 수신 대기 (Ctrl+C 종료)...')
count = 0
pos_count = 0
try:
    while True:
        raw = ser.readline().decode('ascii', errors='ignore').strip()
        count += 1
        print(f'[RAW #{count}] {raw!r}')
        result = _parse_pos(raw)
        if result:
            pos_count += 1
            x, y, z, qf = result
            print(f'[POS #{pos_count}] x={x:.3f} y={y:.3f} z={z:.3f} qf={qf}')
        if count >= 20:
            break
except KeyboardInterrupt:
    pass
finally:
    ser.close()

print(f'\n[SUMMARY] 총 {count}줄 수신, POS 파싱 성공: {pos_count}줄')
