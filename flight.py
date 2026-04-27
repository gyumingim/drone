"""flight.py — 제자리 이착륙 (1m 호버)"""
import time
from uwb_reader import UWBReader
from common import connect, do_takeoff, do_land, ts, HOVER_S


def main():
    uwb = UWBReader()
    uwb.start()
    print(f'[UWB] {ts()} origin 대기...')
    while uwb.get_xy() is None:
        time.sleep(0.2)
    print(f'[UWB] {ts()} origin 확정: {uwb.get_xy()}')

    c, stop = connect(uwb)

    if not do_takeoff(c, stop):
        do_land(c, stop)
        return

    print(f'[HOVR] {ts()} 호버 {HOVER_S}s')
    deadline = time.time() + HOVER_S
    while time.time() < deadline:
        if uwb.get_xy() is None:
            print(f'[SAFE] {ts()} UWB 끊김 — 착륙')
            break
        time.sleep(0.5)

    do_land(c, stop)


if __name__ == '__main__':
    main()
