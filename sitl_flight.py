"""
sitl_flight.py — SITL용 flight.py

flight.py와 동일한 흐름. 차이점:
  - FC_PORT: udpin:0.0.0.0:14551 자동 설정
  - UWB: FakeUWB (noise_m으로 노이즈 주입)
  - TagReader: FakeTagReader (depth=TAKEOFF_M 고정)
  - sitl_viz.py 슬라이더로 UWB XY / 고도 Z 실시간 제어

사용법:
  python3 sitl_flight.py          # 무한 호버 (Ctrl+C로 착륙)
  python3 sitl_flight.py -t 30   # 30초 호버 후 자동 착륙

노이즈 시나리오:
  FakeUWB(noise_m=0.0)  → 이상적 조건 (기본 동작 확인)
  FakeUWB(noise_m=0.3)  → ±30cm 노이즈 (진동 재현)
"""
import os
os.environ.setdefault('FC_PORT', 'udpin:0.0.0.0:14551')

import argparse
import subprocess
import sys
import time
from pathlib import Path
from loguru import logger
from lib_fake_sensors import FakeUWB, FakeTagReader, start_udp_control, get_target
from lib_common import connect, do_takeoff, do_land, go_to, start_depth_sender, TAKEOFF_M

_VIZ = Path(__file__).parent / 'sitl_viz.py'


def main():
    parser = argparse.ArgumentParser(description='SITL flight script')
    parser.add_argument('-t', '--time', type=float, default=float('inf'),
                        metavar='SEC', help='hover duration in seconds (default: infinite)')
    args = parser.parse_args()
    hover_s = args.time

    viz = subprocess.Popen([sys.executable, str(_VIZ)], stderr=subprocess.DEVNULL)

    start_udp_control()  # sitl_viz.py 슬라이더 → FakeUWB/FakeTagReader 실시간 반영

    tag = FakeTagReader(alt_m=TAKEOFF_M)
    tag.start()

    uwb = FakeUWB(noise_m=0.0)  # 노이즈 실험 시 noise_m=0.3 으로 변경
    uwb.start()
    logger.info('[SITL] FakeUWB: {}', uwb.get_xy())

    c, stop, cache, lock = connect(uwb)
    if c is None:
        viz.terminate()
        return

    start_depth_sender(c, tag, stop, cache, lock)

    if not do_takeoff(c, stop, cache, lock):
        do_land(c, stop, cache)
        viz.terminate()
        return

    if hover_s == float('inf'):
        logger.info('[HOVR] hover indefinitely — Ctrl+C to land')
    else:
        logger.info('[HOVR] 호버 {:.0f}s', hover_s)

    deadline = time.time() + hover_s
    prev_target = (0.0, 0.0)
    try:
        while time.time() < deadline:
            if uwb.get_xy() is None:
                logger.warning('[SAFE] UWB 끊김 — 착륙')
                break
            tn, te = get_target()
            if (tn, te) != prev_target:
                logger.info('[NAV] 목표 변경: ({:.2f}N, {:.2f}E)', tn, te)
                prev_target = (tn, te)
            go_to(c, tn, te, -TAKEOFF_M)
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass

    do_land(c, stop, cache)
    viz.terminate()


if __name__ == '__main__':
    main()
