"""
sitl_flight.py — SITL용 flight.py

flight.py와 동일한 흐름. 차이점:
  - FC_PORT: udpin:0.0.0.0:14551 자동 설정
  - UWB: FakeUWB (noise_m으로 노이즈 주입)
  - TagReader: FakeTagReader (depth=TAKEOFF_M 고정)

노이즈 시나리오:
  FakeUWB(noise_m=0.0)  → 이상적 조건 (기본 동작 확인)
  FakeUWB(noise_m=0.3)  → ±30cm 노이즈 (진동 재현)
"""
import os
os.environ.setdefault('FC_PORT', 'udpin:0.0.0.0:14551')

import subprocess
import sys
import time
from pathlib import Path
from loguru import logger
from lib_fake_sensors import FakeUWB, FakeTagReader
from lib_common import connect, do_takeoff, do_land, go_to, start_depth_sender, HOVER_S, TAKEOFF_M

_VIZ = Path(__file__).parent / 'sitl_viz.py'


def main():
    viz = subprocess.Popen([sys.executable, str(_VIZ)], stderr=subprocess.DEVNULL)

    tag = FakeTagReader(alt_m=TAKEOFF_M)
    tag.start()

    uwb = FakeUWB(noise_m=0.0)  # 노이즈 실험 시 noise_m=0.3 으로 변경
    uwb.start()
    logger.info('[SITL] FakeUWB: {}', uwb.get_xy())

    c, stop, cache, lock = connect(uwb)
    if c is None:
        return

    start_depth_sender(c, tag, stop, cache, lock)

    if not do_takeoff(c, stop, cache, lock):
        do_land(c, stop, cache)
        return

    logger.info('[HOVR] 호버 {}s', HOVER_S)
    deadline = time.time() + HOVER_S
    while time.time() < deadline:
        if uwb.get_xy() is None:
            logger.warning('[SAFE] UWB 끊김 — 착륙')
            break
        go_to(c, 0, 0, -TAKEOFF_M)
        time.sleep(0.1)

    do_land(c, stop, cache)
    viz.terminate()


if __name__ == '__main__':
    main()
