#!/bin/bash
# SITL 실행 스크립트
# 사용법: ./sitl_run.sh
#
# 터미널 1: ./sitl_run.sh          ← SITL + MAVProxy
# 터미널 2: python3 sitl_flight.py ← 비행 스크립트
# 터미널 3: python3 sitl_viz.py    ← XY 궤적 시각화 (선택)

DRONE_DIR="$(cd "$(dirname "$0")" && pwd)"
PARM_FILE="$DRONE_DIR/sitl.parm"

echo "[SITL] 비행 스크립트 : python3 sitl_flight.py"
echo "[SITL] 궤적 시각화   : python3 sitl_viz.py"
echo ""

python3 ~/ardupilot/Tools/autotest/sim_vehicle.py \
    -v ArduCopter \
    --console \
    --out=127.0.0.1:14551 \
    --out=127.0.0.1:14552 \
    --add-param-file="$PARM_FILE"
