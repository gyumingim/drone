#!/bin/bash
# SITL 전체 자동 실행 스크립트
# 사용법: ./sitl_run.sh

DRONE_DIR="$(cd "$(dirname "$0")" && pwd)"
PARM_FILE="$DRONE_DIR/sitl.parm"

# sitl.parm 임시 생성
cat > "$PARM_FILE" << 'EOF'
GPS_TYPE 0
ARMING_CHECK 0
VISO_TYPE 1
EK3_SRC1_POSXY 6
EK3_SRC1_YAW 6
EK3_SRC1_POSZ 1
RNGFND1_TYPE 0
EOF

echo "[SITL] 파라미터 파일 생성: $PARM_FILE"
echo "[SITL] SITL 시작 중... (MAVProxy 콘솔이 열리면 준비 완료)"
echo "[SITL] 스크립트는 새 터미널에서 실행: python3 sitl_flight_tag.py"
echo ""

python3 ~/ardupilot/Tools/autotest/sim_vehicle.py \
    -v ArduCopter \
    --console \
    --out=127.0.0.1:14551 \
    --add-param-file="$PARM_FILE"
