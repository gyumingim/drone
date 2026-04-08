#!/bin/bash
# run_sitl_cam.sh — x500_cam 모델로 PX4 SITL 실행 (재컴파일 없음)
# 사용: ./run_sitl_cam.sh

PX4_DIR="$HOME/PX4-Autopilot"
BUILD_DIR="$PX4_DIR/build/px4_sitl_default"
AIRFRAME_DST="$BUILD_DIR/etc/init.d-posix/airframes/4022_gz_x500_cam"
AIRFRAME_SRC="$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes/4022_gz_x500_cam"

# 바이너리 확인
if [ ! -f "$BUILD_DIR/bin/px4" ]; then
    echo "❌ PX4 바이너리 없음. 먼저 한 번 빌드하세요:"
    echo "   cd $PX4_DIR && make px4_sitl gz_x500"
    exit 1
fi

# airframe 빌드 디렉토리에 복사
cp "$AIRFRAME_SRC" "$AIRFRAME_DST" 2>/dev/null || true

echo "✅ x500_cam SITL 시작..."
echo "   카메라 토픽: /drone/downward_cam/image"
echo ""

cd "$PX4_DIR"
PX4_SIM_MODEL=gz_x500_cam \
    "$BUILD_DIR/bin/px4" \
    -s "$BUILD_DIR/etc/init.d-posix/rcS" \
    -t "$BUILD_DIR/etc"
