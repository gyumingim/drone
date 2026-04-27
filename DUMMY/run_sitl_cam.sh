#!/bin/bash
# run_sitl_cam.sh — x500_cam 모델로 PX4 SITL 실행
# conda PATH를 제거해서 protobuf 충돌 없이 빌드/실행
#
# 사용: ./run_sitl_cam.sh

# conda PATH 제거 (protobuf 5.x 헤더 충돌 방지)
export PATH=$(echo "$PATH" | tr ":" "\n" | grep -v anaconda3 | grep -v "conda/bin" | tr "\n" ":" | sed "s/:$//")
export PYTHONPATH=""
unset CONDA_PREFIX CONDA_DEFAULT_ENV

PX4_DIR="$HOME/PX4-Autopilot"
AIRFRAME_SRC="$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes/4022_gz_x500_cam"
BUILD_DIR="$PX4_DIR/build/px4_sitl_default"
AIRFRAME_DST="$BUILD_DIR/etc/init.d-posix/airframes/4022_gz_x500_cam"

echo "Python : $(which python3)"
echo "protoc : $(protoc --version 2>/dev/null || echo 'not found')"
echo ""

# 이전 실행 잔여 프로세스 정리 (Ctrl+C 후 재실행 시 SceneBroadcaster 타임아웃 방지)
echo "🧹 이전 Gazebo/PX4 프로세스 정리..."
pkill -f gz-sim 2>/dev/null; pkill -f "bin/px4" 2>/dev/null
sleep 1

# airframe 파일 빌드 디렉토리에 동기화
if [ -f "$AIRFRAME_SRC" ]; then
    cp "$AIRFRAME_SRC" "$AIRFRAME_DST" 2>/dev/null || true
fi

echo "✅ x500_cam SITL 빌드 + 실행..."
echo "   카메라 토픽: /drone/downward_cam/image"
echo "   MAVLink: udpin://0.0.0.0:14540"
echo ""

cd "$PX4_DIR"
make px4_sitl gz_x500_cam
