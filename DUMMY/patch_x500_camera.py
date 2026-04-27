#!/usr/bin/env python3
"""
patch_x500_camera.py — PX4 SITL x500 드론에 하향 카메라 추가

한 번만 실행하면 PX4 모델 디렉토리에 x500_cam 모델이 생성됩니다.

사용법:
  python3 patch_x500_camera.py

이후 PX4 SITL 실행:
  cd ~/PX4-Autopilot
  PX4_GZ_MODEL=x500_cam make px4_sitl gz_x500_cam

카메라 토픽: /drone/downward_cam/image  (gz-transport)
카메라 설정: 320x240, 90° FOV, 15Hz, 하향 마운트
"""

import os
import glob
import sys
import shutil

# ── 삽입할 카메라 SDF 블록 ────────────────────────────────────────────────
CAMERA_SDF = """
    <!-- ════ 하향 카메라 (UWB 앵커 감지용) ════ -->
    <link name="downward_cam_link">
      <pose relative_to="base_link">0 0 -0.05 0 1.5707963 0</pose>
      <inertial>
        <mass>0.001</mass>
        <inertia>
          <ixx>1e-7</ixx><iyy>1e-7</iyy><izz>1e-7</izz>
        </inertia>
      </inertial>
      <sensor name="downward_cam" type="camera">
        <topic>/drone/downward_cam/image</topic>
        <camera>
          <horizontal_fov>1.5707963</horizontal_fov>
          <image>
            <width>320</width>
            <height>240</height>
            <format>RGB_INT8</format>
          </image>
          <clip>
            <near>0.05</near>
            <far>50</far>
          </clip>
        </camera>
        <always_on>1</always_on>
        <update_rate>15</update_rate>
        <visualize>false</visualize>
      </sensor>
    </link>

    <joint name="downward_cam_joint" type="fixed">
      <parent>base_link</parent>
      <child>downward_cam_link</child>
    </joint>
    <!-- ════════════════════════════════════════ -->
"""

# ── PX4 모델 디렉토리 후보 ────────────────────────────────────────────────
SEARCH_PATHS = [
    os.path.expanduser("~/PX4-Autopilot/Tools/simulation/gz/models"),
    os.path.expanduser("~/px4/Tools/simulation/gz/models"),
    "/opt/PX4-Autopilot/Tools/simulation/gz/models",
    "/usr/share/px4/models",
]


def find_px4_models() -> str | None:
    for path in SEARCH_PATHS:
        if os.path.isdir(path):
            return path
    # 재귀 탐색
    for pattern in [
        os.path.expanduser("~/**/Tools/simulation/gz/models"),
        os.path.expanduser("~/*/Tools/simulation/gz/models"),
    ]:
        results = glob.glob(pattern, recursive=True)
        if results:
            return results[0]
    return None


def main():
    print("=" * 60)
    print("  PX4 x500 드론 카메라 패치 스크립트")
    print("=" * 60)

    # 1) PX4 모델 디렉토리 탐색
    px4_models = find_px4_models()
    if not px4_models:
        print("\n⚠ PX4 모델 디렉토리 자동 탐색 실패.")
        print("  직접 경로를 입력하세요:")
        print("  예) ~/PX4-Autopilot/Tools/simulation/gz/models")
        px4_models = os.path.expanduser(input("> ").strip())

    print(f"\n📂 PX4 모델 디렉토리: {px4_models}")

    # 2) x500 원본 모델 확인
    x500_dir = os.path.join(px4_models, "x500")
    x500_sdf = os.path.join(x500_dir, "model.sdf")

    if not os.path.exists(x500_sdf):
        # x500_depth 시도
        x500_dir = os.path.join(px4_models, "x500_depth")
        x500_sdf = os.path.join(x500_dir, "model.sdf")
        if not os.path.exists(x500_sdf):
            print(f"\n❌ x500 모델을 찾을 수 없습니다: {x500_sdf}")
            sys.exit(1)

    print(f"✅ x500 모델 발견: {x500_sdf}")

    # 3) x500_cam 디렉토리 생성
    cam_dir = os.path.join(px4_models, "x500_cam")
    os.makedirs(cam_dir, exist_ok=True)

    # 4) model.config 생성
    config_src = os.path.join(x500_dir, "model.config")
    config_dst = os.path.join(cam_dir, "model.config")
    if os.path.exists(config_src):
        with open(config_src) as f:
            config = f.read()
        config = config.replace(
            "<name>x500</name>", "<name>x500_cam</name>")
        config = config.replace(
            "<name>x500_depth</name>", "<name>x500_cam</name>")
        config = config.replace(
            "<description>",
            "<description>x500 with downward camera for UWB anchor detection. ")
    else:
        config = (
            '<?xml version="1.0"?>\n'
            '<model>\n'
            '  <name>x500_cam</name>\n'
            '  <version>1.0</version>\n'
            '  <sdf version="1.9">model.sdf</sdf>\n'
            '  <description>x500 + downward camera (UWB anchor detection)</description>\n'
            '</model>\n'
        )
    with open(config_dst, "w") as f:
        f.write(config)
    print(f"✅ model.config 생성: {config_dst}")

    # 5) model.sdf 읽기 + 카메라 삽입
    with open(x500_sdf) as f:
        sdf = f.read()

    if "downward_cam" in sdf:
        print("⚠ 이미 downward_cam이 포함된 SDF입니다. 덮어씁니다.")

    # </model> 직전에 카메라 블록 삽입
    if "</model>" not in sdf:
        print("❌ SDF 파싱 실패: </model> 태그를 찾을 수 없습니다.")
        sys.exit(1)

    sdf_new = sdf.replace("</model>", CAMERA_SDF + "\n</model>", 1)

    # model name 변경 (첫 번째 <model name="..."> 만)
    import re
    sdf_new = re.sub(r'<model name="[^"]*">', '<model name="x500_cam">', sdf_new, count=1)

    sdf_dst = os.path.join(cam_dir, "model.sdf")
    with open(sdf_dst, "w") as f:
        f.write(sdf_new)
    print(f"✅ model.sdf 생성: {sdf_dst}")

    # 6) 기타 파일 복사 (meshes, textures 등)
    for item in os.listdir(x500_dir):
        if item in ("model.sdf", "model.config"):
            continue
        src = os.path.join(x500_dir, item)
        dst = os.path.join(cam_dir, item)
        if os.path.isdir(src):
            if os.path.exists(dst):
                shutil.rmtree(dst)
            shutil.copytree(src, dst)
        else:
            shutil.copy2(src, dst)
    print("✅ 리소스 파일 복사 완료")

    # 7) 완료 안내
    print()
    print("=" * 60)
    print("  ✅ x500_cam 모델 생성 완료!")
    print("=" * 60)
    print()
    print("PX4 SITL 실행 방법:")
    print()
    print("  cd ~/PX4-Autopilot")
    print("  PX4_GZ_MODEL=x500_cam make px4_sitl gz_x500_cam")
    print()
    print("카메라 토픽 확인:")
    print("  gz topic -l | grep downward")
    print("  gz topic -e -t /drone/downward_cam/image")
    print()
    print("gz-transport Python 바인딩 설치 (없는 경우):")
    print("  sudo apt install python3-gz-transport13 python3-gz-msgs10")
    print()


if __name__ == "__main__":
    main()
