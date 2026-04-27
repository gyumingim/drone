#!/usr/bin/env python
"""
Simple PX4 Control with AprilTag
Isaac Sim + Pegasus + PX4 SITL + MAVSDK 기본 예제
"""

# Isaac Sim 시작
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import carb
import omni.timeline
import omni.usd
from omni.isaac.core.world import World
from pxr import Sdf, UsdShade, UsdGeom, Gf, UsdLux
import numpy as np
from scipy.spatial.transform import Rotation

# Pegasus
from pegasus.simulator.params import ROBOTS
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.graphical_sensors.monocular_camera import MonocularCamera


def create_apriltag_target(stage, position, size=0.5, texture_path="/home/karma/isaacsim/tag586_ariel.png"):
    """AprilTag 타겟 생성"""
    mesh = UsdGeom.Mesh.Define(stage, "/World/AprilTag")
    half = size / 2

    # 사각형 메시
    mesh.GetPointsAttr().Set([
        Gf.Vec3f(-half, -half, 0),
        Gf.Vec3f(half, -half, 0),
        Gf.Vec3f(half, half, 0),
        Gf.Vec3f(-half, half, 0)
    ])
    mesh.GetFaceVertexCountsAttr().Set([4])
    mesh.GetFaceVertexIndicesAttr().Set([0, 1, 2, 3])
    mesh.GetNormalsAttr().Set([Gf.Vec3f(0, 0, 1)] * 4)
    mesh.SetNormalsInterpolation("vertex")

    # UV 좌표
    texcoords = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex
    )
    texcoords.Set([Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)])

    # 위치 설정
    xform = UsdGeom.Xformable(mesh)
    xform.AddTranslateOp().Set(Gf.Vec3d(*position))

    # 텍스처 머티리얼
    mtl_path = Sdf.Path("/World/AprilTag_Material")
    mtl = UsdShade.Material.Define(stage, mtl_path)
    shader = UsdShade.Shader.Define(stage, mtl_path.AppendPath("Shader"))
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(1.0)

    st_reader = UsdShade.Shader.Define(stage, mtl_path.AppendPath("stReader"))
    st_reader.CreateIdAttr("UsdPrimvarReader_float2")
    st_reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    st_reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)

    diffuse_tex = UsdShade.Shader.Define(stage, mtl_path.AppendPath("DiffuseTexture"))
    diffuse_tex.CreateIdAttr("UsdUVTexture")
    diffuse_tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(texture_path)
    diffuse_tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
        st_reader.ConnectableAPI(), "result"
    )
    diffuse_tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)

    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
        diffuse_tex.ConnectableAPI(), "rgb"
    )

    mtl.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI(mesh.GetPrim()).Bind(mtl)

    print(f"[Setup] AprilTag created at {position}")


def create_sun_light(stage):
    """태양광 생성"""
    sun = UsdLux.DistantLight.Define(stage, "/World/Sun")
    sun.CreateIntensityAttr(3000)
    sun.CreateAngleAttr(0.53)
    sun.CreateColorAttr(Gf.Vec3f(1.0, 0.98, 0.95))
    xform = UsdGeom.Xformable(sun.GetPrim())
    xform.AddRotateXYZOp().Set(Gf.Vec3f(-45, 30, 0))
    print("[Setup] Sun light created")


class SimplePX4App:
    """
    간단한 PX4 제어 앱
    - PX4 SITL 자동 실행
    - AprilTag 타겟
    - 카메라
    - MAVSDK/QGroundControl로 수동 조작
    """

    def __init__(self):
        """초기화"""

        # Timeline
        self.timeline = omni.timeline.get_timeline_interface()

        # Pegasus Interface
        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # 씬 구성
        self.world.scene.add_default_ground_plane()
        stage = omni.usd.get_context().get_stage()
        create_sun_light(stage)

        # AprilTag 타겟 생성 (드론 앞 2m)
        create_apriltag_target(stage, [2.0, 0.0, 0.0], size=0.5)

        # 카메라 설정
        camera = MonocularCamera(
            "downward_cam",
            config={
                "position": [0.0, 0.0, -0.1],
                "orientation": [0.0, -90.0, 0.0],  # 아래 방향
                "resolution": (1280, 720),
                "frequency": 30
            }
        )

        # 드론 생성 (PX4 자동 실행)
        config_multirotor = MultirotorConfig()
        config_multirotor.graphical_sensors = [camera]

        mavlink_config = PX4MavlinkBackendConfig({
            "vehicle_id": 0,
            "px4_autolaunch": True,  # PX4 자동 실행
            "px4_dir": self.pg.px4_path,
            "px4_vehicle_model": self.pg.px4_default_airframe
        })
        config_multirotor.backends = [PX4MavlinkBackend(mavlink_config)]

        self.vehicle = Multirotor(
            "/World/quadrotor",
            ROBOTS['Iris'],
            0,
            [0.0, 0.0, 1.0],  # 초기 위치 (1m 높이에서 시작)
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=config_multirotor,
        )

        # World 리셋
        self.world.reset()

        print("\n" + "="*70)
        print("   Simple PX4 Control Ready!")
        print("="*70)
        print("\n[Usage]")
        print("1. PX4 SITL이 자동으로 실행됩니다")
        print("2. QGroundControl을 연결하세요:")
        print("   - UDP 연결: 127.0.0.1:14550")
        print("3. 또는 MAVSDK Python 스크립트로 제어")
        print("\n[Controls in QGroundControl]")
        print("- Arm: 드론 활성화")
        print("- Takeoff: 자동 이륙")
        print("- Offboard: 수동 제어 모드")
        print("\nPress Ctrl+C to stop")
        print("="*70 + "\n")

        self.stop_sim = False

    def run(self):
        """메인 루프"""

        # 시뮬레이션 시작
        self.timeline.play()

        # 메인 루프
        while simulation_app.is_running() and not self.stop_sim:
            # 물리 스텝
            self.world.step(render=True)

        # 종료
        carb.log_warn("SimplePX4App is closing.")
        self.timeline.stop()
        simulation_app.close()


def main():
    """메인 함수"""
    app = SimplePX4App()
    app.run()


if __name__ == "__main__":
    main()