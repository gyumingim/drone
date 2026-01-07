"""
Main - Isaac Sim + Pegasus + PX4 SITL 통합 초기화
MavlinkBackend를 통해 PX4와 연동
"""
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import numpy as np
import time
from scipy.spatial.transform import Rotation

# Isaac Sim / Pegasus
from omni.isaac.core.world import World
from pegasus.simulator.params import ROBOTS
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.graphical_sensors.monocular_camera import MonocularCamera

# USD
import omni.usd
from pxr import Sdf, UsdShade, UsdGeom, Gf, UsdLux

# 우리 모듈
from apriltag_detector import AprilTagDetector


# ===== 설정 =====
class CONFIG:
    CAMERA_FOV_DEG = 150.0
    CAMERA_RESOLUTION = (1280, 720)
    CAMERA_FPS = 30
    TAG_SIZE = 0.5
    TAG_POSITION = [2.0, 0.0, 0.0]  # 고정 위치 (드론 앞 2m)


def create_apriltag_target(stage, position, size, texture_path="/home/karma/isaacsim/tag586_ariel.png"):
    """AprilTag 타겟 생성 (고정)"""
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


def setup_simulation():
    """
    Isaac Sim + Pegasus + PX4 초기화

    Returns:
        px4_bridge, apriltag_detector, world, camera
    """
    print("="*70)
    print("   Isaac Sim + Pegasus + PX4 SITL Setup")
    print("="*70)

    # Pegasus Interface
    pg = PegasusInterface()
    pg._world = World(**pg._world_settings)
    world = pg.world

    # 씬 구성
    world.scene.add_default_ground_plane()
    stage = omni.usd.get_context().get_stage()
    create_sun_light(stage)

    # AprilTag 타겟 생성 (고정)
    create_apriltag_target(stage, CONFIG.TAG_POSITION, CONFIG.TAG_SIZE)

    # 카메라 설정
    camera = MonocularCamera(
        "downward_cam",
        config={
            "position": [0.0, 0.0, -0.1],
            "orientation": [0.0, -90.0, 0.0],
            "resolution": CONFIG.CAMERA_RESOLUTION,
            "frequency": CONFIG.CAMERA_FPS
        }
    )

    # 드론 생성 (MavlinkBackend 사용)
    from pegasus.simulator.logic.backends.mavlink_backend import MavlinkBackend

    config = MultirotorConfig()
    config.graphical_sensors = [camera]

    # PX4 SITL과 연동하는 MavlinkBackend
    mavlink_backend = MavlinkBackend(
        vehicle_id=0,
        connection="udpin://0.0.0.0:14540",  # PX4 SITL에서 연결 수신
        enable_lockstep=True,  # PX4와 시뮬레이션 동기화
        num_rotors=4
    )
    config.backends = [mavlink_backend]

    drone = Multirotor(
        "/World/quadrotor",
        ROBOTS['Iris'],
        0,
        [0.0, 0.0, 0.1],  # 초기 위치
        Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
        config=config
    )

    # Pegasus에 드론 등록
    pg.vehicles[0] = drone

    # World 리셋
    world.reset()
    print("[Setup] World reset complete")

    # PX4 SITL 연결 안내
    print("\n" + "="*70)
    print("   Pegasus MavlinkBackend will connect to PX4 SITL")
    print("   Make sure PX4 SITL is running in another terminal:")
    print("   cd ~/PX4-Autopilot && make px4_sitl none_iris")
    print("="*70)
    print("\n[Setup] Waiting for PX4 SITL connection...")
    print("[Setup] (Pegasus will automatically connect when you start simulation)")

    # AprilTagDetector 초기화 (상태 모니터링용)
    detector = AprilTagDetector(
        tag_size=CONFIG.TAG_SIZE,
        camera_fov_deg=CONFIG.CAMERA_FOV_DEG,
        image_resolution=CONFIG.CAMERA_RESOLUTION
    )

    print("\n[Setup] All systems ready!")
    print("[Setup] PX4 will control the drone via MavlinkBackend")
    return mavlink_backend, detector, world, camera


def test_detection():
    """AprilTag 감지 테스트 (PX4 제어 + 감지)"""
    mavlink_backend, detector, world, camera = setup_simulation()

    print("\n[Test] Starting detection test...")
    print("[Test] PX4 will control the drone")
    print("[Test] Use QGroundControl or MAVSDK to arm and takeoff")
    print("Press Ctrl+C to stop")

    try:
        for i in range(1000):
            # 시뮬레이션 스텝
            world.step(render=True)

            # 카메라 이미지 가져오기 (5초마다 출력)
            if i % 50 == 0:
                try:
                    rgba = camera.get_rgba()
                    if rgba is not None:
                        result = detector.detect(rgba)
                        if result['detected']:
                            pos = result['position']
                            print(f"[Test] Step {i}: Tag @ [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}] m, dist={result['distance']:.2f}m")
                        else:
                            print(f"[Test] Step {i}: No tag detected")
                except Exception as e:
                    print(f"[Test] Error: {e}")

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n[Test] Stopped by user")

    simulation_app.close()


if __name__ == "__main__":
    test_detection()
