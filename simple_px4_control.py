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
import pupil_apriltags as apriltag  # pip install pupil-apriltags
import cv2

# Pegasus
from pegasus.simulator.params import ROBOTS
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.graphical_sensors.monocular_camera import MonocularCamera
from pegasus.simulator.logic.backends import Backend
from pegasus.simulator.logic.state import State



class CONFIG:
    """
    ============================================================================
    시스템 설정값 - 모든 파라미터를 한 곳에서 관리
    ============================================================================
    """
    # ===== 카메라 설정 =====
    CAMERA_FOV_DEG = 150.0                # 목표 시야각 (도)
    CAMERA_FOCAL_LENGTH = 0.8             # 초점거리 (cm) - 짧을수록 시야각 넓음
    CAMERA_HORIZONTAL_APERTURE = 6.0      # 수평 개구 (cm) - 클수록 시야각 넓음
    CAMERA_RESOLUTION = (1280, 720)       # 이미지 해상도 (픽셀)
    CAMERA_FPS = 30                       # 카메라 프레임레이트

    # ===== 관측 및 안정화 설정 =====
    OBSERVE_FRAMES = 15                   # 태그를 연속으로 관측할 프레임 수
    STABILIZE_TIME = 0.5                  # 정지 후 안정화 대기 시간 (초)
    ATTITUDE_THRESHOLD = 0.03             # 수평 판단 기준 (라디안, ~1.7도)
    
    # ===== AprilTag 설정 =====
    TAG_SIZE = 0.5                        # 태그 실제 크기 (미터)
    TAG_FAMILY = "tag36h11"               # 태그 패밀리 (36h11 = 36비트, 해밍거리 11)
    QUAD_DECIMATE = 2.0                   # 쿼드 검출 다운샘플링 (작은 태그 검출용)
    
    # ===== PID 제어기 게인 =====
    KP_XY, KP_Z = 6.0, 6.0               # 비례 게인 (위치 오차)
    KD_XY, KD_Z = 8.0, 8.0               # 미분 게인 (속도 오차)
    KI_XY, KI_Z = 1.0, 1.0               # 적분 게인 (누적 오차)
    KR, KW = 2.5, 0.35                   # 자세 제어 게인 (회전, 각속도)
    INTEGRAL_LIMIT = 2.5                  # 적분 포화 방지 (windup)
    
    # ===== 고도 설정 =====
    TAKEOFF_HEIGHT = 3                    # 이륙 고도 (미터)
    MID_ALTITUDE = 2.0                    # 1차 이동 후 재관측 고도
    LANDING_ALT = 0.5                     # 최종 착륙 고도
    TRACKING_ALT = 2.0                    # 추적 고도
    APPROACH_ALT = 1.0                    # 접근 고도
    
    # ===== 정렬 임계값 =====
    ALIGNMENT_THRESHOLD_PX = 100          # 태그 중심 정렬 허용 오차 (픽셀)
    TIGHT_ALIGNMENT_PX = 40               # 엄격한 정렬 기준 (픽셀)
    POSITION_DEADZONE_M = 0.15            # 위치 오차 데드존 (미터)
    IMAGE_DEADZONE_PX = 15                # 이미지 중심 데드존 (픽셀)
    
    # ===== 속도 제한 =====
    MAX_XY_VELOCITY = 1.0                 # 최대 수평 속도 (m/s)
    MAX_Z_VELOCITY = 0.3                  # 최대 수직 속도 (m/s)
    MAX_TARGET_CHANGE_RATE = 0.5          # 목표 위치 변화율 제한 (m/s)
    
    # ===== 예측 및 속도 매칭 =====
    PREDICT_COARSE = 0.15                 # 거친 추적 시 예측 시간
    PREDICT_FINE = 0.08                   # 정밀 정렬 시 예측 시간
    PREDICT_APPROACH = 0.06               # 접근 시 예측 시간
    PREDICT_DESCENT = 0.04                # 하강 시 예측 시간
    PREDICT_LANDED = 0.03                 # 착륙 시 예측 시간
    
    VEL_MATCH_COARSE = 0.5                # 거친 추적 시 속도 매칭 비율
    VEL_MATCH_FINE = 0.3                  # 정밀 정렬 시 속도 매칭 비율
    VEL_MATCH_APPROACH = 0.3              # 접근 시 속도 매칭 비율
    VEL_MATCH_DESCENT = 0.2               # 하강 시 속도 매칭 비율
    VEL_MATCH_LANDED = 0.1                # 착륙 시 속도 매칭 비율
    
    # ===== 상태 전환 조건 =====
    DETECT_FRAMES_SEARCH = 30             # SEARCH → COARSE_TRACK 전환 필요 프레임
    DETECT_FRAMES_COARSE = 30             # COARSE_TRACK → FINE_ALIGN 전환 필요 프레임
    ALIGN_FRAMES_FINE = 50                # FINE_ALIGN → APPROACH 전환 필요 프레임
    POSITION_ERROR_THRESHOLD = 0.5        # 위치 오차 임계값 (미터)
    
    # ===== 드론 물리 파라미터 =====
    DRONE_MASS = 1.50                     # 드론 질량 (kg)
    GRAVITY = 9.81                        # 중력 가속도 (m/s²)
    
    # ===== 로버(타겟) 설정 =====
    ROVER_HEIGHT = 0.25                   # 로버 높이 (미터)
    ROVER_TAG_OFFSET = 0.26               # 로버 상단에서 태그까지 오프셋
    ROVER_SCALE = [2.0, 1.5, 0.5]        # 로버 크기 [길이, 폭, 높이]
    ROVER_COLOR = [0.6, 0.15, 0.15]      # 로버 색상 [R, G, B]
    ROVER_MASS = 1000.0                   # 로버 질량 (kg)
    
    # ===== 환경 설정 =====
    SUN_INTENSITY = 3000                  # 태양광 강도
    SUN_ANGLE = 0.53                      # 태양광 각도 (라디안)
    SUN_COLOR = [1.0, 0.98, 0.95]        # 태양광 색상 (따뜻한 백색)

    # ===== 수동 모드 설정 =====
    MANUAL_HOVER_HEIGHT = 2.0             # 수동 모드 호버링 고도 (미터)
    MANUAL_HOVER_POSITION = [0.0, 0.0]    # 수동 모드 호버링 위치 (X, Y)
    MANUAL_MAX_VEL_XY = 0.5               # 수동 모드 최대 수평 속도 (m/s)
    MANUAL_MAX_VEL_Z = 0.3                # 수동 모드 최대 수직 속도 (m/s)

    # ===== MAVLink 설정 =====
    MAVLINK_CONNECTION = "tcp:127.0.0.1:14550"  # MAVLink 연결 문자열

    DEBUG_IMAGE_PATH = "/tmp/apriltag_debug/"  # 디버그 이미지 저장 경로

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

class SimplePX4App(Backend):
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
                "orientation": [0.0, -90.0, 0.0],
                "resolution": (1280, 720),
                "frequency": 30
            }
        )
        self.camera = camera

        # AprilTag 검출기 초기화
        focal_length_mm = 0.8
        h_aperture_mm = 6.0
        sensor_width_px = 1280
        fx = fy = (focal_length_mm / h_aperture_mm) * sensor_width_px
        self.cx = 1280 / 2
        self.cy = 720 / 2
        self.camera_params = [fx, fy, self.cx, self.cy]
        self.tag_size = 0.5  # create_apriltag_target의 size와 동일

        self.at_detector = apriltag.Detector(
            families='tag36h11',  # 또는 'tagStandard52h13', 'tagCustom48h12'
            nthreads=2,
            quad_decimate=2.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
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

        Multirotor(
            "/World/quadrotor", ROBOTS['Iris'], 0,
            [0.0, 0.0, 0.07],
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=config_multirotor
        )

        focal_length_mm = 0.8
        h_aperture_mm = 6.0
        sensor_width_px = 1280
        self.fx = self.fy = (focal_length_mm / h_aperture_mm) * sensor_width_px
        self.cx = 1280 / 2
        self.cy = 720 / 2

        # AprilTag 실제 크기 (미터)
        self.tag_size = 0.5

        print(f"[Setup] AprilTag detector ready (fx={self.fx:.1f})")

        success = force_set_camera_fov_usd()
        if success:
            print("광각 성공광각 성공광각 성공광각 성공광각 성공광각 성공광각 성공광각 성공광각 성공")
        else:
            print("광각 실패광각 실패광각 실패광각 실패광각 실패광각 실패광각 실패광각 실패광각 실패")

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

    def update_graphical_sensor(self, sensor_type, data):
        """
        카메라 이미지 업데이트 - Pegasus에서 자동 호출
        
        Args:
            sensor_type: 센서 타입 ("MonocularCamera")
            data: 센서 데이터 딕셔너리
        """
        if sensor_type == "MonocularCamera" and data is not None:
            self.frame_count += 1
            
            # ===== 50프레임마다 로그 출력 =====
            if self.frame_count % 50 == 1:
                print(f"[CAMERA] Frame #{self.frame_count}")
            
            # ===== 50프레임 이후부터 카메라 사용 (초기화 완료 대기) =====
            if self.frame_count > 50:
                self.camera_ready = True
                camera = data.get("camera")  # 카메라 객체 가져오기
                if camera:
                    try:
                        # ===== RGBA 이미지 가져오기 =====
                        rgba = camera.get_rgba()
                        if rgba is not None and rgba.size > 0:
                            self.camera_image = rgba.copy()
                            
                            # ===== 처음 3프레임은 디버그용 저장 =====
                            if self.frame_count <= 3:
                                gray = cv2.cvtColor(rgba[:,:,:3].astype(np.uint8), cv2.COLOR_RGB2GRAY)
                                cv2.imwrite(f"{CONFIG.DEBUG_IMAGE_PATH}raw_{self.frame_count:04d}.png", gray)
                                print(f"[SAVED] raw_{self.frame_count:04d}.png")
                    except Exception as e:
                        print(f"[ERROR] Camera: {e}")

    def run(self):
        self.timeline.play()
        
        # 종료
        carb.log_warn("SimplePX4App is closing.")
        self.timeline.stop()
        simulation_app.close()

    def stop(self):
        pass
    
    def start(self):
        pass

    def update_sensor(self, sensor_type, data):
        """센서 데이터 업데이트 (현재 미사용)"""
        pass

    def update_state(self, state: State):
        """
        드론 상태 업데이트 - Pegasus에서 자동 호출

        Args:
            state: 드론 현재 상태 (위치, 자세, 속도 등)
        """
        self.p = state.position           # 위치 [x, y, z]
        self.R = Rotation.from_quat(state.attitude)  # 쿼터니언 → 회전 행렬
        self.w = state.angular_velocity   # 각속도
        self.v = state.linear_velocity    # 선속도
        self.received_first_state = True  # 첫 상태 수신 완료

        # MAVLink로 상태 전송
        if hasattr(self, 'mavlink_bridge') and self.mavlink_bridge and self.mavlink_bridge.connected:
            self.mavlink_bridge.send_state(
                self.p,           # position
                self.v,           # velocity
                state.attitude,   # quaternion [x, y, z, w]
                self.w            # angular velocity
            )
    
    def input_reference(self):
        """모터 입력 레퍼런스 반환 - Pegasus에서 호출"""
        return self.input_ref
    
    def update_graphical_sensor(self, sensor_type, data):
        """
        카메라 이미지 업데이트 - Pegasus에서 자동 호출
        
        Args:
            sensor_type: 센서 타입 ("MonocularCamera")
            data: 센서 데이터 딕셔너리
        """
        if sensor_type == "MonocularCamera" and data is not None:
            self.frame_count += 1
            
            # ===== 50프레임마다 로그 출력 =====
            if self.frame_count % 50 == 1:
                print(f"[CAMERA] Frame #{self.frame_count}")
            
            # ===== 50프레임 이후부터 카메라 사용 (초기화 완료 대기) =====
            if self.frame_count > 50:
                self.camera_ready = True
                camera = data.get("camera")  # 카메라 객체 가져오기
                if camera:
                    try:
                        # ===== RGBA 이미지 가져오기 =====
                        rgba = camera.get_rgba()
                        if rgba is not None and rgba.size > 0:
                            self.camera_image = rgba.copy()
                            
                            # ===== 처음 3프레임은 디버그용 저장 =====
                            if self.frame_count <= 3:
                                gray = cv2.cvtColor(rgba[:,:,:3].astype(np.uint8), cv2.COLOR_RGB2GRAY)
                                cv2.imwrite(f"{CONFIG.DEBUG_IMAGE_PATH}raw_{self.frame_count:04d}.png", gray)
                                print(f"[SAVED] raw_{self.frame_count:04d}.png")
                    except Exception as e:
                        print(f"[ERROR] Camera: {e}")

    def _detect_apriltag(self):
        """
        AprilTag 감지 및 3D 위치 추정
        
        Returns:
            (tag_world, tag_image_center, corners, None)
            - tag_world: 태그의 월드 좌표 (3D)
            - tag_image_center: 태그 중심 이미지 좌표 (2D)
            - corners: 태그 코너 좌표
        """
        # ===== 이미지 또는 검출기가 없으면 종료 =====
        if self.camera_image is None or self.detector is None:
            return None, None, None, None
        
        try:
            img = self.camera_image
            
            # ===== RGB → Grayscale 변환 =====
            if len(img.shape) == 3 and img.shape[2] >= 3:
                gray = cv2.cvtColor(img[:, :, :3].astype(np.uint8), cv2.COLOR_RGB2GRAY)
            else:
                gray = img[:, :, 0].astype(np.uint8) if len(img.shape) == 3 else img.astype(np.uint8)
            
            # ===== CLAHE: 대비 제한 적응 히스토그램 평활화 (저조도 개선) =====
            clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
            enhanced = clahe.apply(gray)
            
            # ===== 가우시안 블러 (노이즈 제거) =====
            enhanced = cv2.GaussianBlur(enhanced, (3, 3), 0)
            
            # ===== AprilTag 검출 =====
            tags = self.detector.detect(
                enhanced,
                estimate_tag_pose=True,  # 3D 자세 추정 활성화
                camera_params=[self.fx, self.fy, self.cx, self.cy],  # 카메라 내부 파라미터
                tag_size=self.tag_size   # 태그 실제 크기 (미터)
            )
            
            # ===== 100번마다 감지 결과 로그 =====
            if self.update_count % 100 == 1:
                print(f"[DETECT] tags={len(tags)}")
            
            # ===== 태그가 감지되었으면 처리 =====
            if tags:
                tag = tags[0]  # 첫 번째 태그 사용
                
                # ===== 카메라 좌표계 → 드론 바디 좌표계 변환 =====
                # 카메라 좌표: +X=오른쪽, +Y=아래, +Z=앞
                # 드론 바디 좌표: +X=앞, +Y=왼쪽, +Z=위
                tag_body = np.array([
                    tag.pose_t[0][0],   # X (카메라 오른쪽 → 드론 오른쪽)
                    -tag.pose_t[1][0],  # Y (카메라 아래 → 드론 앞)
                    -tag.pose_t[2][0]   # Z (카메라 앞 → 드론 위)
                ])
                
                # ===== 드론 바디 좌표계 → 월드 좌표계 변환 =====
                # 회전 행렬 적용 후 드론 위치 더하기
                tag_world = self.p + self.R.as_matrix() @ tag_body
                
                # ===== 태그 위치를 멤버 변수에 저장 =====
                self.tag_position = tag_world
                self.tag_detected = True
                self.last_detection_time = time.time()  # 마지막 감지 시각 기록
                print(f"현재 태그 위치 : {self.tag_position}")
            else:
                # ===== 태그를 못 찾음 =====
                self.tag_detected = False
        
        except Exception as e:
            if self.update_count % 100 == 1:
                print(f"[ERROR] Detection: {e}")
        
        return None, None, None, None
    
    def _is_tag_centered(self, strict=False):
        """
        태그가 이미지 중심에 있는지 확인
        
        Args:
            strict: True이면 엄격한 기준 적용 (40px), False이면 느슨한 기준 (100px)
        
        Returns:
            bool: 중심에 있으면 True
        """
        if not self.tag_detected:
            return False
        
        # ===== 이미지 중심과 태그 중심 간 거리 계산 =====
        center_offset = np.linalg.norm(self.tag_image_center - np.array([self.cx, self.cy]))
        
        # ===== 데드존 이내면 무조건 True =====
        if center_offset < CONFIG.IMAGE_DEADZONE_PX:
            return True
        
        # ===== 임계값 비교 =====
        threshold = CONFIG.TIGHT_ALIGNMENT_PX if strict else CONFIG.ALIGNMENT_THRESHOLD_PX
        return center_offset < threshold
    
    def update(self, dt):
        """
        메인 제어 루프 - 매 프레임 호출

        Args:
            dt: 시간 스텝 (초)
        """
        # ===== 첫 상태를 받기 전에는 아무것도 하지 않음 =====
        if not self.received_first_state:
            return

        self.time += dt
        self.update_count += 1

        # ===== 1. AprilTag 감지 (항상 수행 - 검증용) =====
        self._detect_apriltag()

        # ===== 2. 감지 카운터 업데이트 =====
        if self.tag_detected:
            self.consecutive_detections += 1
            self.lost_frames = 0

            if self._is_tag_centered(strict=False):
                self.consecutive_aligned += 1
            else:
                self.consecutive_aligned = 0
        else:
            self.lost_frames += 1
            self.consecutive_aligned = 0

            if self.lost_frames > 30:
                self.consecutive_detections = 0

        # ===== 3. 목표 위치/속도 결정 (모드에 따라 분기) =====
        if self.mode == "manual":
            # 수동 모드: 키보드 입력으로 속도 제어
            if self.keyboard_handler:
                target_vel = self.keyboard_handler.get_velocity_command()
            else:
                target_vel = np.zeros(3)

            # 현재 위치 유지 (속도 제어 모드)
            target_pos = self.p.copy()

            # 5초마다 상태 출력
            if self.update_count % 150 == 0:
                print(f"[MANUAL] Pos: {self.p}, Vel cmd: {target_vel}")
                if self.tag_detected:
                    print(f"[MANUAL] Tag detected at: {self.tag_position}")
        else:
            # 자동 모드: 상태 머신 실행
            target_pos, target_vel = self._state_machine()

        # ===== 4. 목표 위치 급변 방지 (Slew Rate Limiter) =====
        if self.prev_target_pos is not None:
            max_change = CONFIG.MAX_TARGET_CHANGE_RATE * dt
            delta = target_pos - self.prev_target_pos
            delta_norm = np.linalg.norm(delta)

            if delta_norm > max_change:
                target_pos = self.prev_target_pos + (delta / delta_norm) * max_change

        self.prev_target_pos = target_pos.copy()

        # ===== 5. PID 제어 계산 및 모터 입력 생성 =====
        self._compute_control(target_pos, target_vel, dt)

    @staticmethod
    def vee(S):
        """
        Vee 연산자: 반대칭 행렬 → 벡터
        
        [  0  -z   y ]       [ x ]
        [  z   0  -x ]  -->  [ y ]
        [ -y   x   0 ]       [ z ]
        """
        return np.array([-S[1, 2], S[0, 2], -S[0, 1]])
    
    def reset(self):
        """리셋 (현재 미사용)"""
        pass

def force_set_camera_fov_usd(camera_prim_path='/World/quadrotor/body/downward_cam', focal_length_cm=0.8, horizontal_aperture_cm=6.0):
    """
    ============================================================================
    USD Prim 속성 직접 설정 - API 우회 (FOV 강제 설정)
    ============================================================================
    
    Isaac Sim의 카메라 API(set_focal_length, set_horizontal_aperture)가
    제대로 작동하지 않는 버그가 있어, USD Prim 속성을 직접 수정합니다.
    
    Args:
        camera_prim_path: 카메라 USD 경로 (예: "/World/quadrotor/body/downward_cam")
        focal_length_cm: 초점거리 (cm 단위)
        horizontal_aperture_cm: 수평 개구 (cm 단위)
    
    Returns:
        bool: 성공 여부
    """
    # ===== USD Stage 가져오기 =====
    stage = omni.usd.get_context().get_stage()
    camera_prim = stage.GetPrimAtPath(camera_prim_path)
    
    # ===== 카메라 Prim 존재 확인 =====
    if not camera_prim.IsValid():
        print(f"[WARN] Camera prim not found: {camera_prim_path}")
        return False
    
    # ===== USD 속성 직접 설정 =====
    # 중요: USD는 mm 단위, API는 cm 단위 (1/10 stage unit)
    focal_length_mm = focal_length_cm
    h_aperture_mm = horizontal_aperture_cm
    
    camera_prim.CreateAttribute("distortion_model", Sdf.ValueTypeNames.Token).Set("polynomial")
    camera_prim.GetAttribute("focalLength").Set(focal_length_mm)
    camera_prim.GetAttribute("horizontalAperture").Set(h_aperture_mm)
    camera_prim.GetAttribute("focusDistance").Set(1.0)  # 초점 거리 1m
    camera_prim.GetAttribute("fStop").Set(0.0)          # DoF(Depth of Field) 끄기
    
    # ===== 수직 개구 자동 계산 (Aspect Ratio 유지) =====
    aspect = 16/9
    v_aperture_mm = h_aperture_mm * aspect
    camera_prim.GetAttribute("verticalAperture").Set(v_aperture_mm)
    
    # ===== 실제 FOV 계산 및 출력 =====
    # FOV = 2 * arctan(aperture / (2 * focal_length))
    fov_rad = 2 * np.arctan(h_aperture_mm / (2 * focal_length_mm))
    fov_deg = np.degrees(fov_rad)
    
    print(f"[USD] Camera FOV forced:")
    print(f"  ├─ Focal Length: {focal_length_mm:.1f}mm ({focal_length_cm}cm)")
    print(f"  ├─ H Aperture: {h_aperture_mm:.1f}mm ({horizontal_aperture_cm}cm)")
    print(f"  ├─ V Aperture: {v_aperture_mm:.1f}mm")
    print(f"  └─ Calculated FOV: {fov_deg:.1f}°")
    
    return True


def main():
    """메인 함수"""
    app = SimplePX4App()
    app.run()


if __name__ == "__main__":
    main()
