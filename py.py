#!/usr/bin/env python
"""
================================================================================
AprilTag 기반 드론 정밀 착륙 시스템
- 이동하는 로버 위의 AprilTag를 감지하여 자동 착륙
- USD 직접 설정 방식으로 광각 카메라(150°) 구현
- 칼만 필터 없이 직접 감지 방식 사용
================================================================================
"""
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})  # GUI 모드로 시뮬레이션 실행

import argparse
import numpy as np
import omni.timeline  # Isaac Sim 타임라인 제어
import omni.usd      # USD(Universal Scene Description) 접근
import cv2           # OpenCV - 이미지 처리
import time
import threading     # MAVLink 백그라운드 스레드
import queue         # MAVLink 메시지 큐
from pxr import Sdf, UsdShade, UsdGeom, Gf, UsdLux  # USD 씬 구성
from omni.isaac.core.world import World
from omni.isaac.core.objects import DynamicCuboid
from scipy.spatial.transform import Rotation

from pegasus.simulator.params import ROBOTS  # 드론 모델 (Iris)
from pegasus.simulator.logic.state import State
from pegasus.simulator.logic.backends import Backend
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.graphical_sensors.monocular_camera import MonocularCamera
from pupil_apriltags import Detector  # AprilTag 감지 라이브러리


class CoordinateConverter:
    """
    ============================================================================
    좌표계 변환 유틸리티
    Isaac Sim (Right-handed: +X forward, +Y left, +Z up) ↔ NED (Right-handed: +X north, +Y east, +Z down)
    ============================================================================
    """

    @staticmethod
    def isaac_to_ned(position, velocity=None, attitude_quat=None):
        """
        Isaac Sim 좌표계를 NED 좌표계로 변환

        Args:
            position: np.array([x, y, z]) - Isaac Sim 위치 (미터)
            velocity: np.array([vx, vy, vz]) - Isaac Sim 속도 (m/s), optional
            attitude_quat: np.array([x, y, z, w]) - 쿼터니언, optional

        Returns:
            tuple: (ned_position, ned_velocity, ned_euler)
                - ned_position: np.array([north, east, down]) - NED 위치
                - ned_velocity: np.array([vn, ve, vd]) - NED 속도 or None
                - ned_euler: np.array([roll, pitch, yaw]) - 오일러각 (라디안) or None
        """
        # Position: Isaac (+Z up) → NED (+Z down)
        ned_pos = np.array([
            position[0],      # X: Forward → North
            position[1],      # Y: Left → East
            -position[2]      # -Z: Up → Down
        ])

        # Velocity (같은 변환 적용)
        ned_vel = None
        if velocity is not None:
            ned_vel = np.array([
                velocity[0],
                velocity[1],
                -velocity[2]
            ])

        # Attitude: 쿼터니언 → 오일러각 (NED frame)
        ned_euler = None
        if attitude_quat is not None:
            # Isaac Sim 쿼터니언 [x, y, z, w]
            isaac_rot = Rotation.from_quat(attitude_quat)

            # Z축 반전을 위한 180° X축 회전
            flip_rot = Rotation.from_euler('x', 180, degrees=True)
            ned_rot = flip_rot * isaac_rot

            # NED 오일러각 [roll, pitch, yaw]
            ned_euler = ned_rot.as_euler('xyz')

        return ned_pos, ned_vel, ned_euler

    @staticmethod
    def ned_to_isaac(ned_pos, ned_vel=None, ned_euler=None):
        """
        NED 좌표계를 Isaac Sim 좌표계로 변환

        Args:
            ned_pos: np.array([north, east, down]) - NED 위치
            ned_vel: np.array([vn, ve, vd]) - NED 속도, optional
            ned_euler: np.array([roll, pitch, yaw]) - 오일러각 (라디안), optional

        Returns:
            tuple: (isaac_position, isaac_velocity, isaac_quat)
                - isaac_position: np.array([x, y, z]) - Isaac Sim 위치
                - isaac_velocity: np.array([vx, vy, vz]) - Isaac Sim 속도 or None
                - isaac_quat: np.array([x, y, z, w]) - 쿼터니언 or None
        """
        # Position: NED (+Z down) → Isaac (+Z up)
        isaac_pos = np.array([
            ned_pos[0],      # North → X: Forward
            ned_pos[1],      # East → Y: Left
            -ned_pos[2]      # Down → -Z: Up
        ])

        # Velocity (역변환)
        isaac_vel = None
        if ned_vel is not None:
            isaac_vel = np.array([
                ned_vel[0],
                ned_vel[1],
                -ned_vel[2]
            ])

        # Attitude: 오일러각 → 쿼터니언 (Isaac frame)
        isaac_quat = None
        if ned_euler is not None:
            # NED 오일러각 → 회전 행렬
            ned_rot = Rotation.from_euler('xyz', ned_euler)

            # Z축 복원을 위한 -180° X축 회전
            flip_rot = Rotation.from_euler('x', -180, degrees=True)
            isaac_rot = flip_rot * ned_rot

            # Isaac Sim 쿼터니언 [x, y, z, w]
            isaac_quat = isaac_rot.as_quat()

        return isaac_pos, isaac_vel, isaac_quat


class KeyboardInputHandler:
    """
    ============================================================================
    키보드 입력 핸들러 - 수동 드론 제어
    WASD: 수평 이동, Space: 상승, Ctrl: 하강
    ============================================================================
    """

    def __init__(self, max_vel_xy=0.5, max_vel_z=0.3):
        """
        키보드 입력 핸들러 초기화

        Args:
            max_vel_xy: 최대 수평 속도 (m/s)
            max_vel_z: 최대 수직 속도 (m/s)
        """
        self.max_vel_xy = max_vel_xy
        self.max_vel_z = max_vel_z

        # 현재 속도 명령
        self.target_velocity = np.zeros(3)

        # 키 상태 추적
        self.key_states = {
            'W': False,      # 전진
            'S': False,      # 후진
            'A': False,      # 좌
            'D': False,      # 우
            'SPACE': False,  # 상승
            'CTRL': False    # 하강
        }

        # Isaac Sim 키보드 구독
        self._subscription = None
        self._setup_keyboard_subscription()

    def _setup_keyboard_subscription(self):
        """Isaac Sim 키보드 이벤트 구독"""
        try:
            import carb
            import omni.appwindow

            # App window 및 input interface 가져오기
            appwindow = omni.appwindow.get_default_app_window()
            input_iface = carb.input.acquire_input_interface()
            keyboard = appwindow.get_keyboard()

            # 키보드 이벤트 구독
            self._subscription = input_iface.subscribe_to_keyboard_events(
                keyboard,
                self._on_keyboard_event
            )

            print("[Keyboard] Input handler initialized")
            print("[Keyboard] Controls: WASD=Move, Space=Up, Ctrl=Down")

        except Exception as e:
            print(f"[ERROR] Keyboard setup failed: {e}")
            print("[TIP] Ensure running in GUI mode (headless=False)")

    def _on_keyboard_event(self, event, *args, **kwargs):
        """
        키보드 이벤트 콜백

        Args:
            event: carb.input 키보드 이벤트
        """
        try:
            import carb.input

            # 키 눌림/떼짐 판별
            is_pressed = (
                event.type == carb.input.KeyboardEventType.KEY_PRESS or
                event.type == carb.input.KeyboardEventType.KEY_REPEAT
            )
            is_released = (event.type == carb.input.KeyboardEventType.KEY_RELEASE)

            # 키 상태 업데이트
            if event.input == carb.input.KeyboardInput.W:
                self.key_states['W'] = is_pressed
            elif event.input == carb.input.KeyboardInput.S:
                self.key_states['S'] = is_pressed
            elif event.input == carb.input.KeyboardInput.A:
                self.key_states['A'] = is_pressed
            elif event.input == carb.input.KeyboardInput.D:
                self.key_states['D'] = is_pressed
            elif event.input == carb.input.KeyboardInput.SPACE:
                self.key_states['SPACE'] = is_pressed
            elif event.input == carb.input.KeyboardInput.LEFT_CONTROL or \
                 event.input == carb.input.KeyboardInput.RIGHT_CONTROL:
                self.key_states['CTRL'] = is_pressed

            # 속도 명령 재계산
            self._update_velocity_command()

        except Exception as e:
            print(f"[ERROR] Keyboard event: {e}")

    def _update_velocity_command(self):
        """키 상태로부터 목표 속도 계산"""
        vx = 0.0
        vy = 0.0
        vz = 0.0

        # 전진/후진 (X축)
        if self.key_states['W']:
            vx += self.max_vel_xy
        if self.key_states['S']:
            vx -= self.max_vel_xy

        # 좌/우 (Y축)
        if self.key_states['A']:
            vy += self.max_vel_xy
        if self.key_states['D']:
            vy -= self.max_vel_xy

        # 상승/하강 (Z축)
        if self.key_states['SPACE']:
            vz += self.max_vel_z
        if self.key_states['CTRL']:
            vz -= self.max_vel_z

        # 목표 속도 업데이트
        self.target_velocity = np.array([vx, vy, vz])

    def get_velocity_command(self):
        """
        현재 속도 명령 가져오기 (thread-safe)

        Returns:
            np.array: [vx, vy, vz] 목표 속도
        """
        return self.target_velocity.copy()

    def shutdown(self):
        """키보드 구독 정리"""
        if self._subscription:
            self._subscription.unsubscribe()
            print("[Keyboard] Shutdown complete")


class MAVLinkBridge:
    """
    ============================================================================
    MAVLink 통신 브리지 - PX4 SITL과 양방향 통신
    ============================================================================
    """

    def __init__(self, connection_string="tcp:127.0.0.1:14550", enable=True,
                 system_id=1, component_id=1):
        """
        MAVLink 브리지 초기화

        Args:
            connection_string: MAVLink 연결 문자열 (tcp:host:port 또는 udp:host:port)
            enable: MAVLink 통신 활성화 여부
            system_id: MAVLink 시스템 ID
            component_id: MAVLink 컴포넌트 ID
        """
        self.connection_string = connection_string
        self.enable = enable
        self.system_id = system_id
        self.component_id = component_id

        # 연결 상태
        self.connected = False
        self.mavlink_conn = None

        # 스레딩
        self.thread = None
        self.running = False
        self.lock = threading.Lock()

        # PX4로부터 수신한 데이터
        self.rx_position_target = None
        self.rx_velocity_target = None
        self.rx_timestamp = 0.0

        # 전송 메시지 큐
        self.tx_queue = queue.Queue(maxsize=100)

        # 통계
        self.tx_count = 0
        self.rx_count = 0

        if self.enable:
            self._connect()

    def _connect(self):
        """PX4 SITL에 MAVLink 연결"""
        try:
            from pymavlink import mavutil

            print(f"[MAVLink] Connecting to {self.connection_string}...")

            self.mavlink_conn = mavutil.mavlink_connection(
                self.connection_string,
                source_system=self.system_id,
                source_component=self.component_id
            )

            # PX4로부터 heartbeat 대기 (타임아웃 5초)
            print("[MAVLink] Waiting for heartbeat...")
            heartbeat = self.mavlink_conn.wait_heartbeat(timeout=5.0)

            if heartbeat:
                self.connected = True
                print(f"[MAVLink] Connected! PX4 System ID: {heartbeat.get_srcSystem()}")

                # 백그라운드 스레드 시작
                self.running = True
                self.thread = threading.Thread(
                    target=self._mavlink_loop,
                    daemon=True,
                    name="MAVLinkThread"
                )
                self.thread.start()
            else:
                print("[MAVLink] Connection timeout - no heartbeat received")
                print("[TIP] Ensure PX4 SITL is running: cd ~/PX4-Autopilot && make px4_sitl none_iris")

        except ImportError:
            print("[ERROR] pymavlink not installed. Run: pip install pymavlink")
        except Exception as e:
            print(f"[MAVLink] Connection failed: {e}")
            self.connected = False

    def _mavlink_loop(self):
        """백그라운드 스레드 - MAVLink 통신 루프"""
        last_heartbeat = time.time()
        last_stats = time.time()

        print("[MAVLink] Communication thread started")

        while self.running:
            try:
                current_time = time.time()

                # HEARTBEAT 전송 (1 Hz)
                if current_time - last_heartbeat >= 1.0:
                    self._send_heartbeat()
                    last_heartbeat = current_time

                # 송신 큐에서 메시지 가져와서 전송
                try:
                    msg = self.tx_queue.get(timeout=0.01)
                    self.mavlink_conn.mav.send(msg)
                    self.tx_count += 1
                except queue.Empty:
                    pass

                # PX4로부터 메시지 수신
                msg = self.mavlink_conn.recv_match(blocking=False, timeout=0.01)
                if msg:
                    self._handle_message(msg)
                    self.rx_count += 1

                # 10초마다 통계 출력
                if current_time - last_stats >= 10.0:
                    print(f"[MAVLink] TX: {self.tx_count}, RX: {self.rx_count}")
                    last_stats = current_time

                time.sleep(0.001)  # 1ms 루프

            except Exception as e:
                print(f"[MAVLink] Loop error: {e}")
                time.sleep(0.1)

        print("[MAVLink] Communication thread stopped")

    def _send_heartbeat(self):
        """PX4에 HEARTBEAT 메시지 전송"""
        from pymavlink import mavutil

        self.mavlink_conn.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_QUADROTOR,      # 타입
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,   # 외부 autopilot
            0,                                        # Base mode
            0,                                        # Custom mode
            mavutil.mavlink.MAV_STATE_ACTIVE         # 시스템 상태
        )

    def send_state(self, position, velocity, attitude_quat, angular_velocity):
        """
        드론 상태를 PX4에 전송 (메인 스레드에서 호출)

        Args:
            position: np.array([x, y, z]) - Isaac Sim 위치
            velocity: np.array([vx, vy, vz]) - Isaac Sim 속도
            attitude_quat: np.array([x, y, z, w]) - 쿼터니언
            angular_velocity: np.array([wx, wy, wz]) - 각속도 (rad/s)
        """
        if not self.connected:
            return

        try:
            from pymavlink import mavutil

            # NED 좌표계로 변환
            ned_pos, ned_vel, ned_euler = CoordinateConverter.isaac_to_ned(
                position, velocity, attitude_quat
            )

            # 타임스탬프
            time_boot_ms = int(time.time() * 1000) % (2**32)  # 32비트 랩어라운드

            # ATTITUDE 메시지
            att_msg = self.mavlink_conn.mav.attitude_encode(
                time_boot_ms=time_boot_ms,
                roll=float(ned_euler[0]),
                pitch=float(ned_euler[1]),
                yaw=float(ned_euler[2]),
                rollspeed=float(angular_velocity[0]),
                pitchspeed=float(angular_velocity[1]),
                yawspeed=float(angular_velocity[2])
            )
            self.tx_queue.put(att_msg)

            # LOCAL_POSITION_NED 메시지
            pos_msg = self.mavlink_conn.mav.local_position_ned_encode(
                time_boot_ms=time_boot_ms,
                x=float(ned_pos[0]),
                y=float(ned_pos[1]),
                z=float(ned_pos[2]),
                vx=float(ned_vel[0]),
                vy=float(ned_vel[1]),
                vz=float(ned_vel[2])
            )
            self.tx_queue.put(pos_msg)

        except queue.Full:
            # 큐가 가득 참 - 이번 업데이트 건너뜀
            pass
        except Exception as e:
            print(f"[MAVLink] Send error: {e}")

    def _handle_message(self, msg):
        """
        수신한 MAVLink 메시지 처리

        Args:
            msg: MAVLink 메시지 객체
        """
        msg_type = msg.get_type()

        if msg_type == "SET_POSITION_TARGET_LOCAL_NED":
            # NED 위치/속도 타겟 추출
            ned_pos = np.array([msg.x, msg.y, msg.z])
            ned_vel = np.array([msg.vx, msg.vy, msg.vz])

            # Isaac Sim 좌표계로 변환
            isaac_pos, isaac_vel, _ = CoordinateConverter.ned_to_isaac(
                ned_pos, ned_vel
            )

            # Thread-safe 저장
            with self.lock:
                self.rx_position_target = isaac_pos
                self.rx_velocity_target = isaac_vel
                self.rx_timestamp = time.time()

            print(f"[MAVLink] RX Position Target: {isaac_pos}")

        elif msg_type == "HEARTBEAT":
            # PX4 heartbeat 수신 (디버깅용, 무시)
            pass

        elif msg_type == "COMMAND_LONG":
            # 명령어 (arm, disarm 등) - 미래 확장용
            print(f"[MAVLink] RX COMMAND_LONG: {msg.command}")

    def get_position_target(self):
        """
        PX4로부터 수신한 위치 타겟 가져오기 (thread-safe)

        Returns:
            np.array or None: [x, y, z] Isaac Sim 좌표계 목표 위치
        """
        with self.lock:
            if self.rx_position_target is not None:
                return self.rx_position_target.copy()
            return None

    def get_velocity_target(self):
        """
        PX4로부터 수신한 속도 타겟 가져오기 (thread-safe)

        Returns:
            np.array or None: [vx, vy, vz] Isaac Sim 좌표계 목표 속도
        """
        with self.lock:
            if self.rx_velocity_target is not None:
                return self.rx_velocity_target.copy()
            return None

    def shutdown(self):
        """MAVLink 연결 종료"""
        print("[MAVLink] Shutting down...")
        self.running = False

        if self.thread:
            self.thread.join(timeout=2.0)

        if self.mavlink_conn:
            self.mavlink_conn.close()

        print(f"[MAVLink] Shutdown complete (TX: {self.tx_count}, RX: {self.rx_count})")


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


class LandingController(Backend):
    """
    ============================================================================
    드론 착륙 제어기 - PID 기반 위치/자세 제어 + AprilTag 감지
    ============================================================================
    """
    def __init__(self, tag_size=CONFIG.TAG_SIZE, camera_fov=CONFIG.CAMERA_FOV_DEG, mode="auto", enable_mavlink=False):
        """
        제어기 초기화

        Args:
            tag_size: AprilTag 실제 크기 (미터)
            camera_fov: 카메라 시야각 (도)
            mode: 동작 모드 ("auto" 또는 "manual")
            enable_mavlink: MAVLink 통신 활성화 여부
        """
        # ===== 모터 입력 레퍼런스 (4개 로터 속도) =====
        self.input_ref = [0.0, 0.0, 0.0, 0.0]
        
        # ===== 드론 상태 변수 =====
        self.p = np.zeros(3)              # 위치 [x, y, z] (월드 좌표계)
        self.R = Rotation.identity()      # 자세 (회전 행렬)
        self.w = np.zeros(3)              # 각속도 [roll_rate, pitch_rate, yaw_rate]
        self.v = np.zeros(3)              # 속도 [vx, vy, vz]
        
        # ===== PID 제어기 게인 행렬 =====
        self.Kp = np.diag([CONFIG.KP_XY, CONFIG.KP_XY, CONFIG.KP_Z])  # 비례 게인
        self.Kd = np.diag([CONFIG.KD_XY, CONFIG.KD_XY, CONFIG.KD_Z])  # 미분 게인
        self.Ki = np.diag([CONFIG.KI_XY, CONFIG.KI_XY, CONFIG.KI_Z])  # 적분 게인
        self.Kr = np.diag([CONFIG.KR, CONFIG.KR, CONFIG.KR])          # 자세 게인
        self.Kw = np.diag([CONFIG.KW, CONFIG.KW, CONFIG.KW])          # 각속도 게인
        
        # ===== 적분 항 (누적 오차) =====
        self.int = np.zeros(3)
        
        # ===== 물리 상수 =====
        self.m = CONFIG.DRONE_MASS        # 질량
        self.g = CONFIG.GRAVITY           # 중력
        
        # ===== 상태 머신 =====
        self.state = "TAKEOFF"            # 초기 상태: 이륙
        
        # ===== AprilTag 감지 변수 =====
        self.tag_size = tag_size          # 태그 크기
        self.tag_detected = False         # 태그 감지 여부
        self.tag_position = np.zeros(3)   # 태그 월드 좌표 [x, y, z]
        self.tag_velocity = np.zeros(3)   # 태그 속도 (현재 미사용)
        self.tag_image_center = np.zeros(2)  # 태그 이미지 좌표 [u, v]
        
        # ===== 감지 카운터 =====
        self.consecutive_detections = 0   # 연속 감지 프레임 수
        self.consecutive_aligned = 0      # 연속 정렬 프레임 수
        self.detection_count = 0          # 총 감지 횟수
        self.lost_frames = 0              # 태그를 못 본 프레임 수
        
        # ===== 카메라 파라미터 =====
        self.img_w, self.img_h = CONFIG.CAMERA_RESOLUTION  # 이미지 크기
        self.fov_deg = camera_fov         # 시야각
        
        # ===== 카메라 내부 파라미터 (fx, fy, cx, cy) =====
        # fx, fy: 초점거리 (픽셀 단위)
        # cx, cy: 주점 (principal point, 보통 이미지 중심)
        self.fx = self.img_w / (2 * np.tan(np.radians(self.fov_deg / 2)))
        self.fy = self.fx                 # 정사각 픽셀 가정
        self.cx = self.img_w / 2          # 이미지 중심 x
        self.cy = self.img_h / 2          # 이미지 중심 y
        
        # ===== 카메라 데이터 =====
        self.camera_image = None          # 최신 카메라 이미지
        self.camera_ready = False         # 카메라 준비 완료 여부
        self.frame_count = 0              # 수신한 프레임 수
        self.update_count = 0             # update() 호출 횟수
        
        # ===== AprilTag 검출기 초기화 =====
        try:
            self.detector = Detector(
                families=CONFIG.TAG_FAMILY,       # 태그 패밀리
                nthreads=4,                       # 멀티스레드 (4코어 사용)
                quad_decimate=CONFIG.QUAD_DECIMATE,  # 다운샘플링 (작은 태그 검출)
                quad_sigma=0.0,                   # 가우시안 블러 (0=비활성화)
                refine_edges=True,                # 엣지 정제 (정밀도 향상)
                decode_sharpening=0.25            # 디코딩 샤프닝
            )
            print(f"[OK] AprilTag: FOV={self.fov_deg}°, fx={self.fx:.1f}")
        except Exception as e:
            print(f"[ERROR] Detector: {e}")
            self.detector = None
        
        # ===== 시간 및 초기화 플래그 =====
        self.time = 0.0                   # 경과 시간
        self.received_first_state = False # 첫 상태 수신 여부
        self.prev_target_pos = None       # 이전 목표 위치 (급변 방지용)

        # ===== 동작 모드 =====
        self.mode = mode                  # "auto" 또는 "manual"
        self.enable_mavlink = enable_mavlink
        print(f"[INIT] Controller mode: {self.mode.upper()}")

        # ===== 수동 모드용 키보드 핸들러 =====
        if self.mode == "manual":
            self.keyboard_handler = KeyboardInputHandler(
                max_vel_xy=CONFIG.MANUAL_MAX_VEL_XY,
                max_vel_z=CONFIG.MANUAL_MAX_VEL_Z
            )
        else:
            self.keyboard_handler = None

        # ===== MAVLink 브리지 =====
        if enable_mavlink:
            self.mavlink_bridge = MAVLinkBridge(
                connection_string=CONFIG.MAVLINK_CONNECTION,
                enable=True,
                system_id=1,
                component_id=1
            )
        else:
            self.mavlink_bridge = None

        # ===== 디버그 이미지 저장 디렉토리 생성 =====
        import os
        os.makedirs(CONFIG.DEBUG_IMAGE_PATH, exist_ok=True)
    
    def start(self):
        """시뮬레이션 시작 시 호출 - 모든 변수 초기화"""
        self.int = np.zeros(3)
        self.received_first_state = False
        self.state = "TAKEOFF"
        self.time = 0.0
        self.consecutive_detections = 0
        self.consecutive_aligned = 0
        self.detection_count = 0
        self.lost_frames = 0
        self.prev_target_pos = None
        self.update_count = 0
        self.frame_count = 0
    
    def stop(self):
        """시뮬레이션 중지 시 호출"""
        # 키보드 핸들러 정리
        if hasattr(self, 'keyboard_handler') and self.keyboard_handler:
            self.keyboard_handler.shutdown()

        # MAVLink 브리지 정리
        if hasattr(self, 'mavlink_bridge') and self.mavlink_bridge:
            self.mavlink_bridge.shutdown()
    
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

            # 목표 위치를 속도 방향으로 계속 이동 (lookahead 방식)
            # 이렇게 하면 PID 컨트롤러가 목표를 따라가려고 힘을 냄
            lookahead_time = 2.0  # 2초 앞 위치를 목표로
            target_pos = self.p + target_vel * lookahead_time

            # 5초마다 상태 출력
            if self.update_count % 150 == 0:
                print(f"[MANUAL] Pos: {self.p}, Vel cmd: {target_vel}, Target: {target_pos}")
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
    
    def _state_machine(self):
        """
        상태 머신 - 현재 상태에 따라 목표 위치/속도 결정
        
        상태 흐름:
        TAKEOFF → SEARCH → MOVE_TO_TARGET → FINE_ALIGN → 
        APPROACH → FINAL_DESCENT → LANDED
        
        Returns:
            (target_pos, target_vel): 목표 위치 (3D), 목표 속도 (3D)
        """
        target_vel = np.zeros(3)  # 기본 속도는 0
        
        # ========================================================================
        # 1. TAKEOFF: 이륙
        # ========================================================================
        if self.state == "TAKEOFF":
            # 목표: 원점 위 3m
            target_pos = np.array([0.0, 0.0, CONFIG.TAKEOFF_HEIGHT])
            
            # 2.7m 이상 올라가면 SEARCH로 전환
            if self.p[2] > CONFIG.TAKEOFF_HEIGHT - 0.3:
                self.state = "SEARCH"
                self.observation_count = 0  # 관측 카운터 초기화

        # ========================================================================
        # 2. SEARCH: 태그 탐색 및 정밀 관측
        # ========================================================================
        elif self.state == "SEARCH":
            # 목표: 현재 고도 유지
            target_pos = np.array([0.0, 0.0, CONFIG.TAKEOFF_HEIGHT])
            
            # 드론 자세가 수평인지 확인 (Roll, Pitch < 1.7도)
            is_level = np.linalg.norm(self.R.as_euler('xyz')[:2]) < CONFIG.ATTITUDE_THRESHOLD
            
            # 수평 자세 + 태그 감지 시 관측 카운터 증가
            if self.tag_detected and is_level:
                self.observation_count += 1
                
                # 15프레임 연속 관측 성공 → 타겟 위치 확정
                if self.observation_count >= CONFIG.OBSERVE_FRAMES:
                    self.fixed_target_xy = self.tag_position[:2].copy()
                    self.state = "MOVE_TO_TARGET"
                    print(f"[FIXED] Target Locked: {self.fixed_target_xy}")
            else:
                # 자세가 흐트러지면 카운터 리셋
                self.observation_count = 0

        # ========================================================================
        # 3. MOVE_TO_TARGET: 확정된 위치로 이동
        # ========================================================================
        elif self.state == "MOVE_TO_TARGET":
            # 목표: 관측된 XY 위치 + 현재 고도
            target_pos = np.array([
                self.fixed_target_xy[0], 
                self.fixed_target_xy[1], 
                CONFIG.TAKEOFF_HEIGHT
            ])
            
            # 목표 지점까지의 수평 거리
            dist_to_target = np.linalg.norm(self.p[:2] - self.fixed_target_xy)
            
            # 20cm 이내 접근 → FINE_ALIGN (정밀 정렬 시작)
            if dist_to_target < 0.2:
                self.state = "FINE_ALIGN" 
                print("[MOVE] Arrived. Starting Fine Alignment")

        # ========================================================================
        # 4. FINE_ALIGN: 정밀 정렬 (2m 고도에서)
        # ========================================================================
        elif self.state == "FINE_ALIGN":
            # 태그가 보이면 실시간 위치 사용, 안 보이면 확정 위치 사용
            if self.tag_detected:
                target_xy = self.tag_position[:2]
                target_vel = np.zeros(3)
            else:
                target_xy = self.fixed_target_xy
                target_vel = np.zeros(3)

            # 목표 고도: 2m (TRACKING_ALT)
            target_pos = np.array([target_xy[0], target_xy[1], CONFIG.TRACKING_ALT])

            # 전환 조건: 태그 감지 + 거리 30cm 이내
            dist_to_target = np.linalg.norm(self.p[:2] - target_xy)
            
            if self.tag_detected and dist_to_target < 0.3:
                self.state = "APPROACH"
                print(f"[ALIGN] Target captured. Transition to APPROACH")
        
        # ========================================================================
        # 5. APPROACH: 접근 (1m 고도로 하강)
        # ========================================================================
        elif self.state == "APPROACH":
            # 목표: 태그 위 1m
            target_pos = np.array([
                self.tag_position[0], 
                self.tag_position[1], 
                CONFIG.APPROACH_ALT
            ])
            
            # 1.2m 이하로 내려가면 FINAL_DESCENT
            if self.p[2] < CONFIG.APPROACH_ALT + 0.2:
                self.state = "FINAL_DESCENT"
                print(f"\n[STATE] APPROACH → FINAL_DESCENT\n")
        
        # ========================================================================
        # 6. FINAL_DESCENT: 최종 하강
        # ========================================================================
        elif self.state == "FINAL_DESCENT":
            # 착륙 고도: 로버 높이 + 태그 오프셋
            land_z = CONFIG.ROVER_HEIGHT + CONFIG.ROVER_TAG_OFFSET
            
            # 목표: 마지막으로 감지된 태그 위치 (Hold)
            target_pos = np.array([
                self.tag_position[0], 
                self.tag_position[1], 
                land_z
            ])
            target_vel = np.zeros(3)  # 속도 매칭 X (안정성)

            # 착륙 고도 도달 → LANDED
            if self.p[2] < land_z + 0.1:
                self.state = "LANDED"
                print(f"\n[STATE] LANDED! Final Pos: {self.p[:2]}\n")
                
        # ========================================================================
        # 7. LANDED: 착륙 완료
        # ========================================================================
        elif self.state == "LANDED":
            land_z = CONFIG.ROVER_HEIGHT + CONFIG.ROVER_TAG_OFFSET
            
            # 목표: 착륙 위치 유지
            target_pos = np.array([
                self.tag_position[0], 
                self.tag_position[1], 
                land_z
            ])
            target_vel = np.zeros(3)
            
        # ========================================================================
        # 기타: 기본값 (이륙 고도)
        # ========================================================================
        else:
            target_pos = np.array([0.0, 0.0, CONFIG.TAKEOFF_HEIGHT])
        
        return target_pos, target_vel
    
    def _compute_control(self, p_ref, v_ref, dt):
        """
        PID + 기하학적 제어 (Geometric Control)
        
        Args:
            p_ref: 목표 위치 [x, y, z]
            v_ref: 목표 속도 [vx, vy, vz]
            dt: 시간 스텝
        """
        # ===== 1. 위치 오차 계산 =====
        ep = self.p - p_ref
        
        # Z축 데드존 (±5cm)
        if abs(ep[2]) < 0.05:
            ep[2] = 0.0
        
        # XY 데드존 (±15cm)
        if np.linalg.norm(ep[:2]) < CONFIG.POSITION_DEADZONE_M:
            ep[:2] = 0.0
        
        # ===== 2. 속도 오차 계산 및 제한 =====
        ev = self.v - v_ref
        
        # XY 속도 제한 (1 m/s)
        ev_xy_norm = np.linalg.norm(ev[:2])
        if ev_xy_norm > CONFIG.MAX_XY_VELOCITY:
            ev[:2] = ev[:2] / ev_xy_norm * CONFIG.MAX_XY_VELOCITY
        
        # Z 속도 제한 (0.3 m/s)
        if abs(ev[2]) > CONFIG.MAX_Z_VELOCITY:
            ev[2] = np.sign(ev[2]) * CONFIG.MAX_Z_VELOCITY
        
        # ===== 3. 적분 항 계산 (Windup 방지) =====
        self.int = np.clip(
            self.int + ep * dt, 
            -CONFIG.INTEGRAL_LIMIT, 
            CONFIG.INTEGRAL_LIMIT
        )
        
        # ===== 4. 목표 추력 벡터 계산 (PID) =====
        F_des = (
            -(self.Kp @ ep)          # 비례 항 (P)
            - (self.Kd @ ev)         # 미분 항 (D)
            - (self.Ki @ self.int)   # 적분 항 (I)
            + np.array([0, 0, self.m * self.g])  # 중력 보상
        )
        
        # ===== 5. 추력 크기 계산 (드론 Z축 방향) =====
        Z_B = self.R.as_matrix()[:, 2]  # 드론의 Z축 (위쪽)
        u_1 = F_des @ Z_B                # 추력 = F · Z
        
        # ===== 6. 목표 자세 계산 (기하학적 제어) =====
        F_norm = np.linalg.norm(F_des)
        if F_norm < 0.01:
            return  # 추력이 너무 작으면 스킵
        
        # 목표 Z축: 추력 방향
        Z_b_des = F_des / F_norm
        
        # 목표 X축: 전방 (월드 X축 방향 선호)
        X_c_des = np.array([1.0, 0.0, 0.0])
        Z_cross_X = np.cross(Z_b_des, X_c_des)
        cross_norm = np.linalg.norm(Z_cross_X)
        
        if cross_norm < 0.01:
            return  # Z축과 X축이 평행하면 스킵
        
        # 목표 Y축: Z × X
        Y_b_des = Z_cross_X / cross_norm
        
        # 목표 X축: Y × Z
        X_b_des = np.cross(Y_b_des, Z_b_des)
        
        # 목표 회전 행렬 [X, Y, Z]
        R_des = np.c_[X_b_des, Y_b_des, Z_b_des]
        
        # ===== 7. 자세 오차 계산 (SO(3) 매니폴드) =====
        R = self.R.as_matrix()  # 현재 회전 행렬
        
        # e_R = vee(R_des^T R - R^T R_des) / 2
        e_R = 0.5 * self.vee((R_des.T @ R) - (R.T @ R_des))
        
        # 각속도 오차
        e_w = self.w
        
        # ===== 8. 목표 토크 계산 =====
        tau = -(self.Kr @ e_R) - (self.Kw @ e_w)
        
        # ===== 9. 추력/토크 → 모터 속도 변환 =====
        if self.vehicle:
            self.input_ref = self.vehicle.force_and_torques_to_velocities(u_1, tau)
    
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


class MovingRover:
    """
    ============================================================================
    이동하는 로버 - AprilTag를 실은 타겟
    ============================================================================
    """
    def __init__(self, rover_prim, tag_xform, motion_type="circular", radius=1.5, speed=0.3):
        """
        Args:
            rover_prim: 로버 프림 (DynamicCuboid)
            tag_xform: 태그 Transform (UsdGeom.Xformable)
            motion_type: 움직임 타입 ("circular" 또는 "linear")
            radius: 원/직선 운동 반경 (미터)
            speed: 속도 (m/s 또는 rad/s)
        """
        self.rover = rover_prim
        self.tag_xform = tag_xform
        self.motion_type = motion_type
        self.radius = radius
        self.speed = speed
        self.time = 0.0           # 경과 시간
        self.direction = 1        # 직선 운동 방향 (1 또는 -1)
        self.linear_pos = 0.0     # 직선 운동 현재 위치
    
    def update(self, dt):
        """
        로버 위치 업데이트
        
        Args:
            dt: 시간 스텝
            
        Returns:
            tag_pos: 업데이트된 태그 위치
        """
        self.time += dt
        
        # ===== 원운동 =====
        if self.motion_type == "circular":
            x = self.radius * np.cos(self.speed * self.time)
            y = self.radius * np.sin(self.speed * self.time)
        # ===== 직선 왕복 운동 =====
        else:
            self.linear_pos += self.direction * self.speed * dt
            
            # 범위 초과 시 방향 반전
            if abs(self.linear_pos) > self.radius:
                self.direction *= -1
            
            x, y = self.linear_pos, 0.0
        
        # ===== 로버 위치 =====
        rover_pos = np.array([x, y, CONFIG.ROVER_HEIGHT])
        
        # ===== 태그 위치 (로버 위 + 오프셋) =====
        tag_pos = np.array([x, y, CONFIG.ROVER_HEIGHT + CONFIG.ROVER_TAG_OFFSET])
        
        # ===== USD 씬 업데이트 =====
        self.rover.set_world_pose(position=rover_pos)
        
        if self.tag_xform:
            self.tag_xform.GetPrim().GetAttribute("xformOp:translate").Set(
                Gf.Vec3d(tag_pos[0], tag_pos[1], tag_pos[2])
            )
        
        return tag_pos


def create_sun_light(stage):
    """태양광 생성 (Distant Light)"""
    sun = UsdLux.DistantLight.Define(stage, "/World/Sun")
    sun.CreateIntensityAttr(CONFIG.SUN_INTENSITY)
    sun.CreateAngleAttr(CONFIG.SUN_ANGLE)
    sun.CreateColorAttr(Gf.Vec3f(*CONFIG.SUN_COLOR))
    
    # 회전 (-45° X축, +30° Y축)
    xform = UsdGeom.Xformable(sun.GetPrim())
    xform.AddRotateXYZOp().Set(Gf.Vec3f(-45, 30, 0))
    
    print("[OK] Sun Light")


def create_apriltag_with_texture(stage, prim_path, position, size, texture_path):
    """
    텍스처 매핑된 AprilTag 메시 생성
    
    Args:
        stage: USD Stage
        prim_path: 프림 경로
        position: 위치 [x, y, z]
        size: 태그 크기 (한 변 길이)
        texture_path: 텍스처 이미지 경로
    
    Returns:
        UsdGeom.Xformable: 생성된 메시의 Transform
    """
    # ===== 사각형 메시 생성 =====
    mesh = UsdGeom.Mesh.Define(stage, prim_path)
    half = size / 2
    
    # 버텍스 (4개 코너)
    mesh.GetPointsAttr().Set([
        Gf.Vec3f(-half, -half, 0),  # 좌하
        Gf.Vec3f(half, -half, 0),   # 우하
        Gf.Vec3f(half, half, 0),    # 우상
        Gf.Vec3f(-half, half, 0)    # 좌상
    ])
    
    # 면 정의 (하나의 사각형)
    mesh.GetFaceVertexCountsAttr().Set([4])
    mesh.GetFaceVertexIndicesAttr().Set([0, 1, 2, 3])
    
    # 노말 벡터 (위쪽 향함)
    mesh.GetNormalsAttr().Set([Gf.Vec3f(0, 0, 1)] * 4)
    mesh.SetNormalsInterpolation("vertex")
    
    # ===== UV 좌표 (텍스처 매핑) =====
    texcoords = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "st", 
        Sdf.ValueTypeNames.TexCoord2fArray, 
        UsdGeom.Tokens.vertex
    )
    texcoords.Set([
        Gf.Vec2f(0, 0),  # 좌하
        Gf.Vec2f(1, 0),  # 우하
        Gf.Vec2f(1, 1),  # 우상
        Gf.Vec2f(0, 1)   # 좌상
    ])
    
    # ===== 위치 설정 =====
    xform = UsdGeom.Xformable(mesh)
    xform.AddTranslateOp().Set(Gf.Vec3d(*position))
    
    # ===== 머티리얼 생성 =====
    mtl_path = Sdf.Path(prim_path + "_Material")
    mtl = UsdShade.Material.Define(stage, mtl_path)
    
    # USD Preview Surface 셰이더
    shader = UsdShade.Shader.Define(stage, mtl_path.AppendPath("Shader"))
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(1.0)  # 거칠기
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)   # 비금속
    
    # UV 리더
    st_reader = UsdShade.Shader.Define(stage, mtl_path.AppendPath("stReader"))
    st_reader.CreateIdAttr("UsdPrimvarReader_float2")
    st_reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    st_reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)
    
    # 텍스처 샘플러
    diffuse_tex = UsdShade.Shader.Define(stage, mtl_path.AppendPath("DiffuseTexture"))
    diffuse_tex.CreateIdAttr("UsdUVTexture")
    diffuse_tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(texture_path)
    diffuse_tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
        st_reader.ConnectableAPI(), "result"
    )
    diffuse_tex.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("clamp")  # 클램프
    diffuse_tex.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("clamp")
    diffuse_tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    
    # 셰이더에 텍스처 연결
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
        diffuse_tex.ConnectableAPI(), "rgb"
    )
    
    # 머티리얼 출력
    mtl.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    
    # 메시에 머티리얼 바인딩
    UsdShade.MaterialBindingAPI(mesh.GetPrim()).Bind(mtl)
    
    print(f"[OK] AprilTag @ {position}")
    return UsdGeom.Xformable(mesh.GetPrim())


def force_set_camera_fov_usd(camera_prim_path, focal_length_cm, horizontal_aperture_cm):
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
    focal_length_mm = focal_length_cm * 10.0
    h_aperture_mm = horizontal_aperture_cm * 10.0
    
    camera_prim.GetAttribute("focalLength").Set(focal_length_mm)
    camera_prim.GetAttribute("horizontalAperture").Set(h_aperture_mm)
    camera_prim.GetAttribute("focusDistance").Set(1.0)  # 초점 거리 1m
    camera_prim.GetAttribute("fStop").Set(0.0)          # DoF(Depth of Field) 끄기
    
    # ===== 수직 개구 자동 계산 (Aspect Ratio 유지) =====
    aspect = CONFIG.CAMERA_RESOLUTION[1] / CONFIG.CAMERA_RESOLUTION[0]
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


class DroneLandingApp:
    def __init__(self, args):
        print("="*70)
        print(f"   AprilTag Landing - {args.mode.upper()} MODE")
        print("="*70)
        if args.mode == "manual":
            print("[MANUAL MODE]")
            print(f"  Hover Position: X={args.manual_x}m, Y={args.manual_y}m, Z={args.manual_z}m")
            print("  AprilTag detection will be monitored for verification")
        else:
            print("[AUTO MODE]")
            print("  Autonomous landing sequence enabled")
        print("="*70)

        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world
        
        self.world.scene.add_default_ground_plane()
        stage = omni.usd.get_context().get_stage()
        create_sun_light(stage)
        
        self.rover_prim = self.world.scene.add(DynamicCuboid(
            prim_path="/World/rover", name="rover",
            position=np.array([args.radius, 0.0, CONFIG.ROVER_HEIGHT]),
            scale=np.array(CONFIG.ROVER_SCALE),
            color=np.array(CONFIG.ROVER_COLOR),
            mass=CONFIG.ROVER_MASS
        ))
        
        tag_pos = np.array([args.radius, 0.0, CONFIG.ROVER_HEIGHT + CONFIG.ROVER_TAG_OFFSET])
        self.tag_xform = create_apriltag_with_texture(stage, "/World/AprilTag", tag_pos, CONFIG.TAG_SIZE, args.tag_path)

        self.rover_mover = MovingRover(self.rover_prim, self.tag_xform, args.motion, args.radius, args.rover_speed)

        # ===== 수동 모드일 경우 CONFIG 업데이트 =====
        if args.mode == "manual":
            CONFIG.MANUAL_HOVER_POSITION = [args.manual_x, args.manual_y]
            CONFIG.MANUAL_HOVER_HEIGHT = args.manual_z
            CONFIG.MANUAL_MAX_VEL_XY = args.manual_vel_xy
            CONFIG.MANUAL_MAX_VEL_Z = args.manual_vel_z

        # ===== MAVLink 설정 업데이트 =====
        if args.mavlink:
            CONFIG.MAVLINK_CONNECTION = f"tcp:127.0.0.1:{args.mavlink_port}"

        self.controller = LandingController(
            CONFIG.TAG_SIZE,
            args.camera_fov,
            mode=args.mode,
            enable_mavlink=args.mavlink
        )
        
        self.camera = MonocularCamera(
            "downward_cam",
            config={
                "position": [0.0, 0.0, -0.1],
                "orientation": [0.0, -90.0, 0.0],
                "resolution": CONFIG.CAMERA_RESOLUTION,
                "frequency": CONFIG.CAMERA_FPS
            }
        )
        
        config = MultirotorConfig()
        config.backends = [self.controller]
        config.graphical_sensors = [self.camera]
        
        Multirotor(
            "/World/quadrotor", ROBOTS['Iris'], 0,
            [0.0, 0.0, 0.07],
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=config
        )
        
        # ===== 외부 뷰 설정 (백업용) =====
        # self.pg.set_viewport_camera([6.0, 6.0, 5.0], [0.0, 0.0, 1.0])
        
        self.world.reset()
        
        print("\n[WAITING] Forcing USD camera properties in 2 seconds...")
        time.sleep(2.0)
        
        camera_path = "/World/quadrotor/body/downward_cam"
        success = force_set_camera_fov_usd(
            camera_path,
            CONFIG.CAMERA_FOCAL_LENGTH,
            CONFIG.CAMERA_HORIZONTAL_APERTURE
        )
        
        if success:
            print(f"\n[OK] USD FOV set!")
        else:
            print(f"\n[ERROR] USD set failed!")
        
        # ===== 드론 카메라로 전환 =====
        self._switch_to_drone_camera(camera_path)

        print("\n[OK] Ready! Press Play! 🚁\n")

    def _switch_to_drone_camera(self, camera_path):
        """뷰포트를 드론 카메라로 전환"""
        try:
            import omni.kit.viewport.utility as vp_util
            viewport_window = vp_util.get_active_viewport()
            
            if viewport_window:
                viewport_window.viewport_api.set_active_camera(camera_path)
                print(f"[VIEWPORT] Switched to: {camera_path}")
            else:
                print("[WARN] Viewport not found - switch manually")
                
        except Exception as e:
            print(f"[ERROR] Viewport: {e}")
            print("[TIP] Click: Window → Viewport → downward_cam")
    
    def run(self):
        while simulation_app.is_running():
            self.world.step(render=True)
            timeline = omni.timeline.get_timeline_interface()
            if timeline.is_playing():
                self.rover_mover.update(self.world.get_physics_dt())
        simulation_app.close()



    def _switch_to_drone_camera(self, camera_path):
        """
        뷰포트를 드론의 downward_cam으로 전환
        
        Args:
            camera_path: 카메라 USD 경로
        """
        try:
            import omni.kit.viewport.utility as vp_util
            
            # 메인 뷰포트 가져오기
            viewport_window = vp_util.get_active_viewport()
            
            if viewport_window:
                # 카메라 경로로 전환
                viewport_window.viewport_api.set_active_camera(camera_path)
                print(f"[VIEWPORT] Switched to: {camera_path}")
            else:
                print("[WARN] Viewport not found")
                
        except Exception as e:
            print(f"[ERROR] Viewport switch failed: {e}")
            print("[TIP] Manually click: Window → Viewport → Camera → downward_cam")

def parse_args():
    """커맨드라인 인자 파싱"""
    parser = argparse.ArgumentParser(description="AprilTag 기반 드론 착륙 시스템")

    # ===== 동작 모드 =====
    parser.add_argument("--mode", type=str, default="auto",
                       choices=["auto", "manual"],
                       help="동작 모드 (auto: 자동 착륙, manual: 수동 호버링)")

    # ===== 수동 모드 설정 =====
    parser.add_argument("--manual-x", type=float, default=0.0,
                       help="수동 모드 호버링 X 위치 (m)")
    parser.add_argument("--manual-y", type=float, default=0.0,
                       help="수동 모드 호버링 Y 위치 (m)")
    parser.add_argument("--manual-z", type=float, default=2.0,
                       help="수동 모드 호버링 고도 (m)")

    # ===== 로버 설정 =====
    parser.add_argument("--motion", type=str, default="circular",
                       choices=["circular", "linear"],
                       help="로버 움직임 타입")
    parser.add_argument("--rover-speed", type=float, default=1.2,
                       help="로버 속도 (m/s)")
    parser.add_argument("--radius", type=float, default=1.5,
                       help="로버 운동 반경 (m)")

    # ===== 카메라 설정 =====
    parser.add_argument("--camera-fov", type=float, default=150.0,
                       help="카메라 시야각 (도)")
    parser.add_argument("--tag-path", type=str,
                       default="/home/karma/isaacsim/tag586_ariel.png",
                       help="AprilTag 텍스처 경로")

    # ===== MAVLink 설정 =====
    parser.add_argument("--mavlink", action="store_true",
                       help="PX4 SITL과 MAVLink 통신 활성화")
    parser.add_argument("--mavlink-port", type=int, default=14550,
                       help="MAVLink 포트 (기본값: 14550)")

    # ===== 수동 모드 속도 설정 =====
    parser.add_argument("--manual-vel-xy", type=float, default=0.5,
                       help="수동 모드 최대 수평 속도 (m/s)")
    parser.add_argument("--manual-vel-z", type=float, default=0.3,
                       help="수동 모드 최대 수직 속도 (m/s)")

    return parser.parse_args()


if __name__ == "__main__":
    DroneLandingApp(parse_args()).run()