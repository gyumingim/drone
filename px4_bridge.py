"""
PX4 Bridge - MAVSDK Offboard Controller
Isaac Sim + Pegasus + PX4 SITL 연동
"""
import asyncio
import numpy as np
from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed, OffboardError


class PX4Bridge:
    """MAVSDK를 사용한 PX4 Offboard 제어"""

    def __init__(self, system_address="udp://:14540"):
        """
        초기화

        Args:
            system_address: PX4 SITL 연결 주소
        """
        self.drone = System()
        self.system_address = system_address
        self.connected = False

        # 현재 상태
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.yaw = 0.0

        print(f"[PX4Bridge] Initialized with {system_address}")

    async def connect(self):
        """PX4 SITL에 연결 및 Offboard 모드 시작"""
        print(f"[PX4Bridge] Connecting to {self.system_address}...")

        # PX4 연결
        await self.drone.connect(system_address=self.system_address)

        # 연결 대기
        print("[PX4Bridge] Waiting for connection...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("[PX4Bridge] Connected!")
                self.connected = True
                break

        # PX4 파라미터 설정 (preflight check 우회)
        print("[PX4Bridge] Setting PX4 parameters...")
        try:
            await self.drone.param.set_param_int("COM_ARM_WO_GPS", 1)  # GPS 없이 arming
            await self.drone.param.set_param_int("SYS_HAS_MAG", 0)     # 나침반 비활성화
            await self.drone.param.set_param_int("SYS_HAS_BARO", 0)    # 기압계 비활성화
            await self.drone.param.set_param_int("CBRK_SUPPLY_CHK", 894281)  # 전원 체크 비활성화
            await self.drone.param.set_param_int("COM_RCL_EXCEPT", 4)  # RC loss 예외
            await self.drone.param.set_param_int("CBRK_AIRSPD_CHK", 162128)  # Airspeed check 비활성화
            await self.drone.param.set_param_int("COM_ARM_EKF_AB", 0)  # Accel bias check 비활성화
            await self.drone.param.set_param_int("CBRK_GYRO_FFT", 7122069)  # Gyro FFT check 비활성화
            print("[PX4Bridge] Parameters set successfully")
        except Exception as e:
            print(f"[PX4Bridge] Warning: Could not set all parameters: {e}")

        # 잠시 대기 (파라미터 적용)
        await asyncio.sleep(1.0)

        # Arm 및 Offboard 모드 시작
        print("[PX4Bridge] Arming...")
        try:
            await self.drone.action.arm()
        except Exception as e:
            print(f"[PX4Bridge] Arming failed: {e}")
            print("[PX4Bridge] ERROR: Cannot arm - PX4 preflight checks failing")
            print("[PX4Bridge] Please manually calibrate sensors in PX4:")
            print("[PX4Bridge]   1. In PX4 console: commander calibrate accelerometer")
            print("[PX4Bridge]   2. Or disable all checks with these params:")
            print("[PX4Bridge]      param set COM_ARM_WO_GPS 1")
            print("[PX4Bridge]      param set CBRK_SUPPLY_CHK 894281")
            print("[PX4Bridge]      param set COM_ARM_EKF_AB 0")
            print("[PX4Bridge]      param set COM_ARM_MAG_STR 0")
            print("[PX4Bridge]      param save")
            raise

        print("[PX4Bridge] Starting offboard mode...")
        # 초기 setpoint 설정 (필수)
        await self.drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
        )

        try:
            await self.drone.offboard.start()
            print("[PX4Bridge] Offboard mode started!")
        except OffboardError as e:
            print(f"[PX4Bridge] Offboard start failed: {e}")
            raise

        # 상태 업데이트 백그라운드 태스크 시작
        asyncio.create_task(self._update_state_loop())

    async def send_velocity(self, vx, vy, vz, yaw_rate):
        """
        속도 명령 전송 (Body Frame)

        Args:
            vx: Forward velocity (m/s)
            vy: Right velocity (m/s)
            vz: Down velocity (m/s)
            yaw_rate: Yaw rate (deg/s)
        """
        if not self.connected:
            return

        await self.drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(
                float(vx),
                float(vy),
                float(vz),
                float(yaw_rate)
            )
        )

    async def _update_state_loop(self):
        """상태 업데이트 루프 (백그라운드)"""
        async for odom in self.drone.telemetry.odometry():
            # Position (NED frame)
            self.position = np.array([
                odom.position_body.x_m,
                odom.position_body.y_m,
                odom.position_body.z_m
            ])

            # Velocity (Body frame)
            self.velocity = np.array([
                odom.velocity_body.x_m_s,
                odom.velocity_body.y_m_s,
                odom.velocity_body.z_m_s
            ])

            # Yaw (rad)
            self.yaw = odom.q.yaw_rad

    def get_state(self):
        """
        현재 드론 상태 반환

        Returns:
            dict: {
                'position': np.array([x, y, z]),
                'velocity': np.array([vx, vy, vz]),
                'yaw': float (rad)
            }
        """
        return {
            'position': self.position.copy(),
            'velocity': self.velocity.copy(),
            'yaw': self.yaw
        }

    async def takeoff(self, altitude=2.0):
        """이륙 (간편 함수)"""
        print(f"[PX4Bridge] Taking off to {altitude}m...")
        await self.drone.action.set_takeoff_altitude(altitude)
        await self.drone.action.takeoff()

        # 이륙 완료 대기
        await asyncio.sleep(5)
        print("[PX4Bridge] Takeoff complete!")

    async def land(self):
        """착륙"""
        print("[PX4Bridge] Landing...")
        await self.drone.action.land()

    async def stop(self):
        """Offboard 모드 중지"""
        if self.connected:
            print("[PX4Bridge] Stopping offboard...")
            await self.drone.offboard.stop()
            await self.drone.action.disarm()
            print("[PX4Bridge] Stopped!")


# 동기 함수로 래핑 (Gym 환경에서 사용)
def run_async(coro):
    """비동기 함수를 동기적으로 실행"""
    loop = asyncio.get_event_loop()
    return loop.run_until_complete(coro)
