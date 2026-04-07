import asyncio
from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed


async def stream(drone, vx, vy, vz, duration):
    """duration 동안 setpoint 계속 전송"""
    steps = int(duration / 0.1)
    for _ in range(steps):
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(vx, vy, vz, 0.0))
        await asyncio.sleep(0.1)


async def run():
    drone = System()
    await drone.connect(system_address="serial:///dev/ttyACM0:57600")

    print("연결 대기...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("연결됨"); break

    print("로컬 포지션 대기...")
    async for health in drone.telemetry.health():
        print(f"  local_pos={health.is_local_position_ok}  accel={health.is_accelerometer_calibration_ok}  gyro={health.is_gyrometer_calibration_ok}")
        if health.is_local_position_ok:
            print("준비됨"); break

    print("ARM...")
    await drone.action.arm()
    await asyncio.sleep(1.0)

    # offboard 시작 전 setpoint 먼저 전송
    print("Offboard setpoint 준비...")
    for _ in range(10):
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))
        await asyncio.sleep(0.1)

    print("Offboard 시작...")
    from mavsdk.offboard import OffboardError
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"Offboard 시작 실패: {e}")
        await drone.action.disarm()
        return

    print("이륙 (2m)...")
    await stream(drone, 0, 0, -1.0, 2.0)

    print("호버링...")
    await stream(drone, 0, 0, 0, 2.0)

    print("하강...")
    await stream(drone, 0, 0, 1.0, 2.0)

    await drone.offboard.stop()
    await drone.action.land()
    print("완료")

asyncio.run(run())