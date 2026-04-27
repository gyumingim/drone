import asyncio
from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed


SPEED = 1.0  # m/s
DIST = 1.0  # m
T = DIST / SPEED  # 1초


async def stream(drone, vx, vy, vz, duration):
    steps = int(duration / 0.1)
    for _ in range(steps):
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(vx, vy, vz, 0.0))
        await asyncio.sleep(0.1)


async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")
    async for state in drone.core.connection_state():
        if state.is_connected:
            break

    for _ in range(10):
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0,0,0,0))
        await asyncio.sleep(0.1)

    await drone.offboard.start()
    await drone.action.arm()

    print("이륙...")
    await stream(drone, 0, 0, -1.0, 2.0)
    await stream(drone, 0, 0, 0,    2.0)

    # yaw=0 기준: x=North, y=East
    directions = [
        ("North", ( SPEED, 0,     0)),
        ("South", (-SPEED, 0,     0)),
        ("East",  ( 0,     SPEED, 0)),
        ("West",  ( 0,    -SPEED, 0)),
    ]

    for name, (vx, vy, vz) in directions:
        print(f"{name} 이동...")
        await stream(drone, vx, vy, vz, T)
        await stream(drone, 0,  0,  0,  1.0)  # 호버링
        print(f"{name} 복귀...")
        await stream(drone, -vx, -vy, -vz, T)
        await stream(drone, 0,   0,   0,   1.0)

    print("하강...")
    await stream(drone, 0, 0, 1.0, 2.0)

    await drone.offboard.stop()
    await drone.action.disarm()
    print("완료")

asyncio.run(run())