#!/usr/bin/env python3
"""
Manual Control Script
MAVSDK를 사용하여 PX4 드론을 수동 조작
simple_px4_control.py를 실행한 후 이 스크립트 실행
"""

import asyncio
from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed


async def run():
    """메인 함수"""

    # System 생성
    drone = System()
    print("[Control] Connecting to drone...")
    await drone.connect(system_address="udp://:14540")

    # 연결 대기
    print("[Control] Waiting for drone connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("[Control] Connected!")
            break

    # Offboard 모드 준비 (초기 setpoint를 계속 보냄)
    print("[Control] Setting initial setpoint...")
    print("[Control] Waiting for PX4 to be ready (10 seconds)...")

    for _ in range(100):  # 10초 동안 setpoint 전송
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
        )
        await asyncio.sleep(0.1)

    # Offboard 시작 (arm 전에 시작해야 함)
    print("[Control] Starting offboard mode...")
    await drone.offboard.start()

    # Arm
    print("[Control] Arming...")
    await drone.action.arm()

    print("\n" + "="*70)
    print("   Manual Control Active!")
    print("="*70)
    print("\nCommands:")
    print("- Takeoff: 위로 상승")
    print("- Hover: 제자리 호버링")
    print("- Land: 착륙")
    print("\nPress Ctrl+C to stop")
    print("="*70 + "\n")

    try:
        # Takeoff (2m 상승)
        print("[Control] Taking off...")
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0.0, 0.0, -5, 0.0)  # 위로 0.5 m/s
        )
        await asyncio.sleep(12)

        # Hover
        print("[Control] Hovering...")
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
        )
        await asyncio.sleep(5)

        # 전진
        print("[Control] Moving forward...")
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(5, 0.0, 0.0, 0.0)  # 앞으로 0.5 m/s
        )
        await asyncio.sleep(5)

        # Hover
        print("[Control] Hovering...")
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
        )
        await asyncio.sleep(5)

        # Land
        print("[Control] Landing...")
        for _ in range(40):  # 4초 하강
            await drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(0.0, 0.0, 5, 0.0)  # 아래로 0.5 m/s
            )
            await asyncio.sleep(5)

        # Offboard 중지
        print("[Control] Stopping offboard mode...")
        await drone.offboard.stop()

        # Disarm
        print("[Control] Disarming...")
        await drone.action.disarm()

        print("[Control] Done!")

    except KeyboardInterrupt:
        print("\n[Control] Interrupted by user")
        await drone.offboard.stop()
        await drone.action.disarm()


if __name__ == "__main__":
    asyncio.run(run())
