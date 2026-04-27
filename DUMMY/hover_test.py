"""
1m 호버링 테스트 (GPS 없는 환경)
- 로컬 포지션(광류/VIO 등)만 있으면 동작
- offboard 모드로 1m 상공 유지 후 착지
"""

import asyncio
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw

HOVER_ALT   = -1.0   # NED: 음수 = 위 (1m)
HOVER_SEC   = 5.0    # 호버링 유지 시간
ADDRESS     = "serial:///dev/ttyACM0:57600"


async def run():
    drone = System()
    await drone.connect(system_address=ADDRESS)

    print("드론 연결 대기...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("연결됨"); break

    print("로컬 포지션 대기...")
    async for health in drone.telemetry.health():
        if health.is_local_position_ok:
            print("로컬 포지션 OK"); break

    print("ARM...")
    await drone.action.arm()

    # offboard 시작 전 반드시 setpoint 하나 먼저 보내야 함
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    print("offboard 시작...")
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"offboard 시작 실패: {e}")
        await drone.action.disarm()
        return

    print(f"1m 상공으로 상승...")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, HOVER_ALT, 0.0))
    await asyncio.sleep(3.0)   # 상승 대기

    print(f"{HOVER_SEC}초 호버링...")
    await asyncio.sleep(HOVER_SEC)

    print("offboard 정지 → 착지...")
    await drone.offboard.stop()
    await drone.action.land()

    # 착지 완료 대기
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print("착지 완료"); break

    print("완료")


if __name__ == "__main__":
    asyncio.run(run())
