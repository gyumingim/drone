#!/usr/bin/env python
"""
| 파일: 1_px4_single_vehicle.py
| 저자: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| 라이선스: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
| 설명: Pegasus API를 사용하여 단일 무인 항공기(Vehicle) 시뮬레이션을 구축하는 예제입니다.
|       이 예제에서는 MAVLink 제어 백엔드를 사용하여 드론을 제어합니다.
"""

# Isaac Sim을 스크립트에서 실행하기 위한 임포트
import carb
from isaacsim import SimulationApp

# Isaac Sim 시뮬레이션 환경 시작
# 참고: 이 SimulationApp 객체는 다른 라이브러리 임포트 직후에 바로 생성되어야 합니다.
# 그렇지 않으면 시뮬레이터가 확장 기능을 로드하거나 실제 엔진을 구동할 때 충돌(Crash)이 발생할 수 있습니다.
simulation_app = SimulationApp({"headless": False})

# -----------------------------------
# 여기서부터 실제 스크립트 로직이 시작됩니다.
# -----------------------------------
import omni.timeline
from omni.isaac.core.world import World

# 드론 시뮬레이션을 위한 Pegasus API 임포트
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.state import State
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

# 보조 라이브러리 (scipy, numpy 등)
from scipy.spatial.transform import Rotation

class PegasusApp:
    """
    간단한 Isaac Sim 독립형 앱(Standalone App) 구축 방법을 보여주는 템플릿 클래스입니다.
    """

    def __init__(self):
        """
        PegasusApp을 초기화하고 시뮬레이션 환경을 설정하는 메서드입니다.
        """

        # 시뮬레이션 시작/정지를 제어하기 위한 타임라인 인터페이스 획득
        self.timeline = omni.timeline.get_timeline_interface()
        # Pegasus 인터페이스 시작
        self.pg = PegasusInterface()
        # World 객체 획득. 월드는 물리 설정, 에셋 생성 등을 총괄하는 싱글톤 객체입니다.
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world
        # NVIDIA에서 제공하는 기본 시뮬레이션 환경 중 하나를 로드합니다 (Curved Gridroom)
        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])
        # 드론(Vehicle) 생성
        # 선택한 로봇 모델을 지정된 네임스페이스에 스폰(Spawn)합니다.
        config_multirotor = MultirotorConfig()
        # MAVLink 백엔드 설정 (PX4 연결용)
        mavlink_config = PX4MavlinkBackendConfig({
            "vehicle_id": 0,                # 드론의 고유 ID
            "px4_autolaunch": True,         # PX4 SITL을 자동으로 함께 실행할지 여부
            "px4_dir": self.pg.px4_path,    # PX4 소스 코드 경로
            "px4_vehicle_model": 'iris' # 사용할 기체 모델 (v1.14 미만은 'iris'로 수정 필요)
        })
        # 드론 설정에 MAVLink 백엔드 추가
        config_multirotor.backends = [PX4MavlinkBackend(mavlink_config)]
        # 실제 월드에 드론 배치
        Multirotor(
            "/World/quadrotor",             # 드론이 생성될 USD 경로
            ROBOTS['Iris'],                 # 로봇 모델 (Iris 쿼드콥터)
            0,                              # 기체 인덱스
            [0.0, 0.0, 0.07],               # 초기 생성 위치 (X, Y, Z)
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(), # 초기 자세 (쿼터니언)
            config=config_multirotor,       # 위에서 만든 설정 적용
        )
        # 모든 관절 및 로봇이 초기화되도록 시뮬레이션 환경을 리셋합니다.
        self.world.reset()
        # 타임라인 콜백 예제를 위한 보조 변수
        self.stop_sim = False

    def run(self):
        """
        애플리케이션의 메인 루프를 실행하고 물리 단계를 수행하는 메서드입니다.
        """
        # 시뮬레이션 타임라인 재생 (시작)
        self.timeline.play()
        # 앱이 실행되는 동안 무한 루프 수행
        while simulation_app.is_running() and not self.stop_sim:

            # 앱의 UI를 업데이트하고 물리 스텝(Physics Step)을 한 단계 진행합니다.
            self.world.step(render=True)
        
        # 종료 처리 및 로그 출력
        carb.log_warn("PegasusApp 시뮬레이션 앱을 종료합니다.")
        self.timeline.stop()
        simulation_app.close()

def main():

    # 템플릿 앱 객체 생성
    pg_app = PegasusApp()

    # 애플리케이션 루프 실행
    pg_app.run()

if __name__ == "__main__":
    main()