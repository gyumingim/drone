"""
Landing Gym Environment - RL 학습용 강화학습 환경
"""
import gym
import numpy as np
from gym import spaces
import asyncio
import time


class LandingEnv(gym.Env):
    """AprilTag 기반 드론 착륙 Gym 환경"""

    def __init__(self, px4_bridge, apriltag_detector, world, camera, dt=0.1, max_steps=200):
        """
        초기화

        Args:
            px4_bridge: PX4Bridge 인스턴스
            apriltag_detector: AprilTagDetector 인스턴스
            world: Isaac Sim World 인스턴스
            camera: MonocularCamera 인스턴스
            dt: 시뮬레이션 타임스텝 (초)
            max_steps: 최대 스텝 수
        """
        super().__init__()

        self.px4 = px4_bridge
        self.detector = apriltag_detector
        self.world = world
        self.camera = camera
        self.dt = dt
        self.max_steps = max_steps

        # 상태 변수
        self.step_count = 0
        self.current_image = None

        # Observation space: [tag_x, tag_y, tag_z, vel_x, vel_y, vel_z, yaw]
        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(7,),
            dtype=np.float32
        )

        # Action space: [vx, vy, vz, yaw_rate] (-1 ~ 1, 정규화됨)
        self.action_space = spaces.Box(
            low=np.array([-1.0, -1.0, -1.0, -1.0]),
            high=np.array([1.0, 1.0, 1.0, 1.0]),
            dtype=np.float32
        )

        # 액션 스케일링
        self.max_vel_xy = 2.0  # m/s
        self.max_vel_z = 1.0   # m/s
        self.max_yaw_rate = 45.0  # deg/s

        print("[LandingEnv] Environment initialized")
        print(f"  Obs space: {self.observation_space.shape}")
        print(f"  Action space: {self.action_space.shape}")

    def reset(self):
        """환경 리셋"""
        print("[LandingEnv] Resetting...")
        self.step_count = 0

        # 시뮬레이션 리셋
        self.world.reset()

        # 잠시 대기 (안정화)
        time.sleep(1.0)

        # 초기 observation 반환
        obs = self._get_observation()
        print(f"[LandingEnv] Reset complete. Initial obs: {obs}")
        return obs

    def step(self, action):
        """
        환경 스텝

        Args:
            action: np.array([vx, vy, vz, yaw_rate]) -1~1 정규화

        Returns:
            obs, reward, done, info
        """
        self.step_count += 1

        # 액션 스케일링
        vx = action[0] * self.max_vel_xy
        vy = action[1] * self.max_vel_xy
        vz = action[2] * self.max_vel_z
        yaw_rate = action[3] * self.max_yaw_rate

        # PX4에 속도 명령 전송
        asyncio.get_event_loop().run_until_complete(
            self.px4.send_velocity(vx, vy, vz, yaw_rate)
        )

        # 시뮬레이션 스텝 실행
        self.world.step(render=True)
        time.sleep(self.dt)

        # Observation 얻기
        obs = self._get_observation()

        # 보상 계산
        reward, done = self._compute_reward(obs)

        # 최대 스텝 체크
        if self.step_count >= self.max_steps:
            done = True
            print(f"[LandingEnv] Max steps reached ({self.max_steps})")

        info = {}
        return obs, reward, done, info

    def _get_observation(self):
        """현재 observation 반환"""
        # 카메라 이미지 가져오기
        try:
            rgba = self.camera.get_rgba()
            if rgba is not None and rgba.size > 0:
                self.current_image = rgba.copy()
        except Exception as e:
            print(f"[LandingEnv] Camera error: {e}")

        # AprilTag 감지
        tag_result = self.detector.detect(self.current_image)

        # PX4 상태 가져오기
        px4_state = self.px4.get_state()

        # Observation 구성: [tag_x, tag_y, tag_z, vel_x, vel_y, vel_z, yaw]
        obs = np.array([
            tag_result['position'][0],  # tag_x (forward)
            tag_result['position'][1],  # tag_y (right)
            tag_result['position'][2],  # tag_z (down)
            px4_state['velocity'][0],   # vel_x
            px4_state['velocity'][1],   # vel_y
            px4_state['velocity'][2],   # vel_z
            px4_state['yaw']             # yaw (rad)
        ], dtype=np.float32)

        return obs

    def _compute_reward(self, obs):
        """
        보상 함수

        Args:
            obs: observation

        Returns:
            reward, done
        """
        tag_x, tag_y, tag_z = obs[0], obs[1], obs[2]

        # 수평 거리
        horizontal_dist = np.sqrt(tag_x**2 + tag_y**2)

        # 보상 = -수평거리 - |수직거리|
        reward = -3.0 * horizontal_dist - 1.0 * abs(tag_z)

        done = False

        # 성공 조건: 수평 10cm, 수직 5cm 이내
        if horizontal_dist < 0.1 and abs(tag_z) < 0.05:
            reward += 200.0
            done = True
            print(f"[LandingEnv] SUCCESS! Distance: {horizontal_dist:.3f}m")

        # 실패 조건: 너무 멀리 벗어남 (5m 이상)
        if horizontal_dist > 5.0 or abs(tag_z) > 5.0:
            reward -= 100.0
            done = True
            print(f"[LandingEnv] FAILED! Too far: {horizontal_dist:.3f}m")

        return reward, done

    def render(self, mode='human'):
        """렌더링 (Isaac Sim이 자동으로 렌더링)"""
        pass

    def close(self):
        """환경 종료"""
        print("[LandingEnv] Closing...")
        asyncio.get_event_loop().run_until_complete(self.px4.stop())
