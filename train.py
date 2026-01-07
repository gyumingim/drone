"""
Train - PPO 강화학습 학습 스크립트
"""
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
from landing_env import LandingEnv
import main  # Isaac Sim + Pegasus 초기화용


def train():
    """PPO 학습 실행"""
    print("="*70)
    print("   AprilTag Landing - RL Training with PPO")
    print("="*70)

    # Isaac Sim + Pegasus 환경 초기화 (main.py에서 가져옴)
    px4, detector, world, camera = main.setup_simulation()

    # Gym 환경 생성
    env = LandingEnv(
        px4_bridge=px4,
        apriltag_detector=detector,
        world=world,
        camera=camera,
        dt=0.1,
        max_steps=200
    )

    # PPO 모델 생성
    print("[Train] Creating PPO model...")
    model = PPO(
        "MlpPolicy",
        env,
        learning_rate=3e-4,
        n_steps=2048,
        batch_size=64,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.01,
        verbose=1,
        tensorboard_log="./tensorboard_logs/"
    )

    # 체크포인트 콜백
    checkpoint_callback = CheckpointCallback(
        save_freq=10000,
        save_path="./checkpoints/",
        name_prefix="landing_ppo"
    )

    # 학습 시작
    print("[Train] Starting training...")
    print(f"  Total timesteps: 1,000,000")
    print(f"  Save path: ./checkpoints/")

    model.learn(
        total_timesteps=1_000_000,
        callback=checkpoint_callback,
        log_interval=10
    )

    # 모델 저장
    model.save("landing_ppo_final")
    print("[Train] Training complete! Model saved to 'landing_ppo_final.zip'")

    # 환경 종료
    env.close()


if __name__ == "__main__":
    train()
