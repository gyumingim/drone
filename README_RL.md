# AprilTag 기반 드론 착륙 강화학습

Isaac Sim + Pegasus + PX4 SITL + MAVSDK를 사용한 드론 착륙 강화학습 시스템

## 📁 파일 구조

```
drone/
├── px4_bridge.py           # MAVSDK Offboard 컨트롤러
├── apriltag_detector.py    # AprilTag 감지기
├── landing_env.py          # Gym 강화학습 환경
├── train.py                # PPO 학습 스크립트
└── main.py                 # 통합 초기화 및 테스트
```

## 🔧 필수 패키지 설치

```bash
# Stable Baselines3 (RL 라이브러리)
pip install stable-baselines3

# Gym (이미 설치되어 있어야 함)
pip install gym

# MAVSDK (이미 설치됨)
# pip install mavsdk
```

## 🚀 사용 방법

### 1단계: PX4 SITL 실행

**터미널 1**에서:
```bash
cd ~/PX4-Autopilot

# SIH 모드로 실행 (자체 물리 엔진 사용)
make px4_sitl none_iris

# 또는 기본 설정
make px4_sitl
```

**중요**: PX4 SITL이 센서 캘리브레이션 에러로 arming을 거부할 경우, PX4 콘솔에서 다음 중 하나를 실행하세요:

**방법 1: 가속도계 캘리브레이션 (권장)**
```bash
commander calibrate accelerometer
# 프롬프트가 나오면 그냥 Enter 누르기
commander calibrate gyro
```

**방법 2: Preflight check 비활성화**
```bash
param set COM_ARM_WO_GPS 1
param set SYS_HAS_MAG 0
param set SYS_HAS_BARO 0
param set CBRK_SUPPLY_CHK 894281
param set COM_RCL_EXCEPT 4
param set COM_ARM_EKF_AB 0
param set COM_ARM_MAG_STR 0
param set CBRK_AIRSPD_CHK 162128
param save
```

설정 후 PX4를 재시작하세요 (`Ctrl+C` 후 다시 `make px4_sitl none_iris`).

### 2단계: AprilTag 감지 테스트

학습 전에 감지가 잘 되는지 테스트:

```bash
python main.py
```

- Isaac Sim 창이 열림
- 드론과 AprilTag 타겟 확인
- 콘솔에서 감지 결과 확인
- Ctrl+C로 종료

### 3단계: 강화학습 학습

```bash
python train.py
```

- 1,000,000 timesteps 학습
- 10,000 스텝마다 체크포인트 저장 (`./checkpoints/`)
- TensorBoard 로그 저장 (`./tensorboard_logs/`)
- 최종 모델: `landing_ppo_final.zip`

### 4단계: 학습 모니터링

**터미널 3**에서:
```bash
tensorboard --logdir=./tensorboard_logs/
```

브라우저에서 `http://localhost:6006` 접속

## 📊 시스템 구조

```
┌─────────────────────────────────────────┐
│         Isaac Sim (물리 엔진)           │
│  ┌───────────────────────────────┐      │
│  │   Pegasus Simulator           │      │
│  │   - Iris Quadrotor            │      │
│  │   - Downward Camera           │      │
│  └───────────────────────────────┘      │
└──────────────┬──────────────────────────┘
               │ MAVLink (UDP 14540)
               ▼
    ┌──────────────────────┐
    │    PX4 SITL          │
    │  (Offboard Mode)     │
    └──────────┬───────────┘
               │ MAVSDK
               ▼
    ┌──────────────────────┐
    │   Python Script      │
    │  - PX4Bridge         │
    │  - AprilTagDetector  │
    │  - LandingEnv (Gym)  │
    │  - PPO (SB3)         │
    └──────────────────────┘
```

## 🎮 Gym 환경 사양

### Observation Space (7차원)
- `tag_x`: AprilTag X 위치 (body frame, forward, m)
- `tag_y`: AprilTag Y 위치 (body frame, right, m)
- `tag_z`: AprilTag Z 위치 (body frame, down, m)
- `vel_x`: 드론 X 속도 (body frame, m/s)
- `vel_y`: 드론 Y 속도 (body frame, m/s)
- `vel_z`: 드론 Z 속도 (body frame, m/s)
- `yaw`: 드론 Yaw 각도 (rad)

### Action Space (4차원, -1~1 정규화)
- `vx`: Forward 속도 (-2 ~ 2 m/s)
- `vy`: Right 속도 (-2 ~ 2 m/s)
- `vz`: Down 속도 (-1 ~ 1 m/s)
- `yaw_rate`: Yaw 각속도 (-45 ~ 45 deg/s)

### Reward Function
```python
reward = -3.0 * horizontal_distance - 1.0 * abs(vertical_distance)

# 성공: horizontal < 0.1m, vertical < 0.05m → +200
# 실패: horizontal > 5m or vertical > 5m → -100
```

## ⚙️ 설정 변경

### CONFIG (main.py)
```python
class CONFIG:
    CAMERA_FOV_DEG = 150.0           # 카메라 시야각
    CAMERA_RESOLUTION = (1280, 720)  # 해상도
    TAG_SIZE = 0.5                   # AprilTag 크기 (m)
    TAG_POSITION = [2.0, 0.0, 0.0]   # 타겟 위치
```

### 학습 파라미터 (train.py)
```python
model = PPO(
    learning_rate=3e-4,  # 학습률
    n_steps=2048,        # 배치당 스텝 수
    batch_size=64,       # 미니배치 크기
    gamma=0.99,          # Discount factor
    ...
)
```

## 🐛 문제 해결

### PX4 SITL 연결 실패
```
[PX4Bridge] Waiting for connection...
```

**해결**: PX4 SITL이 실행 중인지 확인
```bash
ps aux | grep px4
```

### AprilTag 감지 안 됨
```
[Test] Step X: No tag detected
```

**해결**:
1. 카메라 FOV 확인 (150°)
2. AprilTag 위치 확인 (드론 앞 2m)
3. 조명 확인 (Sun light 생성됨?)

### Gym 환경 에러
```
ModuleNotFoundError: No module named 'stable_baselines3'
```

**해결**:
```bash
pip install stable-baselines3
```

## 📝 다음 단계

1. **학습 완료 후 평가**:
```python
from stable_baselines3 import PPO

model = PPO.load("landing_ppo_final")
# 평가 코드 작성
```

2. **하이퍼파라미터 튜닝**:
- Learning rate
- Batch size
- 보상 함수 가중치

3. **이동 타겟 추가**:
- MovingRover 클래스 재활용
- 난이도 증가

## 📚 참고 문서

- [Pegasus Simulator](https://pegasussimulator.github.io/)
- [PX4 Offboard Control](https://docs.px4.io/main/en/flight_modes/offboard.html)
- [MAVSDK Python](https://mavsdk.mavlink.io/main/en/python/)
- [Stable Baselines3](https://stable-baselines3.readthedocs.io/)
