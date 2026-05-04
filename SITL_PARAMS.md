# SITL 실행 가이드

## 실행 순서

```bash
# 터미널 1: SITL 시작
./sitl_run.sh

# 터미널 2: 비행 스크립트 (MAVProxy 연결 확인 후)
python3 sitl_flight.py

# 터미널 3: XY 궤적 시각화 (선택)
python3 sitl_viz.py
```

## 파일 구조

| 파일 | 역할 |
|---|---|
| `sitl_run.sh` | SITL 시작 + MAVProxy 포트 설정 |
| `sitl.parm` | SITL 파라미터 자동 적용 |
| `sitl_flight.py` | flight.py와 동일한 흐름 (FakeUWB/FakeTagReader 사용) |
| `sitl_viz.py` | LOCAL_POSITION_NED 실시간 XY 궤적 시각화 |
| `lib_fake_sensors.py` | FakeUWB (노이즈 주입), FakeTagReader |

## 포트 구성

| 포트 | 연결 대상 |
|---|---|
| UDP 14550 | QGC 자동 연결 |
| UDP 14551 | `sitl_flight.py` (비행 스크립트) |
| UDP 14552 | `sitl_viz.py` (궤적 시각화) |

## 파라미터 설명

| 파라미터 | SITL 값 | 실기체 값 | 이유 |
|---|---|---|---|
| `GPS_TYPE` | 0 | 1 | GPS 비활성화 |
| `ARMING_CHECK` | 0 | 0 | pre-arm check 비활성화 |
| `VISO_TYPE` | 1 | 1 | MAVLink VPE 수신 |
| `EK3_SRC1_POSXY` | 6 | 6 | XY = ExternalNav |
| `EK3_SRC1_YAW` | 6 | 6 | Yaw = ExternalNav |
| `EK3_SRC1_POSZ` | 2 | 2 | Z = Rangefinder (실기체와 동일) |
| `RNGFND1_TYPE` | 10 | 10 | MAVLink DISTANCE_SENSOR |
| `RNGFND1_MAX_CM` | 1000 | 1000 | 최대 10m |
| `RNGFND1_MIN_CM` | 10 | 10 | 최소 10cm |
| `PSC_POSXY_P` | 0.5 | 0.5 | 진동 감소 (기본값 1.0에서 낮춤) |
| `PSC_VELXY_P` | 1.0 | 1.0 | 속도 제어 |
| `PSC_VELXY_I` | 0.5 | 0.5 | 적분항 |
| `PSC_VELXY_D` | 0.25 | 0.25 | 미분항 |

## 3D 시각화 — FlightGear 연동

ArduPilot SITL은 UDP 5503으로 FlightGear 호환 상태를 자동 출력.
코드 없이 드론 3D 모형(기울기·회전·비행) 확인 가능.

```bash
# FlightGear 설치
sudo apt install flightgear

# sitl_run.sh 대신 직접 실행 시 (FlightGear 연동)
python3 ~/ardupilot/Tools/autotest/sim_vehicle.py \
    -v ArduCopter -w \
    --fg-host=localhost \
    --out=127.0.0.1:14551 \
    --out=127.0.0.1:14552 \
    --add-param-file=sitl.parm
```

FlightGear는 별도 실행 불필요 — `--fg-host` 지정 시 SITL이 자동 연결.

## 노이즈 시나리오

`sitl_flight.py`의 `FakeUWB(noise_m=...)` 값만 바꾸면 됨:

| noise_m | 시나리오 | 목적 |
|---|---|---|
| `0.0` | 이상적 조건 | 기본 흐름 (connect→EKF→TAKEOFF→HOVER→LAND) 검증 |
| `0.1` | 약한 노이즈 | 정상 UWB 환경 시뮬레이션 |
| `0.3` | 강한 노이즈 | 실내 멀티패스 시뮬레이션, 진동 재현 |
