# 드론 프로젝트 진행 상황

## 전체 목표
UWB(XY) + 바로미터(Z) 기반 실내 자율 비행 — ArduPilot GUIDED 모드

## 사용 기술
- FC: ArduPilot (GUIDED mode, MAVLink)
- XY 위치: DWM1001 UWB → VISION_POSITION_ESTIMATE
- Z 위치: EKF3 바로미터 (EK3_SRC1_POSZ=1)
- 통신: pymavlink (USB serial /dev/ttyACM0 57600baud)
- UWB: /dev/ttyUSB0 115200baud, `les` 명령 스트리밍

## 파일 구조
| 파일 | 역할 |
|------|------|
| `common.py` | 공유 상수·헬퍼·쓰레드·connect/do_takeoff/do_land/go_to/wait_pos |
| `flight.py` | 제자리 이착륙 (1m 호버 5s) |
| `mission.py` | 이륙 → 동쪽 1m → 원점 복귀 → 착륙 |
| `uwb_reader.py` | UWB 시리얼 파싱 (les 포맷) |

## 완료한 것
- [x] UWB XY + 바로미터 Z EKF3 설정 확인
- [x] VISION_POSITION_ESTIMATE 20Hz 전송 (yaw echo-back)
- [x] RC override 루프로 RC failsafe 방지 (모터 불동 버그 수정)
- [x] GCS heartbeat 1Hz 유지
- [x] EKF 준비 대기 (flags 0x033f 확인)
- [x] COMMAND_ACK 확인 (모든 명령)
- [x] 제자리 이착륙 동작 확인 (flight.py)
- [x] common.py 분리 리팩토링
- [x] mission.py 기본 구조 작성

## 하고 있는 것
- [ ] mission.py 이착륙 후 이동 버그 수정
  - 증상: takeoff 후 낙하, 그 다음 지면에서 동쪽 이동
  - 원인: do_takeoff가 vz<-0.2 감지 즉시 리턴 → 고도 미달 상태에서 go_to 호출 → TAKEOFF 중단
  - 수정: z < -(takeoff_m - 0.2) AND vz < 0.15 조건으로 실제 고도 도달 확인 후 리턴

## 할 것
- [ ] mission.py 실제 테스트 및 검증
- [ ] wait_pos 도달 조건 튜닝 (현재 tol=0.3m)
- [ ] 미션 확장 (동서남북 4방향 순차 이동)
- [ ] UWB 신호 끊김 안전 로직 mission.py에도 추가

## 좌표계 정렬 주의사항

**현재 코드는 UWB (x,y) → NED (north,east) 로 그대로 매핑함.**
따라서 `go_to(0, 1, -1)` 가 실제로 "동쪽 1m" 로 날아가려면:

> **드론 시작 시 UWB +x 방향을 향하고 있어야 함**

- EK3_SRC1_YAW=6 (ExternalNav): 코드가 초기에 yaw=0.0 을 EKF에 전송
- EKF가 그 순간 드론이 향하던 방향을 NED North(0°)로 고정
- 이후 IMU(자이로) 적분으로 yaw 변화 추적 + echo-back으로 드리프트 방지
- 결론: UWB +x ≠ 드론 초기 헤딩이면 go_to 이동 방향이 틀어짐

## Yaw 정확도 향상 방법 조사 결과

| 방법 | 정확도 | 실내 사용 | 비고 |
|------|--------|----------|------|
| **나침반 (기본값)** | 중 | △ | 모터·ESC 자기 간섭 심함, 실내 부적합 |
| **IMU 자이로 적분 (현재)** | 낮음 | ○ | 장시간 드리프트 누적 |
| **Dual GPS yaw** | 높음 | ✗ | GPS 없는 실내 사용 불가 |
| **GSF (EK3_SRC1_YAW=8)** | 중 | ✗ | GPS velocity 기반, 실내 불가 |
| **광학 흐름 센서 (Optical Flow)** | 중 | ○ | 하향 카메라 필요, yaw 직접 측정은 아님 |
| **비전 오도메트리 (T265 등)** | 높음 | ○ | 카메라 기반 6DOF, yaw 포함 |
| **UWB 2태그 헤딩** | 높음 | ○ | 드론에 UWB 태그 2개 장착 → 두 좌표 차로 heading 계산 |
| **운용 규칙 (현실적)** | 중 | ○ | 매 비행마다 드론을 UWB +x 방향으로 정렬하고 시작 |

**가장 현실적인 단기 해결책**: 드론 시작 전 UWB +x 축 방향으로 물리적으로 정렬하는 운용 규칙 적용.
**장기 해결책**: UWB 태그 2개로 heading 직접 계산 or 비전 오도메트리 추가.

ref: [ArduPilot Compassless Operation](https://ardupilot.org/copter/docs/common-compassless.html), [EKF Source Selection](https://ardupilot.org/copter/docs/common-ekf-sources.html)

## 주요 문제점 & 해결방안

| 문제 | 원인 | 해결 |
|------|------|------|
| 모터 불동 (arm 후) | RC3=0 → 스로틀 failsafe | RC override ch3=1000 5Hz 전송 |
| 이륙 타임아웃 | vz 임계값(-0.2) 미달, 바로미터 드리프트 오탐 | vz 기반 감지로 변경 |
| 이륙 후 낙하 후 이동 | do_takeoff 조기 리턴 | 고도 도달+속도 안정 조건 추가 |
| yaw=0 강제시 EKF 발산 | UWB 프레임 vs FC 프레임 불일치 | echo-back yaw 유지 |
| SerialException (status_loop) | 멀티쓰레드 시리얼 충돌 | non-blocking + try/except |
| RC로 멈추기 불가 | RC override가 ch1~ch8 전부 덮어씀 | ch3만 override, 나머지 release(0) |
