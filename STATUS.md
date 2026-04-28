# 드론 프로젝트 진행 상황

## 전체 목표
UWB(XY) + 바로미터(Z) 기반 실내 자율 비행 — ArduPilot GUIDED 모드

## 사용 기술
- FC: ArduPilot (GUIDED mode, MAVLink)
- XY 위치: DWM1001 UWB → VISION_POSITION_ESTIMATE
- Z 위치: EKF3 바로미터 (EK3_SRC1_POSZ=1)
- 통신: pymavlink (USB serial /dev/ttyACM0 57600baud)
- UWB: /dev/ttyUSB0 115200baud, `lec` 명령 (CSV DIST 포맷), 삼변측량은 소프트웨어에서 직접 계산

## 파일 구조
| 파일 | 역할 |
|------|------|
| `common.py` | 공유 상수·헬퍼·쓰레드·connect/do_takeoff/do_land/go_to/wait_pos |
| `flight.py` | 제자리 이착륙 (1m 호버 5s) |
| `mission.py` | 이륙 → 동쪽 1m → 원점 복귀 → 착륙 |
| `uwb_reader.py` | UWB 시리얼 파싱 (lec 포맷, scipy 삼변측량) |
| `monitor.py` | 비행 없이 텔레메트리 모니터링 전용 |
| `dashboard.py` | Rich TUI 대시보드 (DroneData + render) |

## 필수 FC 파라미터 (Mission Planner에서 설정)

| 파라미터 | 값 | 의미 |
|----------|-----|------|
| EK3_SRC1_POSXY | 6 | XY 위치 소스 = ExternalNav (UWB) |
| EK3_SRC1_POSZ | 1 | Z 위치 소스 = Baro |
| EK3_SRC1_YAW | 6 | Yaw 소스 = ExternalNav |
| ARMING_CHECK | 0 | pre-arm check 전부 비활성화 (가속도계 캘리 후 ARM FAILED 우회용) |
| VISO_TYPE | 1 | Visual Odometry = MAVLink |

> **비행 전 반드시**: ARMING_CHECK=0 설정되어 있는지 확인. 없으면 MAVLink ARM 항상 실패.

## UWB 앵커 구성

- 앵커 3개: `33BB`, `31D1`, `31D0`
- 태그 → 앵커 거리 → `uwb_reader.py`에서 scipy 삼변측량으로 XY 계산
- lec 포맷: `DIST,N,0,ID,x,y,z,dist,...` (N=앵커 수)
- 앵커 좌표는 DWM1001 네트워크 설정에 저장됨 (lec 응답에 포함)

## 실행 방법

```bash
# 1. 모니터링만 (비행 없음, 텔레메트리 확인용)
cd /home/karma/drone
python3 monitor.py

# 2. 제자리 이착륙 (1m 호버 5s)
python3 flight.py

# 3. 동쪽 1m 왕복 미션
python3 mission.py
```

**주의사항:**
- 실행 전 드론을 UWB +x 방향으로 물리적 정렬 필요 (yaw 기준)
- FC 리부팅 후 실행 권장 (baro 기준점 리셋)
- ARMING_CHECK=0 확인 필수

## 현재 동작 상태 (2026-04-28 기준)

| 스크립트 | 상태 | 비고 |
|----------|------|------|
| `monitor.py` | 동작 확인 | Rich 대시보드 정상 출력 |
| `flight.py` | 동작 확인 | 1m 호버 후 착륙, 위치 유지 확인 |
| `mission.py` | 미테스트 | 코드 작성 완료, 실제 비행 미확인 |

## 완료한 것
- [x] UWB XY + 바로미터 Z EKF3 설정 확인
- [x] VISION_POSITION_ESTIMATE 20Hz 전송 (yaw echo-back)
- [x] RC override 루프로 RC failsafe 방지 (모터 불동 버그 수정)
- [x] GCS heartbeat 1Hz 유지
- [x] EKF 준비 대기 (flags 0x033f 확인)
- [x] COMMAND_ACK 확인 (모든 명령)
- [x] 제자리 이착륙 동작 확인 (flight.py) — hover 5s 후 원점 복귀 확인
- [x] common.py 분리 리팩토링
- [x] mission.py 기본 구조 작성
- [x] UWB lec 포맷으로 전환 + scipy 삼변측량 직접 구현
- [x] dashboard.py + monitor.py 작성 (비행 없이 텔레메트리 확인)
- [x] ARM FAILED 시 즉시 종료 처리 (connect() 반환값 None 처리)
- [x] do_takeoff() 디버그 로그 강화 (x,y,vx,vy,roll,pitch 추가)
- [x] EKF baro 오프셋 버그 원인 파악 (리부팅으로 해결, SET_GPS_GLOBAL_ORIGIN 충돌)

## 하고 있는 것
- [ ] 이륙 중 vy 드리프트 원인 분석 (0.1~0.34 m/s y+ 방향)
  - 현상: 이륙 중 vy 지속적 양수, z 도달 후 안정
  - 의심: UWB 앵커 배치 오류 / 드론 기울기 (roll/pitch NaN이라 확인 불가)
  - 다음 단계: roll/pitch NaN 해결 → 실제 기울기 확인

## 할 것
- [ ] roll/pitch NaN 해결 — ATTITUDE recv 타이밍 이슈 (LOCAL_POSITION_NED blocking 전에 ATTITUDE 먼저 받아야)
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
| ARM ACK: FAILED | 가속도계 캘리 후 pre-arm check 강화 | ARMING_CHECK=0 설정 (GCS/Mission Planner) |
| z=+4.587m 이륙 안됨 | SET_GPS_GLOBAL_ORIGIN(alt=0) + EEPROM 홈 고도 충돌 | FC 리부팅 (EKF baro 기준점 리셋) |
| UWB XY 미출력 | 장치가 lep auto-stream 모드로 고정 | lec 명령 재전송 + 접속시 escape sequence 전송 |
| roll/pitch = NaN | ATTITUDE 메시지가 LOCAL_POSITION_NED blocking 이후 도착 | 해결 예정 — non-blocking 수신 순서 조정 필요 |
| 이륙 중 vy 드리프트 | 원인 조사 중 (UWB 앵커 오프셋 or 물리적 기울기) | roll/pitch NaN 해결 후 재확인 |
