# 드론 프로젝트 STATUS

## 전체 목표
AprilTag + UWB 융합 실내 자율 호버링 드론

## 사용 기술
- ArduCopter 4.6.x, GUIDED 모드
- UWB (DWM1001) — 실내 수평 위치 추정
- RealSense D435i + AprilTag (tag36h11) — 정밀 위치 수렴
- VISION_POSITION_ESTIMATE (MAVLink #102) — EKF 외부 위치 입력
- pymavlink (MAVLink 2), loguru, pyrealsense2

## 파일 구조
| 파일 | 역할 |
|---|---|
| `lib_common.py` | 공유 상수·헬퍼·스레드·connect/takeoff/land/go_to |
| `lib_uwb_reader.py` | DWM1001 시리얼 파싱, origin 보정 |
| `lib_tag_reader.py` | RealSense + AprilTag pose 추출 |
| `flight.py` | UWB 기반 제자리 이착륙 |
| `mission.py` | UWB 기반 웨이포인트 비행 |
| `flight_tag.py` | AprilTag + UWB 융합 호버링 |
| `dbg_monitor.py` | 텔레메트리 + UWB 실시간 모니터링 |
| `dbg_uwb_debug.py` | UWB 시리얼 단독 디버그 |
| `dbg_tag_test.py` | AprilTag 감지 테스트 + 시각화 |

## 필수 FC 파라미터
| 파라미터 | 값 | 의미 |
|---|---|---|
| EK3_SRC1_POSXY | 6 | XY = ExternalNav (UWB) |
| EK3_SRC1_POSZ | 2 | Z = Rangefinder (D435i depth → DISTANCE_SENSOR MAVLink) |
| EK3_SRC1_YAW | 6 | Yaw = ExternalNav |
| ARMING_CHECK | 0 | pre-arm check 비활성화 (mandatory_checks만 실행) |
| VISO_TYPE | 1 | Visual Odometry = MAVLink |
| VISO_YAW_M_NSE | 0.05 | yaw 측정 노이즈(rad). 기본 0.2(≈11.5°) → 0.05로 낮춰야 ArduPilot 내부 하한(5°=0.087rad)까지 활용 가능 |
| VISO_DELAY_MS | 82 | VPE 전송 지연(ms). 카메라 노출→yaw 완료 실측값 82ms. 기본 25ms로 두면 EKF가 잘못된 IMU 시각에 융합함 |
| RNGFND1_TYPE | 10 | MAVLink DISTANCE_SENSOR 수신 |
| RNGFND1_MAX_CM | 1000 | D435i 최대 거리 10m |
| RNGFND1_MIN_CM | 10 | 최소 감지 거리 10cm |

---

## 했던 일

- [x] 파일 접두사 정리 (`lib_*`, `dbg_*`)
- [x] 카메라 180° 장착 보정 → `cv2.ROTATE_180` 이미지 선회전
- [x] loguru 마이그레이션 (전체 파일 `print()` 제거)
- [x] `flight_tag.py` VPE 버그 수정 (`_vision_loop` → `do_takeoff()` 전으로 이동)
- [x] EKF z +131m 버그 수정 (`SET_GPS_GLOBAL_ORIGIN alt=0` → `alt=100000`)
- [x] 호버 위치 고정 개선 (`go_to()` 2Hz 반복 전송 → PosVelAccel 유지)
- [x] `mission.py` 시퀀스 변경 (이륙→1초→북 0.5m→1초 호버→착륙)
- [x] `_hover()` 헬퍼 추가 (go_to 반복 + UWB 끊김 감시)
- [x] 단일 리더 스레드 아키텍처 (recv_match 충돌 해결)
- [x] HARNESS.md 규칙 추가 (깃허브 코드 직접 확인, STATUS.md 수시 업데이트)
- [x] tag yaw → VPE에 직접 사용 (gyro 적분 대신), tag 감지 중 drone_yaw 동기화
- [x] `_COV_UWB[11]` 버그 수정 — ArduPilot 소스 분석으로 발견 (아래 참고)
- [x] D435i depth 고도 소스 추가 — `lib_tag_reader.py` depth 스트림(424×240) + `get_depth_alt()` / `flight_tag.py` DISTANCE_SENSOR 20Hz 전송
- [x] SITL 환경 구성 — `sitl_flight_tag.py`(FakeUWB/FakeTagReader), `sitl_run.sh`(파라미터 자동 적용), `SITL_PARAMS.md`
- [x] `_EKF_NEED` pos_abs(0x010) 제거 — GPS_TYPE=0(SITL/실기체 공통)일 때 ExternalNav만으론 pos_abs 미설정 → EKF 대기 무한루프 수정
- [x] `connect()` force_arm 복원 (param2=21196) — param2=0이면 arm_checks()의 ahrs.healthy()가 EKF 수렴 직후 실패 가능. 21196은 mandatory_checks()만 실행하는 좌측 경로로 전환
- [x] ARM 실패 시 STATUSTEXT 2s 대기 — stop.set() 즉시 호출 시 FC STATUSTEXT 패킷 누락됨
- [x] VIS 로그 1Hz로 제한 — 주석/코드 불일치 수정 (sent%20), INFO 메시지 매몰 방지
- [x] GUIDED ACK 실패 시 abort — 링크 끊김/모드 거부 상태에서 ARM 시도 차단
- [x] `sitl_run.sh` `-w` 플래그 추가 — EEPROM wipe로 이전 빌드의 stale param 제거 (dev 빌드 간 포맷 변경 대응)
- [x] STATUS.md EK3_SRC1_POSZ 중복 수정 — 1(Baro)/2(Rangefinder) 양쪽 기재 → 2로 통일
- [x] FC_PORT SITL 대응 — `udpin:0.0.0.0:14551` (MAVProxy --out=127.0.0.1:14551)

## 하고 있는 일

- [ ] SITL 전체 흐름 검증 (connect→EKF→ARM→TAKEOFF→hover→LAND)

## 할 일

- [ ] 실제 하드웨어 비행 테스트
  - [ ] `dbg_tag_test.py` — AprilTag NED 방향 검증 (north>0 확인)
  - [ ] `dbg_monitor.py` — EKF/UWB 정상 동작 확인
  - [ ] `flight.py` — 이착륙 + 위치 고정 테스트
  - [ ] `mission.py` — 북 0.5m 웨이포인트 테스트
  - [ ] `flight_tag.py` — AprilTag 수렴 테스트
- [ ] SET_GPS_GLOBAL_ORIGIN alt=0 버그 재현 테스트
  - lib_common.py에서 alt=100000 → alt=0으로 임시 변경 후 비행
  - LOCAL_POSITION_NED.z 값이 튀는지 dbg_monitor.py로 확인
  - 재현되면 원인 확정, 안 되면 환경 의존적일 수 있음 (EEPROM 홈 고도 유무 등)
- [ ] EK3_SRC1_POSZ=ExternalNav 고도 호버링 테스트
  - 현재 POSZ=Baro(1) → tag의 tz(카메라↔태그 거리)를 고도로 쓰면 실내 baro보다 정확할 수 있음
  - POSZ=6(ExternalNav)으로 변경 후 flight_tag.py 호버링이 안정적인지 확인
  - 불안정하면 Baro로 복귀 (baro가 실내에서도 상대 고도 기준으로는 충분히 안정적)
- [ ] SET_GPS_GLOBAL_ORIGIN alt 값 조정 + POSZ=ExternalNav 40m 버그 재현 테스트
  - 원인: alt=100000(100m 더미) → EKF origin이 100m MSL → VPE z=0이 "100m MSL" 로 해석
          → 실제 지면과 차이(예: 60m MSL)만큼 고도 오차 발생 (→ 40m 표시)
  - 테스트: lib_common.py에서 alt=100000 → alt=1000(1m)으로 변경
  - 동시에 flight_tag.py TAG 모드 VPE z를 -HOVER_ALT(고정) 대신 -tag_down(실제 거리) 으로 변경
  - 확인: POSZ=ExternalNav에서 고도가 정상(~1m)으로 표시되는지 검증
  - 주의: alt=0은 절대 금지 (+131m EKF Origin Alt=0 Trigger Bug 재발)
  - 불안정하면 Baro로 복귀 (baro가 실내에서도 상대 고도 기준으로는 충분히 안정적)

---

## 문제점 및 해결방안

| 문제 | 원인 | 해결 |
|---|---|---|
| **[EKF Origin Alt=0 Trigger Bug]** EKF z +131m 급상승 | `SET_GPS_GLOBAL_ORIGIN(alt=0)` → ArduPilot이 "해수면=0m"으로 해석 → baro 측정값과 차이를 1Hz마다 재보정 → LOCAL_POSITION_NED.z 급등 | `alt=100000`(100m 더미)로 보정 로직 우회. 정확한 값 불필요, 단 0은 금지 |
| 이륙 중 위치 유실 | `_vision_loop` takeoff 후 시작 | takeoff 전으로 이동 |
| 호버 중 drift | `hold_position()` VelAccel만 유지, 바람에 복귀 안 함 | `go_to()` 2Hz 반복 전송 |
| 180° 카메라 NED 오류 | raw 이미지 뒤집힘 | `cv2.ROTATE_180` 선회전 |
| roll/pitch NaN | 멀티스레드 recv_match 충돌 | 단일 리더 스레드 |
| yaw=0 고정 | 동일한 멀티스레드 충돌 | 단일 리더 스레드 |
| ARM FAILED | pre-arm check | ARMING_CHECK=0 |
| UWB fallback 시 XY 위치 무시 | `_COV_UWB[11]=9999` → GCS_Common.cpp가 `posErr=sqrt(cov[0]+cov[6]+cov[11])`로 합산 → posErr≈100m → EKF가 UWB XY까지 무시 | `_COV_UWB[11]=0.25` (EK3_SRC1_POSZ=RangeFinder가 z를 이미 처리하므로 9999 불필요) |
