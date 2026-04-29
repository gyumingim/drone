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
| EK3_SRC1_POSZ | 1 | Z = Baro |
| EK3_SRC1_YAW | 6 | Yaw = ExternalNav |
| ARMING_CHECK | 0 | pre-arm check 비활성화 |
| VISO_TYPE | 1 | Visual Odometry = MAVLink |

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

## 하고 있는 일

- [ ] GitHub MCP 서버 설치 (PAT 발급 대기 중)

## 할 일

- [ ] GitHub MCP 설치 완료
- [ ] 실제 하드웨어 비행 테스트
  - [ ] `dbg_tag_test.py` — AprilTag NED 방향 검증 (north>0 확인)
  - [ ] `dbg_monitor.py` — EKF/UWB 정상 동작 확인
  - [ ] `flight.py` — 이착륙 + 위치 고정 테스트
  - [ ] `mission.py` — 북 0.5m 웨이포인트 테스트
  - [ ] `flight_tag.py` — AprilTag 수렴 테스트

---

## 문제점 및 해결방안

| 문제 | 원인 | 해결 |
|---|---|---|
| EKF z +131m 급상승 | `SET_GPS_GLOBAL_ORIGIN(alt=0)` → baro 1Hz 리셋 ArduPilot 버그 | `alt=100000`(100m 더미) |
| 이륙 중 위치 유실 | `_vision_loop` takeoff 후 시작 | takeoff 전으로 이동 |
| 호버 중 drift | `hold_position()` VelAccel만 유지, 바람에 복귀 안 함 | `go_to()` 2Hz 반복 전송 |
| 180° 카메라 NED 오류 | raw 이미지 뒤집힘 | `cv2.ROTATE_180` 선회전 |
| roll/pitch NaN | 멀티스레드 recv_match 충돌 | 단일 리더 스레드 |
| yaw=0 고정 | 동일한 멀티스레드 충돌 | 단일 리더 스레드 |
| ARM FAILED | pre-arm check | ARMING_CHECK=0 |
