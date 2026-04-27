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

## 주요 문제점 & 해결방안

| 문제 | 원인 | 해결 |
|------|------|------|
| 모터 불동 (arm 후) | RC3=0 → 스로틀 failsafe | RC override ch3=1000 5Hz 전송 |
| 이륙 타임아웃 | vz 임계값(-0.2) 미달, 바로미터 드리프트 오탐 | vz 기반 감지로 변경 |
| 이륙 후 낙하 후 이동 | do_takeoff 조기 리턴 | 고도 도달+속도 안정 조건 추가 |
| yaw=0 강제시 EKF 발산 | UWB 프레임 vs FC 프레임 불일치 | echo-back yaw 유지 |
| SerialException (status_loop) | 멀티쓰레드 시리얼 충돌 | non-blocking + try/except |
