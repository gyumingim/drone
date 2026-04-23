# 시스템 작동 방식

실내 UWB 위치 추정 기반 ArduPilot 자율 비행 시스템.  
UWB 삼변측량 → MAVLink VISION_POSITION_ESTIMATE → ArduPilot EKF3 융합 → GUIDED 모드 자율 비행.

---

## 전체 흐름

```
UWB 태그 (시리얼)
  │  raw distances (앵커 ID, 좌표, 거리)
  ▼
uwb_tag.py :: UWBTag._run()
  │  trilaterate_z_constrained()  →  절대 좌표 (x, y, z)
  │  원점 기준 상대 좌표 계산     →  (rel_x, rel_y, rel_z)  [NED: z↓]
  │
  ├─► VISION_POSITION_ESTIMATE  →  ArduPilot EKF3
  │                                  EK3_SRC1_POSZ=6 일 때 z도 UWB 기준
  │
  └─► get_drone_pos() / get_tags()  →  manual_control.py 에서 읽음

manual_control.py :: main()
  │  connect()          FC 연결 (/dev/ttyACM0)
  │  UWBTag(conn)       UWB 시작, VISION 주입 연결
  │  _heartbeat_loop()  GCS heartbeat 1Hz (백그라운드)
  │  _flight()          비행 시퀀스 (백그라운드 스레드)
  └─  matplotlib 시각화 (메인 스레드, 200ms 갱신)
```

---

## uwb_tag.py

### 시리얼 통신

- 포트: `/dev/ttyUSB0` @ 115200 baud
- 자동 스트림 대기 → 3초 내 데이터 없으면 `les` 폴링 모드 전환 (5Hz)
- 첫 5줄은 raw 로그 출력으로 포맷 확인

### 파싱 (두 가지 포맷 지원)

| 포맷 | 예시 |
|------|------|
| `les` | `A1[1.2,3.4,2.5]=1.80 A2[...]=2.10 ... est[0.5,1.2,0.1,85]` |
| `DIST` | `DIST,3,AN0,A1,1.2,3.4,2.5,1.80,...,POS,0.5,1.2,0.1,85` |

`parse_line()` → DIST 먼저 시도, 실패 시 les 파싱.  
결과: `{anchor_id: {pos: (ax,ay,az), dist_m: float}}` + 펌웨어 추정 `(x,y,z,qf)`

### 삼변측량 (trilaterate_z_constrained)

```
초기값: x0,y0 = 앵커 평균, z0 = 최저 앵커 z - 1.5m  (드론은 앵커 아래)
scipy.optimize.least_squares (Levenberg-Marquardt)
  residuals[i] = ||p - anchor_i|| - dist_i
안전 클램프: 솔버가 앵커 위로 수렴하면 아래로 미러링
필터: anchors < 3 또는 QF < 40 이면 버림
```

### 좌표 변환

첫 유효 fix를 원점 `(ox, oy, oz)`으로 잠금.

```python
rel_x = x - ox          # ENU X (동쪽)
rel_y = y - oy          # ENU Y (북쪽)
rel_z = -(z - oz)       # NED Z (위 = 음수)
                        # 지상: rel_z = 0
                        # 1m 상승: rel_z = -1.0
```

NED 부호 (`rel_z` 음수 = 위)는 ArduPilot EKF 규약과 일치.

### VISION_POSITION_ESTIMATE 주입

```python
vision_position_estimate_send(
    ts_us,          # 현재 시각 (μs)
    rel_x, rel_y, rel_z,
    roll=0, pitch=0, yaw=0,   # 자세 불명 → 0 전송
    covariance=[0.01, ..., 0.01, ..., 0.1, ...],  # pos 0.1m std, angle 0.32rad std
    reset_counter=0,
)
```

10회마다 `[UWB] vision #N pos=(x,y,z)` 출력.

### 원점 전송 (첫 fix 시 1회)

```python
set_gps_global_origin(lat=0, lon=0, alt=0)   # 실내 더미 좌표
set_home_position(lat=0, lon=0, alt=0, ...)
```

---

## manual_control.py

### 초기화 시퀀스

```
connect()
  └─ /dev/ttyACM0 → ACM1 → ACM2 순서로 시도
  └─ wait_heartbeat(timeout=3)

UWBTag(conn).start()          ← VISION 주입 연결
_heartbeat_loop() 시작         ← MAV_TYPE_GCS heartbeat 1Hz
_flight() 스레드 시작
matplotlib 시각화 시작 (메인 스레드)
```

### _flight() 시퀀스

```
1. send_global_origin()         더미 GPS 원점 전송
2. request_streams()            EKF / LOCAL_NED / ATTITUDE / SERVO 10Hz 요청
3. UWB 원점 잠금 대기            최대 20초
4. debug_status(15초)           EKF, UWB, LOCAL_NED, DRIFT 모니터링
5. wait_ready(15초)             EKF 플래그 att+vel+pos_rel+pos_abs 전부 True 대기
6. set_guided()                 GUIDED 모드 + HEARTBEAT custom_mode==4 확인
7. arm()                        ARM + HEARTBEAT MAV_MODE_FLAG_SAFETY_ARMED 확인
8. takeoff(1.0m)                NAV_TAKEOFF → z 상승 감지 완료 대기
9. hold(0, 0, hover_z, 2s)     위치 유지 (UWB 신호 2초 없으면 강제 착지)
10. land()                      NAV_LAND
```

### 이륙 로직 (takeoff)

```python
NAV_TAKEOFF(alt=1.0m) 전송
start_z = 첫 LOCAL_POSITION_NED.z
threshold = start_z - (alt - TOLERANCE)   # = start_z - 0.7m

루프:
  HEARTBEAT armed=False → 즉시 중단
  SERVO 2초마다 로그
  LOCAL_NED.z < threshold → 이륙 완료 (z 기준 0.7m 이상 상승)
  timeout = alt / CLIMB_RATE * 5 = 100초
```

`CLIMB_RATE=0.05` 는 타임아웃 계산 전용 (실제 상승 속도는 FC가 제어).

### 위치 제어 (goto / hold)

```python
# typemask _POS_ONLY: Vx/Vy/Vz/Ax/Ay/Az/Yaw/YawRate 무시
SET_POSITION_TARGET_LOCAL_NED(frame=LOCAL_NED, x, y, z, typemask=_POS_ONLY)

goto(): 3D 거리 < TOLERANCE(0.3m) 도달 시 완료, timeout 30s
hold(): duration 동안 반복 전송, UWB stale > 2s 시 강제 착지
```

### EKF 준비 판정

```python
_EKF_ATT     = 0x001   # 자세 추정
_EKF_VEL_H   = 0x002   # 수평 속도
_EKF_POS_REL = 0x008   # 상대 위치 (UWB 기반)
_EKF_POS_ABS = 0x010   # 절대 위치
_EKF_NEED    = 0x01B   # 위 네 개 전부

(flags & _EKF_NEED) == _EKF_NEED  →  비행 가능
```

### 시각화 (4패널, 200ms 갱신)

| 패널 | 내용 |
|------|------|
| 좌상 (Tag status) | UWB 태그 ID / X / Y / 고도 / QF / 데이터 나이 표 |
| 우상 (XY map) | 2D 평면 위치 + 이동 궤적 |
| 좌하 (FC Telemetry) | armed/mode, LOCAL_NED, 이륙 진행률, SERVO, EKF 플래그, UWB-EKF z 오차 |
| 우하 (Flight log) | 최근 20줄 이벤트 로그, 상태별 색상 |

---

## 파라미터 요약

| 변수 | 값 | 설명 |
|------|----|------|
| `TAKEOFF_ALT` | 1.0 m | 목표 이륙 고도 |
| `CLIMB_RATE` | 0.05 m/s | 이륙 타임아웃 계산용 |
| `TOLERANCE` | 0.3 m | 이륙 완료 / 위치 도달 허용 오차 |
| `UWB_QF_MIN` | 40 | 최소 품질 지수 (0~100) |
| `TRAIL_LEN` | 80 | 궤적 저장 포인트 수 |
| `FC_PORT` | `/dev/ttyACM0` | 비행 컨트롤러 시리얼 포트 |
| `FC_BAUD` | 57600 | FC 통신 속도 |
| `UWB_PORT` | `/dev/ttyUSB0` | UWB 태그 시리얼 포트 |
| `UWB_BAUD` | 115200 | UWB 통신 속도 |

---

## ArduPilot 필수 설정 (QGC)

| 파라미터 | 값 | 이유 |
|----------|----|------|
| `EK3_SRC1_POSZ` | 6 (ExternalNav) | EKF z를 UWB VISION 기준으로 사용 (baro drift 방지) |
| `ARMING_CHECK` | 0 | EKF variance pre-arm 체크 우회 (실내 환경) |
| `VISO_TYPE` | 1 (MAVLink) | VISION_POSITION_ESTIMATE 수신 활성화 |

> **주의:** `EK3_SRC1_POSZ=6` 변경 후 반드시 FC 재부팅 필요. 재부팅 전까지 파라미터 미적용.

---

## 알려진 문제 및 해결 이력

| 증상 | 원인 | 해결 |
|------|------|------|
| ARM 직후 즉시 disarm | EKF 미준비 상태 ARM | EKF_STATUS_REPORT 플래그 확인 후 ARM |
| 이륙 명령 무시 | GUIDED 모드 전환 미확인 | HEARTBEAT custom_mode==4 루프 확인 |
| SERVO=1000, 모터 미작동 | SET_POSITION_TARGET 단독 사용 (land detector 차단) | NAV_TAKEOFF 병행 |
| NAV_TAKEOFF 즉시 완료 | baro drift로 z 이미 목표값 | 절대 조건 → 상대 조건 (start_z - alt) |
| z = -78m (baro drift) | EK3_SRC1_POSZ=1 (baro 기준) → VISION z 완전 무시 | EK3_SRC1_POSZ=6 + 재부팅 필요 |
| VISION z 무시 (혁신 게이트) | EKF가 -93m 초기화, VISION z=0 → 93m 차이 → 거부 | FC 재부팅으로 EKF 재초기화 |
