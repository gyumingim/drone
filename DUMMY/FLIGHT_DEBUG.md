# 이륙 실패 분석 및 수정 이력

## 전체 목표
실내 UWB 위치 추정 기반으로 드론 자율 이륙 → 호버링 → N/E/S/W 30cm 이동 → 귀환 → 착지

## 현재 상태
- 이륙 로직: 하이브리드 (NAV_TAKEOFF 모터 트리거 + SET_POSITION_TARGET 정밀 제어)
- 모터 스핀업까지는 성공. baro drift가 클 때 DISARM_DELAY 타임아웃 발생 중.

---

## 사용 기술
- ArduPilot GUIDED 모드 + MAVLink (pymavlink)
- UWB trilateration → VISION_POSITION_ESTIMATE → EKF3 융합
- MAV_CMD_NAV_TAKEOFF + SET_POSITION_TARGET_LOCAL_NED 하이브리드

---

## 실패 유형별 분석

### F-1. EKF 미준비 상태 ARM
**증상:** ARM 직후 즉시 disarm  
**원인:** EKF가 UWB 위치 융합 완료 전 GUIDED 모드 진입  
**수정:** `EKF_STATUS_REPORT` 플래그 `pos_abs=True`, `pos_rel=True` 확인 후 ARM

---

### F-2. GUIDED 모드 전환 미확인
**증상:** 이륙 명령 무시  
**원인:** `set_mode()` 직후 ARM → FC가 모드 전환 완료 전에 ARM 명령 도착  
**수정:** ARM 전 HEARTBEAT `custom_mode == 4` 루프로 GUIDED 확인

---

### F-3. SET_POSITION_TARGET 단독 이륙
**증상:** SERVO=1000, 모터 미작동  
**원인:** 이 명령은 이미 비행 중인 드론 제어용. land detector가 "지상"으로 판단하면 모터 출력 차단.  
**수정:** MAV_CMD_NAV_TAKEOFF으로 교체 (land detector 명시 우회)

---

### F-4. NAV_TAKEOFF 절대 고도 조건 즉시 충족
**증상:** 모터 미작동, 이륙 루프 즉시 종료  
**원인:** baro drift로 z 이미 -2~-3m인 상태에서 `z < -(alt - TOLERANCE)` 절대값 조건 사용  
→ ARM 직후 조건 참, 모터 안 켰는데 "이륙 완료" 처리  
**수정:** 절대 조건 → 상대 조건으로 변경: `threshold = start_z - (alt - TOLERANCE)`

---

### F-5. baro drift 음수 방향에서 NAV_TAKEOFF 즉시 거부 (z=-1.30m)
**증상:** NAV_TAKEOFF 전송 후 ~1초 내 disarm  
**원인:** `start_z = -1.30m`이면 EKF는 "드론이 1.3m 위"로 인식.  
Land detector: EKF(공중)와 물리 센서(지상) 충돌 → NAV_TAKEOFF 거부 또는 "크래시 감지" 즉시 disarm  
**수정 (하이브리드):** `nav_alt = abs(start_z) + alt + 5.0`  
→ drift 방향 무관하게 NAV_TAKEOFF 목표를 현재 EKF z보다 항상 위로 설정

---

### F-6. baro drift 양수 방향 (z=+2.05m) + SET_POSITION_TARGET 단독
**증상:** SERVO=1000, disarm  
**원인:** F-3와 동일 (land detector 차단). baro drift 방향만 반대.  
**수정:** F-5 하이브리드 방식으로 통합 해결

---

### F-7. position setpoint 1m 오차 → 최소 출력 → DISARM_DELAY 만료 (해결됨)

**증상 (실행 15:22, 15:29):**
```
start_z = -5.66m / -4.08m  (baro drift)
target_z = start_z - 1.0   → position error = 1m only
SERVO = 1100 (최소) 유지 → 드론 미이륙
ARM 후 ~11s → TAKEOFF ABORTED: drone disarmed
```

**근본 원인 (검증됨):**

position controller 오차 계산:
```
error = target_z - current_z = (start_z - 1.0) - start_z = -1.0m
```
baro drift가 얼마든 position error는 항상 1m. position controller는 hover + 1m분의
소량 추력만 인가 → SERVO=1100(최솟값). 드론은 실제로 지상에 있으므로 이 출력으로는 이륙 불가.

**왜 DO_SET_HOME으로 해결 안 됨 (F-8 참조):**
DO_SET_HOME은 navigation home만 바꾸고 EKF 로컬 프레임(LOCAL_NED z 기준)에는 무관.

**수정: velocity setpoint으로 전환**

position target(오차=1m) 대신 velocity target(vz=-0.3m/s):
- velocity controller: "지금보다 0.3m/s 위로" → baro drift 무관하게 실제 추력 인가
- 완료 판정: UWB z로 실제 고도 확인 (EKF baro 기준 아님)
- typemask=3527 (velocity-only, X/Y/Z position 무시)
  출처: ardupilot.org/dev/docs/copter-commands-in-guided-mode.html

---

### F-8. MAV_CMD_DO_SET_HOME이 LOCAL_NED z에 무효

**증상:** HOME RESET: no ACK, z 여전히 -4.08m
**원인:**
- DO_SET_HOME = RTL 귀환 좌표 변경 명령
- LOCAL_POSITION_NED z = EKF 로컬 프레임 기준 (EKF 초기화 시 baro 기준)
- home 재설정은 EKF 로컬 프레임과 완전히 독립 → z 불변
**근거:** MAVLink spec DO_SET_HOME 설명, ArduPilot mode_guided.cpp
**결론:** 잘못된 접근. 코드에서 제거.

---

## 현재 이륙 로직 (하이브리드 v2 — velocity 기반)

```
ARM
└─ takeoff(conn, alt, uwb)
     ├─ LOCAL_NED 읽기 → start_z, hold_x, hold_y  (EKF, baro 기준)
     ├─ UWB z 읽기 → uwb_ground_z                  (실제 고도 기준)
     ├─ nav_alt = abs(start_z) + alt + 5.0
     ├─ UWB threshold = uwb_ground_z - (alt - TOLERANCE)
     ├─ NAV_TAKEOFF(nav_alt) 전송  ← land detector 우회, 모터 스핀업
     └─ 루프
          ├─ SERVO > 1100 → motors_on = True
          ├─ motors_on → send_velocity(0, 0, -CLIMB_RATE)  ← 0.2s 간격
          │   typemask=3527 velocity-only, baro drift 무관
          ├─ HEARTBEAT armed=False → abort
          ├─ UWB z < uwb_threshold → TAKEOFF COMPLETE (primary)
          └─ EKF z < ekf_threshold → TAKEOFF COMPLETE (fallback, UWB 없을 때)
```

## 코드 파라미터
| 변수 | 값 | 역할 |
|---|---|---|
| `TAKEOFF_ALT` | 1.0m | 목표 호버링 고도 |
| `CLIMB_RATE` | 0.3m/s | velocity 상승 속도 (PILOT_SPEED_UP=50cm/s 제한 내) |
| `TOLERANCE` | 0.3m | 이륙 완료 판정 여유 |
| `MOVE_DIST` | 0.30m | N/E/S/W 이동 거리 |
| `_VEL_ONLY` | 3527 | velocity-only typemask (X/Y/Z position 무시) |
| `_POS_ONLY` | 3576 | position-only typemask (Vx/Vy/Vz 무시) |

ArduPilot QGC 설정:
- `ARMING_CHECK = 0` (EKF compass variance 우회)
- `DISARM_DELAY` 기본값 10s (velocity cmd가 충분한 추력 → 이륙 시 land detector 해제됨)

---

## 할 것
- [x] F-7 해결: velocity setpoint + UWB z 완료 체크
- [ ] 실제 비행 테스트: velocity 이륙 동작 확인
- [ ] N/E/S/W 이동 로직 테스트
