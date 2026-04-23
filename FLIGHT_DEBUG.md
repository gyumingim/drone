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

### F-7. 하이브리드 성공 but DISARM_DELAY 타임아웃 (미해결, z=-5.66m)
**증상 (최신 실행 15:22):**
```
start_z = -5.66m   (역대 최대 baro drift)
nav_alt = 11.7m    → 모터 스핀업 성공 (SERVO=1100)
z 변화: -5.66 → -6.09m (+0.43m 실제 상승)
threshold: -6.36m  (0.27m 남겨두고 타임아웃)
ARM 후 11초 → TAKEOFF ABORTED: drone disarmed
```
**원인:**
1. EKF: "드론이 지상에서 5.66m 위" → position controller는 "1m만 더 올리면 됨"으로 계산
2. → SERVO 최소값(1100)만 인가 → 실제 추력 부족
3. Land detector: "아직 지상" 유지
4. `DISARM_DELAY` 기본값 10초 만료 → auto-disarm

**근본 원인:** 매 실행 시 baro drift 크기가 다름 (-1.3m, +2.05m, -3.17m, **-5.66m**).  
FC 부팅 시 홈 설정 후 ARM 전까지 baro가 계속 드리프트.  
현재 코드는 방향은 핸들링하나, drift가 5m 이상이면 position controller 출력이 너무 작아 DISARM_DELAY 내 이륙 불가.

**수정 후보 (미적용):**
| 방법 | 장점 | 단점 |
|---|---|---|
| ARM 직전 홈 재설정 (`MAV_CMD_DO_SET_HOME`) | z≈0 보장, 근본 해결 | FC 응답 확인 필요 |
| `DISARM_DELAY` 파라미터 증가 (QGC) | 즉시 적용 가능 | 실패 시 오래 켜짐 |
| velocity setpoint 병행 (`vz` 지정) | 상승 속도 보장 | position + velocity 혼용 복잡성 |

---

## 현재 이륙 로직 (하이브리드)

```
ARM
└─ takeoff()
     ├─ LOCAL_POSITION_NED 읽기 → start_z, hold_x, hold_y
     ├─ nav_alt = abs(start_z) + alt + 5.0
     ├─ MAV_CMD_NAV_TAKEOFF(nav_alt) 전송   ← land detector 우회
     └─ 루프
          ├─ SERVO > 1100 → motors_on = True
          ├─ motors_on이면 send_position(hold_x, hold_y, target_z) 전송
          ├─ HEARTBEAT armed=False → abort
          └─ z < threshold → 이륙 완료, (x, y, z) 반환
```

## 코드 파라미터
| 변수 | 값 | 역할 |
|---|---|---|
| `TAKEOFF_ALT` | 1.0m | 목표 호버링 고도 |
| `TOLERANCE` | 0.3m | 이륙 완료 판정 여유 |
| `MOVE_DIST` | 0.30m | N/E/S/W 이동 거리 |

ArduPilot QGC 설정:
- `ARMING_CHECK = 0` (EKF compass variance 우회)
- `DISARM_DELAY` 기본값 10s → **F-7 원인, 증가 필요**

---

## 할 것
- [ ] F-7 해결: ARM 직전 홈 재설정 또는 DISARM_DELAY 증가
- [ ] N/E/S/W 이동 로직 실제 비행 테스트
