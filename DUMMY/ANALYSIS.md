# manual_control.py 분석

## 전체 목표
ArduPilot + GrowSpace UWB 기반 실내 자율비행 시스템 구축

## 세부 목표
- UWB 위치 추정값을 ArduPilot EKF에 주입 (VISION_POSITION_ESTIMATE)
- GUIDED 모드에서 takeoff → hover → land 자동 비행
- matplotlib 실시간 시각화 (태그 위치, FC 텔레메트리, 비행 로그)

## 사용 기술
- pymavlink: FC 통신, MAVLink 메시지 송수신
- UWBTag: GrowSpace UWB 리스너에서 태그 위치 수신
- threading: 비행 스레드 / 시각화(메인) 스레드 분리
- matplotlib FuncAnimation: 200ms 주기 실시간 갱신

---

## Yaw 결정 방식

**현재 Yaw를 완전히 무시하고 있음.**

- `_POS_ONLY` 타입마스크 (L28-37): `YAW_IGNORE`, `YAW_RATE_IGNORE` 포함
- `send_position()` (L208-218): yaw/yaw_rate 파라미터로 `0, 0` 전달하지만 타입마스크로 FC가 읽지 않음
- `takeoff()` (L279): yaw에 `float('nan')` 전달 → ArduPilot 기본 동작
- 결과: 드론은 arm 시점의 yaw를 유지하거나 ArduPilot 내부 heading 제어를 따름

---

## 문제점

### 심각도 높음
1. **ned_x/ned_y None 처리 오류 (L499-500)**
   - `ned_x = t.get('ned_x') or 0.0` — 값이 `0.0`이면 `False`로 평가되어 `0.0`으로 대체됨
   - 수정: `ned_x = t.get('ned_x') if t.get('ned_x') is not None else 0.0`

2. **goto 3D 거리 판정 (L233-237)**
   - 수직/수평 허용오차가 동일 (`TOLERANCE=0.3m`)
   - 고도 미달인데 수평이 가까워서 도달 판정 가능
   - 수정: 수평 dist와 수직 dist 분리 체크

### 심각도 중간
3. **goto 메시지 처리 비효율 (L226-251)**
   - `blocking=False`로 한 턴에 메시지 1개 처리
   - 같은 루프에서 `LOCAL_POSITION_NED`와 `SERVO_OUTPUT_RAW` 동시 수신 불가
   - 수정: 내부 while로 큐 드레인 또는 별도 메시지 수신 스레드

4. **Yaw 미제어**
   - 실내에서는 문제없을 수 있으나 방향 제어 필요 시 대응 불가
   - 수정: `_POS_ONLY`에서 `YAW_IGNORE` 제거하고 원하는 yaw 전달

### 심각도 낮음
5. **_flight_log O(n) 삭제 (L52-53)**
   - `del _flight_log[0]` → `collections.deque(maxlen=_MAX_LOG)` 교체
6. **_update 함수 120줄 (L575-693)**
   - 4개 패널을 1개 함수에서 처리
   - `_draw_log()` 함수 분리로 `_draw_fc_panel()`과 대칭 구조 만들기

---

## 리팩토링 포인트

| 우선순위 | 대상 | 방법 |
|---------|------|------|
| 1 | `ned_x/ned_y` None 체크 | `is not None` 명시적 비교 |
| 2 | `_flight_log` | `deque(maxlen=_MAX_LOG)` 교체 |
| 3 | `_update` 분할 | `_draw_log()` 함수 분리 |
| 4 | goto 거리 판정 | 수평/수직 분리 |
| 5 | Yaw 제어 | 타입마스크에서 `YAW_IGNORE` 제거, yaw 파라미터 추가 |

---

## 한 것 (Done)
- [x] manual_control.py 전체 구조 분석
- [x] Yaw 결정 방식 파악
- [x] 문제점 목록화
- [x] 리팩토링 포인트 도출

## 하고 있는 것 (In Progress)
- [ ] 없음 (분석 단계)

## 할 것 (Todo)
- [ ] `ned_x/ned_y` None 처리 버그 수정
- [ ] `_flight_log`를 `deque`로 교체
- [ ] `_draw_log()` 함수 분리
- [ ] goto 거리 판정 수직/수평 분리
- [ ] Yaw 제어 추가 여부 결정 (사용 시나리오 확인 필요)
