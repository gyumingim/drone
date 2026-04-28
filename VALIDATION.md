# 코드 타당성 검증 보고서

검증일: 2026-04-28  
검증 방법: 공식 문서 + GitHub + 포럼 (웹 검색)

---

## 1. pymavlink 단일 리더 스레드

**검증 결과: ✅**

- pymavlink `mavlink_connection`은 **thread-safe하지 않음** (공식 입장)
- 권장 패턴: 단일 스레드만 `recv_match()` 호출, 나머지는 lock 보호 cache 사용
- 현재 코드: `_reader_loop`만 `recv_match(blocking=True, timeout=0.1)` 호출 → 정확

**참고:**
- https://groups.google.com/g/mavlink/c/ZnJzT2CMN8c — Google Groups pymavlink 멀티스레드 논의, 단일 리더 스레드 + queue 패턴 공식 권장
- https://mavlink.io/en/mavgen_python/ — pymavlink 공식 가이드

---

## 2. VISION_POSITION_ESTIMATE + EKF3

**검증 결과: ✅**

| 항목 | 공식 기준 | 현재 코드 |
|------|-----------|-----------|
| 최소 주파수 | 4Hz 이상 | 20Hz ✅ |
| XY covariance | 낮게 설정 | `cov[0]=cov[6]=0.01` ✅ |
| Z covariance | 높게 설정 (baro 사용 시) | `cov[11]=9999.0` ✅ |
| EK3_SRC1_POSXY | ExternalNav(6) | 6 ✅ |
| EK3_SRC1_POSZ | Baro(1) | 1 ✅ |
| EK3_SRC1_YAW | ExternalNav(6) | 6 ✅ |

**참고:**
- https://ardupilot.org/dev/docs/mavlink-nongps-position-estimation.html — 4Hz 이상 권장, Z covariance 높게 설정 패턴 명시
- https://ardupilot.org/copter/docs/common-ekf-sources.html — EK3_SRC1_* 파라미터 설명
- https://ardupilot.org/copter/docs/common-optitrack.html — 동일한 ExternalNav 설정 구조 (Optitrack 예제)

---

## 3. RC_CHANNELS_OVERRIDE

**검증 결과: ✅**

- ch값 `0` = "release to RC radio" — MAVLink 공식 정의
- ch값 `65535(UINT16_MAX)` = "ignore" (release와 다름)
- 현재 코드: `ch3=1000`, 나머지 `0` → ch3만 override, 물리 RC 나머지 채널 통과
- 5Hz 전송 → ArduPilot 최소 요구 2Hz 초과 ✅

**참고:**
- https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE — `0` = release 공식 정의
- https://ardupilot.org/dev/docs/mavlink-rcinput.html

---

## 4. UWB 삼변측량 (scipy.least_squares)

**검증 결과: ✅**

- `method='trf'` (Trust Region Reflective): 경계조건(z≥0) 처리에 최적 알고리즘
- `bounds=([-inf,-inf,0], [inf,inf,inf])`: z≥0 강제 (드론은 바닥 위)
- 잔차 함수 `norm(pts - p) - dst`: 거리 오차 최소화 표준 공식
- DWM1001 lec 포맷 파싱 정확

**참고:**
- https://forum.qorvo.com/t/lec-command-output-in-uart-shell-mdek1001/5075 — DWM1001 lec UART 포맷 공식 확인
- https://github.com/madfolio/Least-Squares-Trilateration — scipy 삼변측량 패턴

---

## 종합

| 항목 | 결과 |
|------|------|
| pymavlink 단일 리더 스레드 | ✅ 공식 권장 패턴과 일치 |
| VISION_POSITION_ESTIMATE 20Hz | ✅ 공식 최소(4Hz) 이상 |
| Z covariance=9999 | ✅ baro Z 신뢰 시 공식 패턴 |
| RC override ch3=1000, 나머지 0 | ✅ MAVLink 공식 정의와 일치 |
| EKF3 파라미터 (POSXY=6, POSZ=1, YAW=6) | ✅ ArduPilot 공식 indoor non-GPS 권장 |
| scipy 삼변측량 (trf + z≥0 bound) | ✅ 수학적으로 타당, 표준 구현 |

**현재 코드는 공식 문서 기준 모두 타당함.**
