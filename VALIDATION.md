# 코드 타당성 검증 보고서

검증일: 2026-05-04  
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
| Z covariance | **0.25 (9999 금지)** | `cov[11]=0.25` ✅ |
| EK3_SRC1_POSXY | ExternalNav(6) | 6 ✅ |
| EK3_SRC1_POSZ | Baro(1) | 1 ✅ |
| EK3_SRC1_YAW | ExternalNav(6) | 6 ✅ |

### ⚠️ cov[11]=9999 금지 — ArduPilot 소스 코드 근거

`GCS_Common.cpp::handle_common_vision_position_estimate_data` (SHA: 3797eb7f):

```cpp
if (!isnan(covariance[0])) {
    posErr = sqrtf(covariance[0]+covariance[6]+covariance[11]);  // x+y+z 합산
    angErr = sqrtf(covariance[15]+covariance[18]+covariance[20]);
}
visual_odom->handle_pose_estimate(usec, timestamp_ms, x, y, z, roll, pitch, yaw, posErr, angErr, reset_counter, 0);
```

- `posErr`는 x/y/z covariance를 **합산**해서 단일 스칼라로 EKF에 전달
- `cov[11]=9999` 설정 시: `posErr = sqrt(0.25 + 0.25 + 9999) ≈ 100m`
- EKF는 이 posErr을 XY 신뢰도로도 사용 → **XY까지 사실상 무시됨**
- EK3_SRC1_POSZ=Baro라도 cov[11] 값은 posErr 합산에 영향을 줌
- 따라서 z를 "무시"하기 위해 cov[11]=9999를 쓰면 XY 수용이 함께 망가지는 버그

**올바른 설정:** `cov[11] = (UWB z 오차)² = 0.5² = 0.25`

**참고:**
- https://github.com/ArduPilot/ardupilot/blob/master/libraries/GCS_MAVLink/GCS_Common.cpp — 실제 posErr 계산 코드
- https://ardupilot.org/dev/docs/mavlink-nongps-position-estimation.html — 4Hz 이상 권장
- https://ardupilot.org/copter/docs/common-ekf-sources.html — EK3_SRC1_* 파라미터 설명

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
| Z covariance=0.25 (9999 금지) | ✅ ArduPilot GCS_Common.cpp posErr 합산 구조 확인 |
| RC override ch3=1000, 나머지 0 | ✅ MAVLink 공식 정의와 일치 |
| EKF3 파라미터 (POSXY=6, POSZ=1, YAW=6) | ✅ ArduPilot 공식 indoor non-GPS 권장 |
| scipy 삼변측량 (trf + z≥0 bound) | ✅ 수학적으로 타당, 표준 구현 |

**현재 코드는 공식 문서 기준 모두 타당함.**
