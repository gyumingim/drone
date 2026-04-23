# ArduPilot QGC Parameter Setup
Pixhawk 6C Mini + GrowSpace UWB Listener

---

## 필수 파라미터

| 파라미터 | 값 | 설명 |
|---|---|---|
| `FRAME_CLASS` | `1` | Quadrotor |
| `FRAME_TYPE` | `1` | X frame |
| `BRD_SAFETYENABLE` | `0` | 하드웨어 안전 스위치 비활성화 |
| `BRD_SAFETY_DEFLT` | `0` | 부팅 시 안전 스위치 기본값 OFF |
| `GPS_TYPE` | `0` | GPS 비활성화 (실내 UWB 사용) |
| `ARMING_CHECK` | `0` | 모든 pre-arm 체크 비활성화 (테스트용) |

---

## EKF3 위치 소스 (UWB Vision)

| 파라미터 | 값 | 설명 |
|---|---|---|
| `EK3_SRC1_POSXY` | `6` | 수평 위치 = ExternalNav (UWB) |
| `EK3_SRC1_POSZ` | `6` | 수직 위치 = ExternalNav (UWB) |
| `EK3_SRC1_VELXY` | `0` | 수평 속도 = None |
| `EK3_SRC1_VELZ` | `0` | 수직 속도 = None |
| `EK3_SRC1_YAW` | `1` | Yaw = Compass |
| `VISO_TYPE` | `1` | Visual Odometry 활성화 (VISION 메시지 수신) |

---

## 비행 안전 (실내 테스트)

| 파라미터 | 값 | 설명 |
|---|---|---|
| `FS_GCS_ENABLE` | `0` | GCS 연결 끊겨도 RTL 하지 않음 |
| `PILOT_SPEED_UP` | `50` | 최대 상승 속도 50cm/s (기본 250) |
| `RTL_ALT` | `200` | RTL 고도 2m (기본 1500cm=15m) |

---

## 모터 출력 배치 (X frame 기준)

```
        Front
   3(CCW)  1(CW)
       X
   2(CW)   4(CCW)
        Rear
```

| ArduPilot 모터 | 위치 | 회전 | Pixhawk 출력 |
|---|---|---|---|
| Motor 1 (A) | 앞오른쪽 | CW  | MAIN OUT 1 |
| Motor 2 (B) | 뒤왼쪽   | CW  | MAIN OUT 2 |
| Motor 3 (C) | 앞왼쪽   | CCW | MAIN OUT 3 |
| Motor 4 (D) | 뒤오른쪽 | CCW | MAIN OUT 4 |

---

## 파라미터 적용 순서

1. QGC Parameters 탭에서 위 값 모두 입력
2. **Reboot Vehicle** (EK3, BRD 파라미터는 재부팅 필요)
3. Motors 탭 → 토글 켜고 A/B/C/D 각각 테스트로 배선 확인
4. 이상 없으면 `python3 manual_control.py` 실행
