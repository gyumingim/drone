# SITL 파라미터 설정 가이드

## 실행 순서

```bash
# 터미널 1: SITL 시작 (--out으로 14551 포트 자동 추가)
python3 ~/ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter --console --out=127.0.0.1:14551

# 터미널 2: 스크립트
cd ~/drone && python3 sitl_flight_tag.py
```

> `--out=127.0.0.1:14551` 없이 SITL을 이미 실행한 경우 MAVProxy 콘솔에서:
> ```
> output add 127.0.0.1:14551
> ```

## MAVProxy 콘솔 파라미터 (매 SITL 재시작마다 입력)

```
param set GPS_TYPE 0
param set ARMING_CHECK 0
param set VISO_TYPE 1
param set EK3_SRC1_POSXY 6
param set EK3_SRC1_YAW 6
param set EK3_SRC1_POSZ 1
param set RNGFND1_TYPE 0
reboot
```

> reboot 후 MAVProxy가 재연결되면 스크립트 실행

## 파라미터 설명

| 파라미터 | SITL 값 | 실기체 값 | 이유 |
|---|---|---|---|
| `GPS_TYPE` | **0** | 1 | SITL GPS 시뮬레이터 비활성화 — GPS prearm check 3개 제거 |
| `ARMING_CHECK` | **0** | 0 | 모든 prearm check 비활성화 |
| `VISO_TYPE` | 1 | 1 | MAVLink VISION_POSITION_ESTIMATE 수신 활성화 |
| `EK3_SRC1_POSXY` | 6 | 6 | XY 소스 = ExternalNav (VPE) |
| `EK3_SRC1_YAW` | 6 | 6 | Yaw 소스 = ExternalNav (VPE) |
| `EK3_SRC1_POSZ` | **1** | 2 | Z 소스: SITL=Baro / 실기체=Rangefinder(D435i) |
| `RNGFND1_TYPE` | **0** | 10 | SITL=Rangefinder 비활성화 / 실기체=MAVLink |
| `RNGFND1_MAX_CM` | - | 1000 | D435i 최대 거리 10m |
| `RNGFND1_MIN_CM` | - | 10 | 최소 감지 거리 10cm |
| `VISO_DELAY_MS` | - | 82 | VPE 전송 지연 보정 (카메라 실측값) |
| `VISO_YAW_M_NSE` | - | 0.05 | yaw 측정 노이즈 (기본 0.2 → 0.05) |

## 실기체 전용 파라미터 (SITL에서는 불필요)

```
param set VISO_DELAY_MS 82
param set VISO_YAW_M_NSE 0.05
param set RNGFND1_TYPE 10
param set RNGFND1_MAX_CM 1000
param set RNGFND1_MIN_CM 10
param set EK3_SRC1_POSZ 2
```
