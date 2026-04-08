# UWB Anchor Dropper SITL — 개발 가이드

## 존재 이유 (Why This Exists)

로버가 미지의 환경을 탐사할 때 **UWB 측위 인프라**를 실시간으로 구축해야 한다.  
앵커를 사람이 직접 설치하면 느리고 위험하므로, 드론이 로버를 따라다니며 자동으로 앵커를 투하한다.  
이 파일은 그 전체 임무를 **계획 → 시뮬레이션 → 실기체 실행**까지 하나의 GUI로 통합한 SITL 도구다.

---

## 전체 프로세스 흐름

```
┌─────────────────────────────────────────────────────────────┐
│                        실행 환경                             │
│                                                             │
│  터미널 A                    터미널 B                        │
│  ~/drone/run_sitl_cam.sh     python3 uwb_anchor_dropper_sitl.py │
│        │                           │                        │
│        ▼                           ▼                        │
│  PX4 SITL 바이너리           tkinter GUI                    │
│  + Gazebo Harmonic               │                          │
│  (x500_cam 모델)                 │                          │
│        │                          │                          │
│        │ MAVLink UDP              │ gz CLI subprocess        │
│        │ udpin://0.0.0.0:14540    │ (앵커 spawn/despawn)    │
│        │                          │                          │
│        └──────── MAVSDK ──────────┘                         │
│                                                             │
│  gz-transport ──→ /drone/downward_cam/image ──→ camera_detector.py │
└─────────────────────────────────────────────────────────────┘
```

### 미션 실행 흐름

```
사용자 드래그 (캔버스)
    │
    ▼
smart_place_anchors()  — 지그재그 후보점 생성
    │
    ▼
plan_mission()         — 디포 픽업 / 재배치 / 건너뜀 스텝 계획
    │
    ▼
[미션 시작 버튼]
    │
    ├─ HAS_MAVSDK=False ──→ _run_dummy_mission()  (tkinter 애니메이션)
    │
    └─ HAS_MAVSDK=True
         │
         ├─ MODE=SITL ──→ _mission_coro(udpin://0.0.0.0:14540)
         └─ MODE=REAL ──→ _mission_coro(serial:///dev/ttyACM0:57600)
                │
                ├─ arm → offboard → 이륙
                ├─ 스텝별: 픽업(_real_fly_pick) → 투하(_real_fly_drop)
                │          └─ 카메라 정렬(_cam_align) 포함
                └─ 홈 복귀 → land
```

### 카메라 정렬 흐름 (REAL/SITL 픽업 시)

```
_real_fly_pick()
    │
    ├─ 목표 위치로 비행
    ├─ 중간 고도(flight_alt/2)에서 정지
    │
    ▼
_cam_align()  — 최대 8초
    │
    ├─ camera.get_frame()
    ├─ detect_anchor()  — HSV 빨간색 마스크 → 컨투어 → 중심점
    ├─ pixel_to_ned_offset()  — 픽셀 오프셋 → NED 미터 변환
    └─ 오프셋 < 0.15m 이면 정렬 완료, 아니면 보정 이동 반복
    │
    ▼
정렬된 위치로 하강 → 그리퍼 닫기
```

---

## 최종 목표 (End Goal)

```
[로버] ──── UWB앵커 디포 적재 ────→ 경로 끝점
  ↑
드론이 앵커를 집어서 지그재그로 설치
  ↑
기존 앵커가 로버 뒤에 남으면 회수 → 앞쪽으로 재배치
```

실제 PX4 드론 + Gazebo Harmonic 환경에서 **완전 자율 UWB 앵커 배치 임무**를 수행하는 것.

---

## 기술 스택

| 레이어 | 기술 |
|--------|------|
| GUI | Python `tkinter` |
| 드론 제어 | `mavsdk` (offboard NED + gripper) |
| 비동기 | `asyncio` + `threading` |
| 시뮬레이터 | Gazebo Harmonic (`gz` CLI + gz-transport Python) |
| 카메라 | `camera_detector.py` — gz-transport 또는 V4L2 |
| 좌표계 | NED (North-East-Down), alt 음수 = 위 |
| Python | 시스템 `python3` (Ubuntu 22.04, Python 3.10) |

---

## 설치 및 실행

### 의존성

```bash
# 시스템 패키지 (Ubuntu 22.04)
sudo apt install python3-gz-transport13 python3-gz-msgs10 \
                 python3-opencv python3-pil python3-pil.imagetk

# pip (시스템 python3)
pip3 install mavsdk
pip3 install "protobuf>=5.0,<6"   # mavsdk + gz.msgs10 동시 호환

# conda 사용 금지: conda protobuf가 PX4 C++ 빌드와 충돌함
```

> `PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python` 은 `camera_detector.py` 에서 자동으로 설정된다.

### SITL 실행 (카메라 포함)

```bash
# 1회만 — x500_cam Gazebo 모델 생성
python3 ~/drone/patch_x500_camera.py

# 터미널 A: PX4 + Gazebo (x500_cam 모델)
~/drone/run_sitl_cam.sh

# 터미널 B: GUI
python3 ~/drone/uwb_anchor_dropper_sitl.py
```

### SITL 실행 (카메라 없음, 기본 x500)

```bash
# 터미널 A
cd ~/PX4-Autopilot && make px4_sitl gz_x500

# 터미널 B
python3 ~/drone/uwb_anchor_dropper_sitl.py
```

### 실기체 실행

```bash
# CFG["real_address"] = "serial:///dev/ttyACM0:57600" 확인 후
# uwb_anchor_dropper_sitl.py 상단에서 MODE = "REAL" 로 변경
python3 ~/drone/uwb_anchor_dropper_sitl.py
```

---

## 파일 구조

```
drone/
├── uwb_anchor_dropper_sitl.py   — 메인 GUI + 미션 로직
├── camera_detector.py           — AnchorDetector (gz/V4L2 카메라)
├── patch_x500_camera.py         — PX4 x500_cam Gazebo 모델 생성 (1회 실행)
├── run_sitl_cam.sh              — x500_cam SITL 직접 실행 스크립트
└── UWB_ANCHOR_DROPPER.md        — 이 문서
```

---

## 아키텍처 (코드 구조)

```
uwb_anchor_dropper_sitl.py
│
├── GazeboMonitor          — Gazebo 폴링 / spawn / despawn
├── CoordMapper            — 픽셀 ↔ NED 좌표 변환
├── _zigzag_candidates()   — 지그재그 후보점 생성 (순수 함수)
├── smart_place_anchors()  — 배제 필터링 (순수 함수)
├── _make_proj()           — 경로 투영 클로저 생성 (순수 함수)
├── plan_mission()         — 미션 스텝 계획 (순수 함수)
└── UWBApp                 — tkinter 메인 앱
    ├── _build_ui / _build_panel   — UI 레이아웃
    ├── _draw_map / _draw_*        — 캔버스 렌더링
    ├── _on_press/drag/release     — 마우스 이벤트
    ├── _update_preview            — 슬라이더/드래그 → 미리보기 갱신
    ├── _confirm                   — 미션 시작 버튼
    ├── _run_dummy_mission         — 더미 시뮬레이션 (별도 스레드)
    ├── _mission_coro              — 실 MAVSDK 비동기 코루틴
    ├── _real_fly_pick/drop        — REAL/SITL 비행 헬퍼
    ├── _cam_align                 — 카메라 기반 앵커 위치 정렬
    ├── _open_cam_window           — 독립 카메라 팝업 창
    └── _cam_refresh               — 패널 소형 카메라 뷰 갱신 (15fps)

camera_detector.py
└── AnchorDetector
    ├── _init_gz()              — gz-transport 구독 초기화
    ├── _init_v4l2()            — V4L2 카메라 초기화
    ├── detect_anchor()         — HSV 빨간색 컨투어 감지
    ├── annotate()              — 프레임에 감지 결과 오버레이
    ├── get_tk_image()          — tkinter PhotoImage 변환
    └── pixel_to_ned_offset()   — 픽셀 → NED 오프셋 변환 (정적)
```

---

## 주요 설정값 (CFG)

| 항목 | 기본값 | 설명 |
|------|--------|------|
| `MODE` | `"SITL"` | `"SITL"` 또는 `"REAL"` |
| `CFG["address"]` | `udpin://0.0.0.0:14540` | SITL MAVSDK 주소 |
| `CFG["real_address"]` | `serial:///dev/ttyACM0:57600` | 실기체 주소 |
| `CFG["flight_alt"]` | `-2.5` | 비행 고도 (NED m, 음수=위) |
| `CFG["drop_alt"]` | `-0.4` | 투하/픽업 고도 |
| `CFG["camera_source"]` | `"gz"` | `"gz"` / `"v4l2"` / `"none"` |
| `CFG["camera_fov"]` | `90.0` | 카메라 수평 FOV (도) |
| `CFG["cam_align_tol"]` | `0.15` | 카메라 정렬 허용 오차 (m) |
| `ANCHOR_SPACING` | `3.5m` | 앵커 간격 (슬라이더로 변경) |
| `DEPOT_STOCK_DEFAULT` | `3` | 로버 초기 UWB 보유 수 |
| `COVERAGE_RADIUS` | `30.0m` | 커버리지 원 반경 (시각화용) |

---

## 알려진 함정 (Pitfalls)

### 1. tkinter 스레드 안전성 — **가장 중요**
```python
# 잘못된 예시 (스레드에서 직접 위젯 접근)
self.label.config(text="...")  # → 크래시 가능

# 올바른 방법
self.root.after(0, lambda: self.label.config(text="..."))
```

### 2. protobuf 버전 충돌
- pip protobuf ≥ 4.x는 `gz.msgs10` Python 코드(`/usr/lib/python3/dist-packages/gz/msgs10/`)와 충돌
- `PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python` 으로 우회 (`camera_detector.py` 최상단에서 자동 설정)
- conda 환경 사용 시 conda protobuf 헤더가 PX4 C++ 빌드에 섞여서 빌드 실패 → **conda 사용 금지**

### 3. make px4_sitl gz_x500_cam 빌드 실패
- conda가 활성화된 상태에서 PX4 빌드 시 `-isystem ~/anaconda3/include` 가 추가되어 protobuf 5.x 헤더와 충돌
- 해결: `~/drone/run_sitl_cam.sh` 사용 (기존 바이너리 직접 실행)

### 4. CoordMapper 고정 크기 의존
- 캔버스에 `fill/expand` 주면 좌표 어긋남 → 항상 `pack()` (fill/expand 없이) 유지

### 5. SDF 이스케이핑
- 앵커 이름에 특수문자 금지, `uwb_anchor_<int>` 형식만 사용

### 6. 창 종료 시 미션 스레드 타이밍
- `drone.connect()` 대기 중에 창을 닫으면 스레드가 즉시 종료되지 않을 수 있음
- `_on_close()` → `_stop_evt.set()` → 다음 `await` 체크 시 종료됨

---

## TODO

- [ ] 경로 여러 개 지원 (현재 단일 경로만)
- [ ] 실기체 gripper 피드백 확인 로직
- [ ] ROS 2 토픽으로 로버 실제 위치 수신 연동
- [ ] Gazebo spawn 실패 시 재시도 또는 UI 경고
- [ ] `reloc_new_positions` 부동소수점 집합 비교 → epsilon 비교로 개선
