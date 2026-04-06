# UWB Anchor Dropper SITL — 개발 가이드

## 존재 이유 (Why This Exists)

로버가 미지의 환경을 탐사할 때 **UWB 측위 인프라**를 실시간으로 구축해야 한다.  
앵커를 사람이 직접 설치하면 느리고 위험하므로, 드론이 로버를 따라다니며 자동으로 앵커를 투하한다.  
이 파일은 그 전체 임무를 **계획 → 시뮬레이션 → 실기체 실행**까지 하나의 GUI로 통합한 SITL 도구다.

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
| 시뮬레이터 | Gazebo Harmonic (`gz` CLI subprocess) |
| 좌표계 | NED (North-East-Down), alt 음수 = 위 |
| 실행 | `python3 uwb_anchor_dropper_sitl.py` |

**의존성:**
```
pip install mavsdk
# Gazebo Harmonic: gz 바이너리가 PATH에 있어야 함
```

mavsdk가 없으면 자동으로 더미 시뮬레이션 모드로 동작한다.

---

## 주요 기능 (Features)

### 1. 인터랙티브 맵
- 캔버스에 드래그 → 경로 시작점/끝점 지정
- 실시간 앵커 배치 미리보기 (지그재그 패턴)
- 드론·로버 위치 및 이동 궤적 시각화

### 2. 지그재그 앵커 배치 알고리즘 (`smart_place_anchors`)
- 경로를 따라 `spacing`(m) 간격으로 좌우 교대 배치
- 기존 앵커 `min_dist` 이내면 배제
- 신규 앵커끼리는 `spacing × 0.7` 이내면 배제

### 3. 로버 디포 관리
- 로버가 초기 UWB 재고 N개를 싣고 출발
- 재고 있으면 → **디포 픽업** (로버 위치로 날아가서 집기)
- 재고 소진 → **스마트 재배치** 모드 자동 전환

### 4. 스마트 재배치 알고리즘 (`plan_mission`)
재배치 후보 조건:
1. 앵커의 경로 투영값 < 현재 로버 투영값 (= 로버 뒤에 있음)
2. 기존 앵커 + 이미 설치한 신규 앵커 모두 후보
3. 끝점(endpoint)과의 거리가 **가장 먼 것** 선택 → 공백 최소화

### 5. Gazebo 연동
- `GazeboMonitor`: 2초마다 `gz model --list` 폴링
- 앵커 설치 시 `gz service /world/.../create` 로 SDF 모델 스폰
- 앵커 회수 시 `gz service /world/.../remove` 로 디스폰
- "미확인 앵커 스폰" 버튼: 기존 앵커가 Gazebo에 없으면 강제 스폰

### 6. 실기체 MAVSDK 모드
- `drone.offboard.set_position_ned()` 로 NED 좌표 이동
- `drone.gripper.grab/release()` 또는 actuator fallback
- 텔레메트리 비동기 스트림으로 실시간 위치 표시

### 7. 더미 시뮬레이션 모드
- mavsdk 없이도 동작
- `time.sleep` 기반 애니메이션으로 임무 흐름 시각화

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
    └── _mission_coro              — 실 MAVSDK 비동기 코루틴
```

---

## 조심해야 할 버그 & 함정

### 1. tkinter 스레드 안전성 — **가장 중요**
```python
# 잘못된 예시 (스레드에서 직접 위젯 접근)
self.label.config(text="...")  # → 크래시 가능

# 올바른 방법
self.root.after(0, lambda: self.label.config(text="..."))
```
`_run_dummy_mission`과 `GazeboMonitor._loop`은 별도 스레드에서 돌기 때문에 반드시 `root.after(0, ...)` 로만 UI 변경해야 한다.

### 2. CoordMapper 고정 크기 의존
`CoordMapper`는 `CW=720, CH=720`으로 **초기화 시 고정**된다.  
캔버스에 `fill/expand`를 주면 실제 픽셀 크기가 달라져서 좌표가 어긋난다.  
→ 캔버스는 항상 `pack()` (fill/expand 없이) 유지.

### 3. plan_mission 재실행 타이밍
`_update_preview`에서 미리 계획을 세우고, `_confirm`에서 **다시** 계획을 세운다.  
슬라이더를 바꾼 뒤 confirm을 누르면 preview와 실제 스텝이 일치하지만,  
슬라이더 콜백(`_slider_changed`)이 `depot_remaining`을 리셋하는 것에 주의.

### 4. SDF 문자열 이스케이핑 취약
```python
sdf = '<?xml ... name=\\"...\\">'
```
현재 `\\"` 방식으로 이스케이프하는데, 앵커 이름에 특수문자가 들어가면 깨진다.  
→ 앵커 이름은 `uwb_anchor_<int>` 형식만 사용할 것.

### 5. `reloc_new_positions` 부동소수점 비교
```python
reloc_pos_set = set(self.reloc_new_positions)
if (n, e) in reloc_pos_set:
```
좌표가 `round(..., 2)`로 생성되므로 대부분 안전하지만,  
직접 부동소수점 연산으로 생성된 좌표는 집합 조회에서 miss할 수 있다.

### 6. asyncio + threading 혼용
`_run_mission`은 별도 스레드에서 `asyncio.run()`을 호출한다.  
미션 도중 창을 닫으면 `pos_task.cancel()`이 실행되지만,  
`_mission_coro`가 `await asyncio.sleep()` 중이 아닌 `await drone.connect()` 단계면 취소가 늦어질 수 있다.  
→ 창 종료 시 무한 대기 가능성 있음. `root.protocol("WM_DELETE_WINDOW", ...)` 처리 고려.

### 7. Gazebo subprocess fire-and-forget
`GazeboMonitor.spawn/despawn`의 실패(timeout, returncode 비정상)가 조용히 무시된다.  
스폰 실패를 GUI에 알리지 않으므로, Gazebo가 다운된 상태에서 미션이 진행될 수 있다.

---

## 깔끔하게 코드 짜는 법 (이 코드베이스 스타일)

**순수 함수 분리**: 계획 로직(`plan_mission`, `smart_place_anchors`)은 UI 상태와 완전히 분리되어 있다. 새 알고리즘 추가 시 동일하게 순수 함수로 작성.

**색상 팔레트 중앙화**: 하드코딩 금지, 반드시 `C["key"]` 사용.

**로그는 `_log()`만**: 스레드 안전 처리가 내장되어 있음. `print()` 대신 `_log()` 사용.

**슬라이더 추가 패턴**: `_slider()` 헬퍼 함수를 재사용하면 된다.

**좌표 변환**: 모든 픽셀 ↔ 세계 좌표 변환은 `self.mapper.w2c/c2w()` 를 통해서만.

---

## 설정값 요약 (CFG / 전역 상수)

| 상수 | 기본값 | 설명 |
|------|--------|------|
| `CFG["address"]` | `udp://:14540` | MAVSDK 접속 주소 |
| `CFG["flight_alt"]` | `-2.5` | 비행 고도 (NED, 음수=위) |
| `CFG["drop_alt"]` | `-0.4` | 투하 고도 |
| `CFG["move_wait"]` | `4.0s` | 이동 후 대기 |
| `ANCHOR_SPACING` | `3.5m` | 앵커 간격 (슬라이더로 변경 가능) |
| `ANCHOR_WIDTH` | `3.0m` | 좌우 폭 |
| `MIN_DIST_FROM_EXISTING` | `8.0m` | 기존 앵커 배제 반경 |
| `DEPOT_STOCK_DEFAULT` | `3` | 로버 초기 UWB 보유 수 |
| `COVERAGE_RADIUS` | `30.0m` | 커버리지 원 반경 (시각화용) |
| `MAP_N / MAP_E` | `(-2, 20)` | 맵 표시 범위 (미터) |

---

## 앞으로 만들어야 할 것 (TODO)

- [ ] 창 종료 시 미션 스레드 안전 종료 처리
- [ ] Gazebo spawn 실패 시 재시도 또는 UI 경고
- [ ] 앵커 ID 충돌 방지 (재배치 후 재스폰 시 id 중복 가능)
- [ ] 경로 여러 개 지원 (현재 단일 경로만 가능)
- [ ] 실기체에서의 gripper 피드백 확인 로직
- [ ] ROS 2 토픽으로 로버 실제 위치 수신 연동
