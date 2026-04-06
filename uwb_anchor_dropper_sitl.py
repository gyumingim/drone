"""
╔══════════════════════════════════════════════════════════════════════════════╗
║              UWB 앵커 GUI 설치 도구 v3  —  개발 문서                        ║
╠══════════════════════════════════════════════════════════════════════════════╣
║                                                                              ║
║  🎯 목표 (GOAL)                                                              ║
║  ─────────────────────────────────────────────────────────────────────────  ║
║  드론이 자율 비행으로 UWB(초광대역) 앵커를 현장에 설치하는 전 과정을        ║
║  tkinter 맵으로 시각화한다.                                                 ║
║                                                                              ║
║  핵심 기능:                                                                  ║
║   1) 마우스 드래그 → 지그재그 앵커 경로 미리보기                            ║
║   2) 드론 실시간 위치를 맵에 표시 (MAVSDK 텔레메트리 or 시뮬)               ║
║   3) Gazebo 모델과 2초 폴링 동기화                                           ║
║   4) 앵커 상태: 미확인(점선) / Gazebo확인(실선) / 설치예정(초록) / 설치중   ║
║   5) 기존 앵커 + 신규 앵커 모두 맵에 표시                                   ║
║                                                                              ║
║  🏗  만들 것 / 유지할 것 (FEATURES TO KEEP)                                  ║
║  ─────────────────────────────────────────────────────────────────────────  ║
║   [ ] 드론 아이콘이 실시간으로 맵 위를 이동                                 ║
║   [ ] EXISTING_ANCHORS 는 Gazebo 연결 여부와 무관하게 항상 맵에 표시        ║
║   [ ] 커버리지 원(30m) 기본값 OFF (눈이 아프므로)                           ║
║   [ ] Gazebo 없는 환경에서도 더미 시뮬로 전 기능 테스트 가능                ║
║   [ ] mavsdk 없어도 tkinter GUI 단독 실행 가능                              ║
║                                                                              ║
║  🐛 과거 버그 / 주의점 (KNOWN BUGS & PITFALLS)                              ║
║  ─────────────────────────────────────────────────────────────────────────  ║
║   ⚠ tkinter 는 메인 스레드 전용 — 모든 canvas/widget 업데이트는             ║
║      root.after(0, fn) 으로 메인 스레드에 위임할 것                         ║
║      (백그라운드 스레드에서 직접 호출하면 세그폴트 or 무반응)               ║
║                                                                              ║
║   ⚠ asyncio.run() 은 스레드 내부에서 호출 — 메인 루프와 별도 이벤트루프    ║
║      사용. 메인 스레드에서 asyncio.run() 하면 tkinter mainloop 와 충돌.     ║
║                                                                              ║
║   ⚠ GazeboMonitor.spawn() 은 static 메서드 — 인스턴스 없이 호출 가능.      ║
║      subprocess 타임아웃을 짧게(2~5s) 유지하지 않으면 GUI 블로킹 위험.      ║
║                                                                              ║
║   ⚠ CoordMapper.m2px() 는 E(동서) 축 스케일 기준 — 맵이 정사각형이 아닐    ║
║      경우 N/S 방향 커버리지 원이 타원이 될 수 있음.                         ║
║                                                                              ║
║   ⚠ 드론 위치 폴링 코루틴은 mission 종료 후에도 계속 돌아야 함.            ║
║      _telemetry_task 가 미션 코루틴에 묶이지 않도록 별도 Task 로 관리.      ║
║                                                                              ║
║   ⚠ 기존 앵커(EXISTING_ANCHORS) 맵 표시는 Gazebo 와 무관하게 항상 그림.    ║
║      Gazebo 확인 여부로 색상/선 스타일만 바꾼다.                            ║
║                                                                              ║
║   🐛 FIX v3.1: create_oval 에 dash= 옵션 사용 불가 (tkinter 버전 의존)      ║
║      → dash 대신 stipple="gray50" + 색상 구분으로 대체.                     ║
║      dash 옵션이 TclError 를 조용히 삼켜 앵커 oval 자체가 안 그려지는 버그.  ║
║                                                                              ║
║   🐛 FIX v3.1: 캔버스 600×600 → 앵커(py≈470) 가 작은 화면에서 잘림.        ║
║      → CW/CH = 480, MAP 범위 조정으로 앵커가 화면 중앙에 오도록 수정.       ║
║                                                                              ║
║  🧠 v3.2 스마트 배치 알고리즘 (SMART PLACEMENT)                             ║
║  ─────────────────────────────────────────────────────────────────────────  ║
║   EXISTING_ANCHORS: 좌표가 이미 알려진 앵커. Gazebo 연결 여부와 무관하게    ║
║   항상 확인됨(solid) 으로 표시. Gazebo 는 시뮬 참고용일 뿐.                 ║
║                                                                              ║
║   smart_place_anchors() 동작:                                                ║
║     1) 드래그 경로 따라 지그재그 후보 좌표 생성                             ║
║     2) 각 후보 → 기존 앵커 전부와 거리 비교                                 ║
║        → MIN_DIST_FROM_EXISTING(슬라이더) 이내면 "blocked"                  ║
║     3) 이미 채택된 신규 앵커와 spacing×0.7 이내면 blocked                   ║
║     4) blocked 위치: 맵에 회색 × 표시 (시각 피드백)                         ║
║     5) 통과 후보만 순서대로 채택 → self.preview 에 저장                     ║
║   ⚠ self.preview_blocked 에 제외 후보 좌표 보관됨.                          ║
║                                                                              ║
║  📐 좌표계 (COORDINATE SYSTEM)                                               ║
║  ─────────────────────────────────────────────────────────────────────────  ║
║   NED (North-East-Down) 사용.  N↑ E→  (D는 고도 — 음수가 위)               ║
║   Gazebo 스폰 시: x=E, y=N, z=0.175 (지상 약 17cm)                         ║
║   MAVSDK offboard: PositionNedYaw(N, E, alt_m, yaw_deg)                     ║
║    alt 는 음수 = 위 (예: -2.5 = 지상 2.5m 상공)                            ║
║                                                                              ║
║  🖥  실행 방법 (HOW TO RUN)                                                  ║
║  ─────────────────────────────────────────────────────────────────────────  ║
║   pip install mavsdk       # 실제 드론 연결 시 (없어도 시뮬 동작)           ║
║   python3 uwb_gui.py                                                        ║
║                                                                              ║
╚══════════════════════════════════════════════════════════════════════════════╝
"""

import tkinter as tk
from tkinter import scrolledtext
import asyncio
import math
import time
import threading
import subprocess

# ─────────────────────────────────────────────────────────────────────────────
# MAVSDK 선택적 임포트
#   mavsdk 가 없으면 HAS_MAVSDK=False 로 설정되어 더미 시뮬레이션 모드로 동작.
#   GUI 자체는 mavsdk 없이도 완전히 실행 가능하다.
# ─────────────────────────────────────────────────────────────────────────────
try:
    from mavsdk import System
    from mavsdk.offboard import OffboardError, PositionNedYaw
    HAS_MAVSDK = True
except ImportError:
    HAS_MAVSDK = False


# ════════════════════════════════════════════════════════════════════════════
#  전역 설정 (CONFIG)
#  ‣ CFG 딕셔너리는 비행 파라미터 모음.  하드코딩 대신 여기서 한 번에 관리.
# ════════════════════════════════════════════════════════════════════════════
MODE = "SITL"   # "SITL" | "REAL"  — 현재 실 비행 로직 분기 없음(예약)

CFG = {
    # MAVSDK 연결 주소 (SITL: udp://:14540, 실기체: serial:///dev/ttyUSB0:57600)
    "address"         : "udp://:14540",

    # 비행 고도 (NED, 음수 = 위).  -2.5 → 지상 2.5m 상공
    "flight_alt"      : -2.5,

    # 앵커 투하 고도.  낮을수록 정확하나 충돌 위험 증가
    "drop_alt"        : -0.4,

    # 각 웨이포인트 도달 후 대기 시간 (초)
    "move_wait"       : 4.0,

    # 이륙 후 안정화 대기 시간 (초)
    "takeoff_wait"    : 5.0,

    # 투하 직후 대기 시간 (그리퍼 열림 확인용)
    "drop_wait"       : 1.5,

    # 그리퍼 액추에이터 채널 번호 (MAVSDK set_actuator 용)
    "gripper_actuator": 1,

    # True 이면 앵커 투하 후 Gazebo 에 원기둥 모델 스폰
    "spawn_markers"   : True,

    # Gazebo 월드 이름 (gz model --list -w <name>)
    "gz_world"        : "default",
}

# ─────────────────────────────────────────────────────────────────────────────
# 맵 표시 범위 (미터, NED 기준)
#   MAP_N = (최솟값, 최댓값) — 화면 위쪽이 N 큰 방향
#   MAP_E = (최솟값, 최댓값) — 화면 오른쪽이 E 큰 방향
# ─────────────────────────────────────────────────────────────────────────────
MAP_N           = (-2.0, 20.0)   # N 범위. 앵커(n=2~15)가 화면 중앙에 오도록 조정
MAP_E           = (-2.0, 20.0)   # E 범위. 앵커(e=2~15)가 화면 중앙에 오도록 조정

COVERAGE_RADIUS = 30.0   # UWB 앵커 이론 커버리지 반경 (m) — 표시 전용
GZ_POLL_SEC     = 2.0    # Gazebo 모델 목록 폴링 간격 (초)
DRONE_POLL_SEC  = 0.2    # 드론 위치 화면 갱신 간격 (초) — 시뮬용

# ─────────────────────────────────────────────────────────────────────────────
# 기존(사전 설치된) UWB 앵커 목록
#   id  : 앵커 고유 번호 (Gazebo 모델명: uwb_anchor_<id>)
#   n,e : NED 좌표 (미터)
# ─────────────────────────────────────────────────────────────────────────────
EXISTING_ANCHORS = [
    {"id": 1, "n":  2.0, "e":  2.0},
    {"id": 2, "n":  2.0, "e": 10.0},
    {"id": 3, "n": 10.0, "e":  5.0},
    {"id": 4, "n": 15.0, "e": 15.0},
]

# 신규 앵커 지그재그 배치 기본값
ANCHOR_SPACING = 3.5   # 앵커 간 경로 방향 간격 (m)
ANCHOR_WIDTH   = 3.0   # 경로 좌우 폭 (m) — 지그재그 진폭

# 기존 앵커(EXISTING_ANCHORS) 또는 이미 채택된 신규 앵커로부터
# 새 앵커가 배치될 수 있는 최소 거리 (m).
# 이 거리 이내는 "blocked"로 표시되고 배치 제외됨.
# 패널 슬라이더로 실시간 조정 가능. 기본값 = 8m.
MIN_DIST_FROM_EXISTING = 8.0

# 드론 이륙 홈 위치 (N, E) — 맵에 "H" 마커로 표시
DRONE_HOME = (0.0, 0.0)


# ════════════════════════════════════════════════════════════════════════════
#  색상 팔레트 (COLOR PALETTE)
#  어두운 배경(#0a0f14) 기반의 사이버틱 UI 테마.
#  여기서 한 번만 바꾸면 전체 UI 에 반영된다.
# ════════════════════════════════════════════════════════════════════════════
C = {
    "bg"          : "#0a0f14",   # 전체 배경
    "panel"       : "#111820",   # 오른쪽 컨트롤 패널 배경
    "grid"        : "#1a2530",   # 맵 그리드 선
    "border"      : "#1e3a4a",   # 패널 테두리, 구분선
    "home"        : "#ffd060",   # 홈(H) 마커 — 노랑
    "text"        : "#c8d8e8",   # 일반 텍스트
    "dim"         : "#3a5060",   # 흐린 텍스트 / 보조 정보
    "accent"      : "#4a9eff",   # 강조 (제목, 슬라이더 값)
    "log_bg"      : "#080d12",   # 로그창 배경

    # 앵커 상태별 색상
    "gz_ok"       : "#e05252",   # ● Gazebo 확인된 기존 앵커 (빨강)
    "gz_missing"  : "#804040",   # ○ 코드에만 있는 기존 앵커 (어두운 빨강 점선)
    "new_anchor"  : "#52e0a0",   # ● 신규 설치 예정 앵커 (초록)
    "installing"  : "#ffd060",   # ↓ 드론이 이동 중인 앵커 (노랑)

    # 커버리지 원 색상 (30m 반경, 반투명 stipple)
    "cov_gz"      : "#e05252",   # Gazebo 확인된 앵커 커버리지
    "cov_missing" : "#804040",   # 미확인 앵커 커버리지
    "cov_new"     : "#52e0a0",   # 신규 앵커 커버리지

    # 경로 표시
    "path_line"   : "#4a9eff",   # 드래그 중심선
    "path_border" : "#2a4060",   # 코리더 경계 점선

    # 버튼 배경
    "btn_confirm" : "#1a6a40",   # 미션 시작 버튼 (초록)
    "btn_spawn"   : "#1a4a6a",   # Gazebo 스폰 버튼 (파랑)
    "btn_clear"   : "#4a2a2a",   # 초기화 버튼 (빨강)

    # 드론 아이콘
    "drone"       : "#00e5ff",   # 드론 현재 위치 (청록)
    "drone_trail" : "#004455",   # 드론 이동 궤적 (어두운 청록)
}


# ════════════════════════════════════════════════════════════════════════════
#  Gazebo 모니터 (GAZEBO MONITOR)
#  ─────────────────────────────────────────────────────────────────────────
#  역할: 백그라운드 스레드에서 주기적으로 Gazebo 모델 목록을 폴링하여
#        GUI 에 콜백으로 전달한다.
#
#  주의: GUI 업데이트는 직접 하지 않고 on_update 콜백만 호출.
#        실제 widget 조작은 UWBApp._on_gz_update → root.after() 경로로.
# ════════════════════════════════════════════════════════════════════════════
class GazeboMonitor:

    def __init__(self, on_update, world="default"):
        """
        Parameters
        ----------
        on_update : callable(set[str])
            Gazebo 모델 이름 집합을 받는 콜백. 백그라운드 스레드에서 호출됨.
        world : str
            Gazebo 월드 이름 (gz model --list -w <world>)
        """
        self._on_update = on_update
        self._world     = world
        self._running   = False
        self.connected  = False   # 마지막 폴링 성공 여부

    def start(self):
        """데몬 스레드로 폴링 루프 시작."""
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def stop(self):
        """폴링 루프 중단 (스레드는 최대 GZ_POLL_SEC 후 자연 종료)."""
        self._running = False

    def _loop(self):
        """[백그라운드 스레드] GZ_POLL_SEC 마다 모델 목록 fetch & 콜백."""
        while self._running:
            models = self._fetch()
            self._on_update(models)
            time.sleep(GZ_POLL_SEC)

    def _fetch(self) -> set:
        """
        'gz model --list' 실행 → 모델 이름 집합 반환.
        실패(Gazebo 미실행, gz 명령어 없음) 시 빈 집합 반환.
        """
        try:
            r = subprocess.run(
                ["gz", "model", "--list", "-w", self._world],
                capture_output=True, text=True, timeout=3
            )
            if r.returncode == 0:
                self.connected = True
                return {ln.strip() for ln in r.stdout.splitlines() if ln.strip()}
            self.connected = False
            return set()
        except Exception:
            self.connected = False
            return set()

    @staticmethod
    def spawn(anchor: dict, world: str = "default"):
        """
        앵커 하나를 Gazebo 에 빨간 원기둥으로 스폰한다.

        Parameters
        ----------
        anchor : dict
            {"id": int, "n": float, "e": float, "name": str}
        world  : str
            Gazebo 월드 이름

        Note
        ----
        ‣ static method 이므로 GazeboMonitor.spawn(a) 처럼 인스턴스 없이 호출 가능.
        ‣ Gazebo 좌표: x=E(동), y=N(북), z=지상 높이(m).
        ‣ timeout=5 로 짧게 설정 — GUI 블로킹 방지.
        """
        n, e  = anchor["n"], anchor["e"]
        name  = anchor.get("name", f"uwb_anchor_{anchor['id']}")
        # SDF: 반지름 12cm, 높이 35cm 원기둥, 빨간 재질
        sdf = (
            '<?xml version=\\"1.0\\"?><sdf version=\\"1.7\\">'
            f'<model name=\\"{name}\\"><static>true</static>'
            '<link name=\\"link\\"><visual name=\\"visual\\">'
            '<geometry><cylinder><radius>0.12</radius><length>0.35</length>'
            '</cylinder></geometry>'
            '<material><ambient>1 0 0 1</ambient>'
            '<diffuse>1 0 0 1</diffuse></material>'
            '</visual></link></model></sdf>'
        )
        req = (f'sdf: "{sdf}" '
               f'pose {{ position {{ x: {e} y: {n} z: 0.175 }} }} '
               f'name: "{name}"')
        try:
            subprocess.run(
                ["gz", "service", "-s", f"/world/{world}/create",
                 "--reqtype", "gz.msgs.EntityFactory",
                 "--reptype", "gz.msgs.Boolean",
                 "--timeout", "2000", "--req", req],
                capture_output=True, timeout=5
            )
        except Exception:
            pass   # Gazebo 없으면 조용히 무시 (GUI 기능에 영향 없음)


# ════════════════════════════════════════════════════════════════════════════
#  좌표 변환기 (COORD MAPPER)
#  ─────────────────────────────────────────────────────────────────────────
#  월드(NED 미터) ↔ 캔버스(픽셀) 상호 변환.
#  맵 범위(MAP_N, MAP_E)와 캔버스 크기(w, h)로 선형 매핑.
# ════════════════════════════════════════════════════════════════════════════
class CoordMapper:

    def __init__(self, w: int, h: int, pad: int = 32):
        """
        Parameters
        ----------
        w, h : int  캔버스 픽셀 크기
        pad  : int  외곽 여백 픽셀 (좌표 라벨 공간)
        """
        self.x0 = pad           # 캔버스 유효 영역 왼쪽 경계 (픽셀)
        self.y0 = pad           # 캔버스 유효 영역 위쪽 경계 (픽셀)
        self.uw = w - pad * 2   # 유효 영역 너비 (픽셀)
        self.uh = h - pad * 2   # 유효 영역 높이 (픽셀)

    def w2c(self, n: float, e: float) -> tuple:
        """
        월드(N, E 미터) → 캔버스(px, py) 변환.
        N 이 클수록 화면 위쪽 (y 작아짐).
        E 이 클수록 화면 오른쪽 (x 커짐).
        """
        px = self.x0 + (e - MAP_E[0]) / (MAP_E[1] - MAP_E[0]) * self.uw
        py = self.y0 + (1 - (n - MAP_N[0]) / (MAP_N[1] - MAP_N[0])) * self.uh
        return px, py

    def c2w(self, px: float, py: float) -> tuple:
        """캔버스(px, py) → 월드(N, E 미터) 역변환."""
        e = MAP_E[0] + (px - self.x0) / self.uw * (MAP_E[1] - MAP_E[0])
        n = MAP_N[0] + (1 - (py - self.y0) / self.uh) * (MAP_N[1] - MAP_N[0])
        return round(n, 2), round(e, 2)

    def m2px(self, meters: float) -> float:
        """
        미터 → 픽셀 변환 (커버리지 원 반경 계산용).
        E 축 스케일 기준이므로 정사각형 맵에서만 정원이 됨.
        """
        return meters / (MAP_E[1] - MAP_E[0]) * self.uw


# ════════════════════════════════════════════════════════════════════════════
#  앵커 배치 알고리즘 (ANCHOR PLACEMENT)
#  ─────────────────────────────────────────────────────────────────────────
#  두 단계로 분리:
#   1) _zigzag_candidates()  — 경로 따라 순수 지그재그 후보 좌표 생성
#   2) smart_place_anchors() — 기존/신규 앵커와 너무 가까운 후보를 제거
#
#  인덱스 0,2,4,… → 왼쪽(side=+1),  1,3,5,… → 오른쪽(side=-1)
#  수직 단위벡터 (pn, pe) = 진행 방향 90° 좌회전
# ════════════════════════════════════════════════════════════════════════════

def _zigzag_candidates(n0: float, e0: float,
                       n1: float, e1: float,
                       spacing: float, width: float) -> list:
    """
    경로 (n0,e0)→(n1,e1) 를 따라 지그재그 후보 좌표를 생성한다.
    필터링 전 원시(raw) 후보 목록.

    Returns
    -------
    list of (n, e) tuples
    """
    total = math.hypot(n1 - n0, e1 - e0)
    if total < 0.1:
        return []

    dn, de = (n1 - n0) / total, (e1 - e0) / total
    pn, pe = -de, dn   # 90° 좌회전 = 왼쪽 방향
    count  = max(1, round(total / spacing))
    result = []
    for i in range(count + 1):
        t  = i / count
        cn = n0 + t * (n1 - n0)
        ce = e0 + t * (e1 - e0)
        side = +1 if i % 2 == 0 else -1
        result.append((round(cn + side * width * pn, 2),
                       round(ce + side * width * pe, 2)))
    return result


def smart_place_anchors(n0: float, e0: float,
                        n1: float, e1: float,
                        spacing: float, width: float,
                        existing: list,
                        min_dist: float) -> tuple:
    """
    지그재그 후보를 생성한 뒤 겹침 최소화 필터를 적용한다.

    필터 규칙
    ─────────
    ① 기존 앵커(existing) 중 하나라도 min_dist 이내 → blocked
    ② 이미 채택된 신규 앵커 중 하나라도 spacing*0.7 이내 → blocked
       (신규 앵커끼리도 너무 붙으면 제외)

    Parameters
    ----------
    n0, e0, n1, e1 : float  드래그 시작/끝 월드 좌표 (m)
    spacing        : float  앵커 간격 (m)
    width          : float  코리더 좌우 폭 (m)
    existing       : list   기존 앵커 dict 목록 [{"n":..,"e":..}, ...]
    min_dist       : float  기존 앵커 배제 반경 (m)

    Returns
    -------
    (accepted, blocked)
      accepted : list[(n,e)]  채택된 신규 앵커 위치
      blocked  : list[(n,e)]  제외된 후보 위치 (맵에 ×로 표시)
    """
    candidates = _zigzag_candidates(n0, e0, n1, e1, spacing, width)
    accepted   = []
    blocked    = []
    new_gap    = spacing * 0.7   # 신규 앵커끼리 최소 간격

    for (cn, ce) in candidates:
        # ① 기존 앵커와 거리 확인
        too_close_existing = any(
            math.hypot(cn - a["n"], ce - a["e"]) < min_dist
            for a in existing
        )
        # ② 이미 채택된 신규 앵커와 거리 확인
        too_close_new = any(
            math.hypot(cn - an, ce - ae) < new_gap
            for (an, ae) in accepted
        )

        if too_close_existing or too_close_new:
            blocked.append((cn, ce))
        else:
            accepted.append((cn, ce))

    return accepted, blocked


# 하위 호환용 별칭 (기존 코드에서 anchors_along_line 을 직접 부르던 곳이 있을 경우 대비)
def anchors_along_line(n0, e0, n1, e1, spacing, width):
    """구버전 호환 — 필터 없는 순수 지그재그 반환."""
    return _zigzag_candidates(n0, e0, n1, e1, spacing, width)


# ════════════════════════════════════════════════════════════════════════════
#  메인 GUI 클래스 (MAIN APPLICATION)
# ════════════════════════════════════════════════════════════════════════════
class UWBApp:
    """
    UWB 앵커 설치 GUI.

    주요 상태 변수
    ──────────────
    line_start / line_end  : 마우스 드래그 시작/끝 픽셀 좌표
    preview                : 현재 미리보기 앵커 위치 목록 [(n,e), ...]
    gz_models              : Gazebo 에 실제 존재하는 모델 이름 집합
    installing             : 드론이 이동 중인 앵커 이름 집합
    installed_new          : 신규 설치 완료 + Gazebo 확인된 앵커 이름 집합
    drone_pos              : 드론 현재 위치 (n, e, alt) or None
    drone_trail            : 드론 이동 궤적 [(n, e), ...] (최대 50점)
    """

    CW = 480   # 캔버스 너비 (픽셀) — 600→480: 작은 화면에서도 전체 창이 보이도록
    CH = 480   # 캔버스 높이 (픽셀)

    TRAIL_MAX = 50   # 드론 궤적 최대 저장 포인트 수

    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("UWB Anchor Installer  v3")
        self.root.configure(bg=C["bg"])
        self.root.resizable(False, False)

        # ── 상태 초기화 ─────────────────────────────────────────────────────
        self.line_start    = None    # 드래그 시작 픽셀 (x, y)
        self.line_end      = None    # 드래그 끝 픽셀 (x, y)
        self.preview       = []      # 신규 앵커 미리보기 [(n, e), ...]
        self.gz_models     = set()   # Gazebo 실제 모델 이름 집합
        self.installing    = set()   # 설치 진행 중인 앵커 이름 집합
        self.installed_new = set()   # 신규 설치 완료 앵커 이름 집합

        # ── 드론 위치 상태 ───────────────────────────────────────────────────
        # drone_pos = (n, e, alt) 미터.  None 이면 위치 미수신 상태.
        self.drone_pos   = None
        # drone_trail: 최근 TRAIL_MAX 개 (n, e) 좌표 — 궤적 표시용
        self.drone_trail = []

        # ── tkinter 슬라이더 변수 ────────────────────────────────────────────
        self.spacing_var  = tk.DoubleVar(value=ANCHOR_SPACING)
        self.width_var    = tk.DoubleVar(value=ANCHOR_WIDTH)
        # 기존 앵커 배제 반경 슬라이더 (m)
        self.min_dist_var = tk.DoubleVar(value=MIN_DIST_FROM_EXISTING)
        # 커버리지 원: 기본 OFF (눈부심 방지)
        self.coverage_var = tk.BooleanVar(value=False)

        # preview_blocked: smart_place_anchors 에서 제외된 후보 위치
        # 맵에 회색 × 로 표시되어 "여기는 기존 앵커 때문에 건너뜀" 을 알림
        self.preview_blocked: list = []

        # ── UI 빌드 ──────────────────────────────────────────────────────────
        self._build_ui()

        # ── Gazebo 폴링 시작 ─────────────────────────────────────────────────
        self.gz_monitor = GazeboMonitor(
            on_update=self._on_gz_update,
            world=CFG["gz_world"]
        )
        self.gz_monitor.start()

    # ══════════════════════════════════════════════════════════════════════
    #  UI 구성 (BUILD UI)
    # ══════════════════════════════════════════════════════════════════════

    def _build_ui(self):
        """최상위 레이아웃: 좌=맵 캔버스, 우=컨트롤 패널."""

        # ── 왼쪽: 맵 영역 ────────────────────────────────────────────────────
        left = tk.Frame(self.root, bg=C["bg"])
        left.pack(side=tk.LEFT, padx=(10, 4), pady=10)

        tk.Label(
            left, text="UWB Anchor Map",
            bg=C["bg"], fg=C["accent"],
            font=("Courier", 13, "bold")
        ).pack(anchor="w")

        # Gazebo 연결 상태 레이블 (폴링마다 갱신됨)
        self.gz_status_lbl = tk.Label(
            left, text="● Gazebo 연결 대기...",
            bg=C["bg"], fg="#606060",
            font=("Courier", 8)
        )
        self.gz_status_lbl.pack(anchor="w", pady=(0, 4))

        # 맵 캔버스
        self.canvas = tk.Canvas(
            left, width=self.CW, height=self.CH,
            bg=C["bg"], highlightthickness=1,
            highlightbackground=C["border"]
        )
        self.canvas.pack()
        self.mapper = CoordMapper(self.CW, self.CH)

        # 마우스 이벤트 바인딩 (경로 드래그)
        self.canvas.bind("<ButtonPress-1>",   self._on_press)
        self.canvas.bind("<B1-Motion>",       self._on_drag)
        self.canvas.bind("<ButtonRelease-1>", self._on_release)

        self._draw_map()

        # ── 오른쪽: 컨트롤 패널 ─────────────────────────────────────────────
        right = tk.Frame(
            self.root, bg=C["panel"], width=215,
            highlightthickness=1, highlightbackground=C["border"]
        )
        right.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10), pady=10)
        right.pack_propagate(False)
        self._build_panel(right)

    def _sep(self, p):
        """패널용 수평 구분선 (1px 높이 frame)."""
        tk.Frame(p, bg=C["border"], height=1).pack(fill=tk.X, padx=8, pady=3)

    def _build_panel(self, p):
        """오른쪽 컨트롤 패널 전체 위젯 구성."""
        pad = {"padx": 12, "pady": 3}

        # ── 제목 ─────────────────────────────────────────────────────────────
        tk.Label(
            p, text="CONTROL", bg=C["panel"],
            fg=C["accent"], font=("Courier", 11, "bold")
        ).pack(pady=(12, 2))
        self._sep(p)

        # ── 드론 위치 표시 ───────────────────────────────────────────────────
        # 실시간으로 (N, E, Alt) 값이 표시됨. 미수신 시 "위치 없음"
        self.drone_pos_lbl = tk.Label(
            p, text="🚁  위치 없음",
            bg=C["panel"], fg=C["drone"],
            font=("Courier", 8), justify=tk.LEFT
        )
        self.drone_pos_lbl.pack(anchor="w", padx=12, pady=(0, 2))
        self._sep(p)

        # ── 앵커 간격 슬라이더 ───────────────────────────────────────────────
        tk.Label(
            p, text="앵커 간격 (m)",
            bg=C["panel"], fg=C["text"],
            font=("Courier", 9)
        ).pack(anchor="w", **pad)
        self.lbl_spacing = tk.Label(
            p, text=f"{ANCHOR_SPACING:.1f} m",
            bg=C["panel"], fg=C["accent"],
            font=("Courier", 13, "bold")
        )
        self.lbl_spacing.pack()
        tk.Scale(
            p, from_=1.0, to=10.0, resolution=0.5,
            orient=tk.HORIZONTAL, variable=self.spacing_var,
            command=lambda _: self._slider_changed(),
            bg=C["panel"], fg=C["text"], troughcolor=C["grid"],
            highlightthickness=0, showvalue=False, length=185
        ).pack(**pad)
        self._sep(p)

        # ── 좌우 폭 슬라이더 ─────────────────────────────────────────────────
        tk.Label(
            p, text="좌우 폭 (m)",
            bg=C["panel"], fg=C["text"],
            font=("Courier", 9)
        ).pack(anchor="w", **pad)
        self.lbl_width = tk.Label(
            p, text=f"{ANCHOR_WIDTH:.1f} m",
            bg=C["panel"], fg="#e0a052",
            font=("Courier", 13, "bold")
        )
        self.lbl_width.pack()
        tk.Scale(
            p, from_=0.5, to=10.0, resolution=0.5,
            orient=tk.HORIZONTAL, variable=self.width_var,
            command=lambda _: self._slider_changed(),
            bg=C["panel"], fg=C["text"], troughcolor=C["grid"],
            highlightthickness=0, showvalue=False, length=185
        ).pack(**pad)
        self._sep(p)

        # ── 배제 반경 슬라이더 ───────────────────────────────────────────────
        # 기존 앵커로부터 이 거리 이내는 신규 앵커 배치 금지 (회색 × 표시)
        tk.Label(
            p, text="기존앵커 배제반경 (m)",
            bg=C["panel"], fg=C["text"],
            font=("Courier", 9)
        ).pack(anchor="w", **pad)
        self.lbl_min_dist = tk.Label(
            p, text=f"{MIN_DIST_FROM_EXISTING:.1f} m",
            bg=C["panel"], fg="#a070e0",
            font=("Courier", 13, "bold")
        )
        self.lbl_min_dist.pack()
        tk.Scale(
            p, from_=0.0, to=30.0, resolution=0.5,
            orient=tk.HORIZONTAL, variable=self.min_dist_var,
            command=lambda _: self._slider_changed(),
            bg=C["panel"], fg=C["text"], troughcolor=C["grid"],
            highlightthickness=0, showvalue=False, length=185
        ).pack(**pad)
        tk.Checkbutton(
            p, text=f"커버리지 원 ({COVERAGE_RADIUS:.0f}m)",
            variable=self.coverage_var,
            command=self._draw_map,
            bg=C["panel"], fg=C["text"],
            selectcolor=C["bg"], activebackground=C["panel"],
            font=("Courier", 8)
        ).pack(anchor="w", padx=12, pady=4)
        self._sep(p)

        # ── 범례 ─────────────────────────────────────────────────────────────
        tk.Label(
            p, text="범례", bg=C["panel"],
            fg=C["dim"], font=("Courier", 8)
        ).pack(anchor="w", padx=12)

        legend_items = [
            # (색상,           레이블,                    점선여부)
            (C["gz_ok"],      "기존 앵커 (위치 확인됨)",  False),
            (C["new_anchor"], "설치 예정 (L/R)",          False),
            (C["installing"], "설치 중",                  False),
            ("#505060",       "배제됨 (기존앵커 근처)",   False),
            (C["drone"],      "드론 현재 위치",            False),
        ]
        for color, label, dashed in legend_items:
            row = tk.Frame(p, bg=C["panel"])
            row.pack(anchor="w", padx=14, pady=1)
            ic = tk.Canvas(row, width=18, height=12,
                           bg=C["panel"], highlightthickness=0)
            ic.pack(side=tk.LEFT)
            if dashed:
                ic.create_line(0, 6, 18, 6, fill=color, width=2, dash=(4, 2))
            else:
                ic.create_oval(2, 2, 10, 10, fill=color, outline="")
            tk.Label(
                row, text=f" {label}", bg=C["panel"],
                fg=C["text"], font=("Courier", 8)
            ).pack(side=tk.LEFT)
        self._sep(p)

        # ── 미리보기 경로 정보 ───────────────────────────────────────────────
        self.info_var = tk.StringVar(value="선 없음")
        tk.Label(
            p, textvariable=self.info_var,
            bg=C["panel"], fg=C["new_anchor"],
            font=("Courier", 8), justify=tk.LEFT,
            wraplength=190
        ).pack(padx=12, pady=2)
        self._sep(p)

        # ── 버튼: 미션 시작 ──────────────────────────────────────────────────
        self.btn_confirm = tk.Button(
            p, text="▶  드론 미션 시작",
            command=self._confirm,
            bg=C["btn_confirm"], fg="white",
            font=("Courier", 10, "bold"),
            relief="flat", cursor="hand2", pady=7,
            state=tk.DISABLED   # 경로 드래그 전까지 비활성
        )
        self.btn_confirm.pack(fill=tk.X, padx=12, pady=3)

        # ── 버튼: 미확인 앵커 Gazebo 스폰 ───────────────────────────────────
        self.btn_spawn = tk.Button(
            p, text="⚡ 미확인 앵커 Gazebo 스폰",
            command=self._spawn_missing,
            bg=C["btn_spawn"], fg="white",
            font=("Courier", 8), relief="flat",
            cursor="hand2", pady=5
        )
        self.btn_spawn.pack(fill=tk.X, padx=12, pady=2)

        # ── 버튼: 초기화 ─────────────────────────────────────────────────────
        tk.Button(
            p, text="✕  초기화",
            command=self._clear,
            bg=C["btn_clear"], fg=C["text"],
            font=("Courier", 9), relief="flat",
            cursor="hand2", pady=4
        ).pack(fill=tk.X, padx=12, pady=2)
        self._sep(p)

        # ── Gazebo 모델 요약 ──────────────────────────────────────────────────
        self.gz_summary = tk.Label(
            p, text="Gazebo: 연결 대기",
            bg=C["panel"], fg=C["dim"],
            font=("Courier", 7), wraplength=190, justify=tk.LEFT
        )
        self.gz_summary.pack(padx=12, pady=2)

        # ── 로그창 ────────────────────────────────────────────────────────────
        tk.Label(
            p, text="LOG", bg=C["panel"],
            fg=C["dim"], font=("Courier", 7)
        ).pack(anchor="w", padx=12, pady=(4, 0))
        self.log = scrolledtext.ScrolledText(
            p, width=26, height=10,
            bg=C["log_bg"], fg=C["dim"],
            font=("Courier", 7), relief="flat",
            state=tk.DISABLED
        )
        self.log.pack(padx=8, pady=(0, 8), fill=tk.BOTH, expand=True)

    # ══════════════════════════════════════════════════════════════════════
    #  Gazebo 폴링 콜백 (GAZEBO CALLBACK)
    # ══════════════════════════════════════════════════════════════════════

    def _on_gz_update(self, models: set):
        """
        [백그라운드 스레드] Gazebo 모델 목록 수신 → 메인 스레드 위임.
        주의: tkinter 직접 조작 금지 — root.after() 로 전달.
        """
        self.root.after(0, lambda: self._apply_gz(models))

    def _apply_gz(self, models: set):
        """
        [메인 스레드] Gazebo 모델 집합 적용 & UI 갱신.
        기존 앵커 확인 수, 신규 앵커 설치 완료 여부를 업데이트한다.
        """
        prev           = self.gz_models
        self.gz_models = models

        # 기존 앵커의 Gazebo 확인 상태 계산
        anchor_names = {f"uwb_anchor_{a['id']}" for a in EXISTING_ANCHORS}
        found        = anchor_names & models
        missing      = anchor_names - models

        if self.gz_monitor.connected:
            # Gazebo 연결 O
            gz_uwb = sorted(m for m in models if m.startswith("uwb_anchor_"))
            self.gz_status_lbl.config(
                text=(f"● Gazebo 연결됨  |  기존앵커 확인 "
                      f"{len(found)}/{len(EXISTING_ANCHORS)}개"),
                fg="#52e052" if not missing else "#e0a052"
            )
            self.gz_summary.config(
                text=f"Gazebo UWB 모델 {len(gz_uwb)}개:\n" +
                     (", ".join(gz_uwb) if gz_uwb else "없음")
            )
        else:
            # Gazebo 연결 X
            self.gz_status_lbl.config(
                text="● Gazebo 미연결  (gz 명령어 없거나 실행 안됨)",
                fg="#e05050"
            )
            self.gz_summary.config(text="Gazebo: 오프라인")

        # 신규 앵커 설치 중 → Gazebo 에서 발견되면 설치완료로 전환
        newly_done = self.installing & models
        if newly_done:
            self.installed_new |= newly_done
            self.installing    -= newly_done
            for nm in newly_done:
                self._log(f"  ✅ Gazebo 확인됨: {nm}")

        # 변화 있을 때만 맵 재그리기
        if models != prev:
            self._draw_map()

    # ══════════════════════════════════════════════════════════════════════
    #  맵 렌더링 (DRAW MAP)
    #  모든 시각 요소를 매번 전체 재그림. 레이어 순서:
    #    1) 그리드
    #    2) 커버리지 원 (옵션)
    #    3) 홈 마커
    #    4) 경로선 + 코리더 경계
    #    5) 지그재그 연결선
    #    6) 기존 앵커 점
    #    7) 신규 앵커 점
    #    8) 드론 궤적 + 드론 아이콘   ← 신규
    #    9) 커버리지 반경 텍스트
    # ══════════════════════════════════════════════════════════════════════

    def _draw_map(self):
        """캔버스 전체를 지우고 현재 상태로 다시 그린다."""
        self.canvas.delete("all")
        m        = self.mapper
        show_cov = self.coverage_var.get()
        r_px     = m.m2px(COVERAGE_RADIUS)

        # ── 1) 그리드 ─────────────────────────────────────────────────────
        step = 2
        v = MAP_N[0]
        while v <= MAP_N[1] + 0.01:
            x0, y0 = m.w2c(v, MAP_E[0])
            x1, y1 = m.w2c(v, MAP_E[1])
            self.canvas.create_line(x0, y0, x1, y1, fill=C["grid"])
            if v == int(v) and int(v) % 5 == 0:
                self.canvas.create_text(
                    x0 - 16, y0, text=str(int(v)),
                    fill=C["dim"], font=("Courier", 7)
                )
            v = round(v + step, 1)
        v = MAP_E[0]
        while v <= MAP_E[1] + 0.01:
            x0, y0 = m.w2c(MAP_N[0], v)
            x1, y1 = m.w2c(MAP_N[1], v)
            self.canvas.create_line(x0, y0, x1, y1, fill=C["grid"])
            if v == int(v) and int(v) % 5 == 0:
                self.canvas.create_text(
                    x0, y0 + 12, text=str(int(v)),
                    fill=C["dim"], font=("Courier", 7)
                )
            v = round(v + step, 1)

        # ── 2) 커버리지 원 (맨 아래 레이어) ─────────────────────────────
        if show_cov:
            # 기존 앵커 커버리지
            for a in EXISTING_ANCHORS:
                cx, cy = m.w2c(a["n"], a["e"])
                nm     = f"uwb_anchor_{a['id']}"
                col    = C["cov_gz"] if nm in self.gz_models else C["cov_missing"]
                self.canvas.create_oval(
                    cx - r_px, cy - r_px, cx + r_px, cy + r_px,
                    outline=col, fill=col, stipple="gray12", width=1
                )
            # 신규 앵커 커버리지
            for (n, e) in self.preview:
                cx, cy = m.w2c(n, e)
                self.canvas.create_oval(
                    cx - r_px, cy - r_px, cx + r_px, cy + r_px,
                    outline=C["cov_new"], fill=C["cov_new"],
                    stipple="gray12", width=1
                )

        # ── 3) 드론 홈 마커 ───────────────────────────────────────────────
        hx, hy = m.w2c(*DRONE_HOME)
        self.canvas.create_oval(
            hx - 9, hy - 9, hx + 9, hy + 9,
            outline=C["home"], width=2, fill=C["bg"]
        )
        self.canvas.create_text(
            hx, hy, text="H",
            fill=C["home"], font=("Courier", 8, "bold")
        )

        # ── 4) 드래그 경로선 + 코리더 경계 ──────────────────────────────
        if self.line_start and self.line_end:
            # 중심 점선
            self.canvas.create_line(
                self.line_start[0], self.line_start[1],
                self.line_end[0],   self.line_end[1],
                fill=C["path_line"], width=1, dash=(6, 4)
            )
            # 코리더 좌우 경계
            n0, e0 = m.c2w(*self.line_start)
            n1, e1 = m.c2w(*self.line_end)
            total  = math.hypot(n1 - n0, e1 - e0)
            if total > 0.1:
                dn, de = (n1 - n0) / total, (e1 - e0) / total
                pn, pe = -de, dn
                w = self.width_var.get()
                for side in (+1, -1):
                    lx0, ly0 = m.w2c(n0 + side * w * pn, e0 + side * w * pe)
                    lx1, ly1 = m.w2c(n1 + side * w * pn, e1 + side * w * pe)
                    self.canvas.create_line(
                        lx0, ly0, lx1, ly1,
                        fill=C["path_border"], width=1, dash=(3, 5)
                    )

        # ── 5) 지그재그 연결선 ────────────────────────────────────────────
        if len(self.preview) >= 2:
            pts = [m.w2c(n, e) for (n, e) in self.preview]
            for i in range(len(pts) - 1):
                self.canvas.create_line(
                    pts[i][0], pts[i][1], pts[i+1][0], pts[i+1][1],
                    fill=C["new_anchor"], width=1, dash=(4, 4)
                )

        # ── 6) 기존 앵커 점 ──────────────────────────────────────────────
        # EXISTING_ANCHORS 는 좌표가 이미 알려진 앵커 — Gazebo 연결 여부와
        # 무관하게 항상 "확인됨(solid)" 상태로 표시한다.
        # (Gazebo 에 없어도 실제 현장에 설치된 앵커이므로 미확인 취급 불필요)
        for a in EXISTING_ANCHORS:
            cx, cy = m.w2c(a["n"], a["e"])
            r      = 8

            # 항상 solid 빨강 원 (Gazebo 여부 무관)
            self.canvas.create_oval(
                cx - r, cy - r, cx + r, cy + r,
                fill=C["gz_ok"], outline="white", width=1
            )
            self.canvas.create_text(
                cx, cy, text=str(a["id"]),
                fill="white", font=("Courier", 7, "bold")
            )
            # ID 라벨
            self.canvas.create_text(
                cx, cy - 14,
                text=f"A{a['id']}",
                fill=C["gz_ok"], font=("Courier", 7)
            )

        # ── 7) 신규 앵커 점 ──────────────────────────────────────────────
        for i, (n, e) in enumerate(self.preview):
            cx, cy = m.w2c(n, e)
            r      = 7
            aid    = len(EXISTING_ANCHORS) + i + 1
            nm     = f"uwb_anchor_{aid}"

            if nm in self.installed_new:
                col, lbl = C["gz_ok"],      "✓"   # 설치 완료
            elif nm in self.installing:
                col, lbl = C["installing"], "↓"   # 드론 이동 중
            else:
                col, lbl = C["new_anchor"], "L" if i % 2 == 0 else "R"  # 예정

            self.canvas.create_oval(
                cx - r, cy - r, cx + r, cy + r,
                fill=col, outline="white", width=1
            )
            self.canvas.create_text(
                cx, cy, text=lbl,
                fill=C["bg"], font=("Courier", 7, "bold")
            )

        # ── 7b) 배제된 후보 위치 — 회색 × ───────────────────────────────
        # smart_place_anchors 에서 걸러진 후보: 기존 앵커 너무 가까움
        # 드래그 중 실시간으로 "여기는 건너뜀" 피드백 제공
        for (n, e) in self.preview_blocked:
            bx, by = m.w2c(n, e)
            s = 5   # × 크기
            self.canvas.create_line(bx-s, by-s, bx+s, by+s,
                                     fill="#505060", width=1)
            self.canvas.create_line(bx-s, by+s, bx+s, by-s,
                                     fill="#505060", width=1)

        # ── 8) 드론 위치 (궤적 → 현재 위치 순서로 그림) ─────────────────
        self._draw_drone()

        # ── 9) 커버리지 반경 텍스트 (우하단) ────────────────────────────
        if show_cov:
            self.canvas.create_text(
                self.CW - 8, self.CH - 6,
                text=f"커버리지 반경 {COVERAGE_RADIUS:.0f} m",
                anchor="se", fill=C["dim"], font=("Courier", 7)
            )

    def _draw_drone(self):
        """
        드론 위치를 맵에 그린다.
        ‣ 궤적(trail): 반투명 작은 점들로 이동 경로 표시
        ‣ 현재 위치: 청록 십자(+) + 원 + 고도 텍스트
        drone_pos 가 None 이면 아무것도 그리지 않는다.
        """
        if not self.drone_pos:
            return

        m  = self.mapper
        dn, de, dalt = self.drone_pos

        # 궤적 점들
        for pt in self.drone_trail:
            tx, ty = m.w2c(pt[0], pt[1])
            self.canvas.create_oval(
                tx - 2, ty - 2, tx + 2, ty + 2,
                fill=C["drone_trail"], outline=""
            )

        # 현재 위치
        cx, cy = m.w2c(dn, de)
        r = 7

        # 원
        self.canvas.create_oval(
            cx - r, cy - r, cx + r, cy + r,
            outline=C["drone"], width=2, fill=""
        )
        # 십자
        self.canvas.create_line(cx - r - 3, cy, cx + r + 3, cy,
                                 fill=C["drone"], width=1)
        self.canvas.create_line(cx, cy - r - 3, cx, cy + r + 3,
                                 fill=C["drone"], width=1)
        # 고도 + 좌표 텍스트
        self.canvas.create_text(
            cx, cy - r - 12,
            text=f"🚁 ({dn:.1f}, {de:.1f})  {abs(dalt):.1f}m",
            fill=C["drone"], font=("Courier", 7, "bold")
        )

    # ══════════════════════════════════════════════════════════════════════
    #  드론 위치 업데이트 (DRONE POSITION UPDATE)
    # ══════════════════════════════════════════════════════════════════════

    def _update_drone_pos(self, n: float, e: float, alt: float):
        """
        드론 위치를 갱신하고 맵을 다시 그린다.
        항상 메인 스레드에서 호출해야 한다.
        (백그라운드라면 root.after(0, lambda: self._update_drone_pos(...)) 사용)

        Parameters
        ----------
        n   : float  North (m)
        e   : float  East  (m)
        alt : float  고도 (m, 양수=위 로 표시. NED alt 를 -1 곱해서 전달)
        """
        self.drone_pos = (n, e, alt)

        # 궤적 추가 (최대 TRAIL_MAX 개 유지)
        self.drone_trail.append((n, e))
        if len(self.drone_trail) > self.TRAIL_MAX:
            self.drone_trail.pop(0)

        # 위치 레이블 갱신
        self.drone_pos_lbl.config(
            text=f"🚁  N:{n:+.1f}  E:{e:+.1f}  Alt:{alt:.1f}m"
        )

        self._draw_map()

    def _clear_drone_pos(self):
        """드론 위치 초기화 (착륙 후 or 초기화 시 호출)."""
        self.drone_pos   = None
        self.drone_trail = []
        self.drone_pos_lbl.config(text="🚁  위치 없음")
        self._draw_map()

    # ══════════════════════════════════════════════════════════════════════
    #  마우스 이벤트 (MOUSE EVENTS)
    # ══════════════════════════════════════════════════════════════════════

    def _on_press(self, ev):
        """마우스 클릭: 드래그 시작점 설정 & 미리보기 초기화."""
        self.line_start = (ev.x, ev.y)
        self.line_end   = None
        self.preview    = []
        self._draw_map()

    def _on_drag(self, ev):
        """마우스 드래그 중: 실시간 미리보기 갱신."""
        self.line_end = (ev.x, ev.y)
        self._update_preview()

    def _on_release(self, ev):
        """마우스 놓기: 최종 미리보기 확정 & 미션 버튼 활성화."""
        self.line_end = (ev.x, ev.y)
        self._update_preview()
        self.btn_confirm.config(
            state=tk.NORMAL if self.preview else tk.DISABLED
        )

    def _update_preview(self):
        """
        드래그 선으로 스마트 앵커 배치 계산 & 정보 텍스트 갱신.
        smart_place_anchors() 로 기존 앵커 근처 후보를 제외하고,
        제외된 위치는 self.preview_blocked 에 저장해 맵에 × 로 표시.
        """
        if not (self.line_start and self.line_end):
            return
        n0, e0 = self.mapper.c2w(*self.line_start)
        n1, e1 = self.mapper.c2w(*self.line_end)

        # 스마트 배치: accepted = 채택, blocked = 제외
        accepted, blocked = smart_place_anchors(
            n0, e0, n1, e1,
            self.spacing_var.get(),
            self.width_var.get(),
            EXISTING_ANCHORS,
            self.min_dist_var.get()
        )
        self.preview         = accepted
        self.preview_blocked = blocked
        self._draw_map()

        # 정보 텍스트
        L = sum(1 for i in range(len(self.preview)) if i % 2 == 0)
        R = len(self.preview) - L
        skipped = len(blocked)
        self.info_var.set(
            f"경로: {math.hypot(n1-n0, e1-e0):.1f} m\n"
            f"채택: {len(self.preview)}개  (L:{L}  R:{R})\n"
            f"배제: {skipped}개  (기존앵커 근처)\n"
            f"간격 {self.spacing_var.get():.1f}m  폭 {self.width_var.get():.1f}m"
        )

    def _slider_changed(self):
        """슬라이더 변경 시 라벨 & 미리보기 갱신."""
        self.lbl_spacing.config(text=f"{self.spacing_var.get():.1f} m")
        self.lbl_width.config(text=f"{self.width_var.get():.1f} m")
        self.lbl_min_dist.config(text=f"{self.min_dist_var.get():.1f} m")
        self._update_preview()

    # ══════════════════════════════════════════════════════════════════════
    #  버튼 콜백 (BUTTON CALLBACKS)
    # ══════════════════════════════════════════════════════════════════════

    def _spawn_missing(self):
        """EXISTING_ANCHORS 중 Gazebo 미확인 앵커를 스폰."""
        missing = [a for a in EXISTING_ANCHORS
                   if f"uwb_anchor_{a['id']}" not in self.gz_models]
        if not missing:
            self._log("  ℹ 모든 기존 앵커 Gazebo 확인됨")
            return
        self._log(f"  ⚡ 미확인 {len(missing)}개 스폰 시도...")

        def _do():
            for a in missing:
                anchor = dict(a, name=f"uwb_anchor_{a['id']}")
                GazeboMonitor.spawn(anchor, world=CFG["gz_world"])
                self._log(f"    → uwb_anchor_{a['id']} 스폰")

        threading.Thread(target=_do, daemon=True).start()

    def _clear(self):
        """경로·앵커·드론 위치 전체 초기화."""
        self.line_start      = self.line_end = None
        self.preview         = []
        self.preview_blocked = []   # 배제 후보도 초기화
        self.installing      = set()
        self.installed_new   = set()
        self.info_var.set("선 없음")
        self.btn_confirm.config(state=tk.DISABLED)
        self._clear_drone_pos()   # 드론 위치도 초기화

    def _confirm(self):
        """미션 시작 버튼: 앵커 목록 생성 후 백그라운드 미션 실행."""
        if not self.preview:
            return
        anchors = [
            {
                "id"  : len(EXISTING_ANCHORS) + i + 1,
                "n"   : n,
                "e"   : e,
                "name": f"uwb_anchor_{len(EXISTING_ANCHORS)+i+1}"
            }
            for i, (n, e) in enumerate(self.preview)
        ]
        self.btn_confirm.config(state=tk.DISABLED)
        self._log("▶ 드론 미션 시작")
        threading.Thread(
            target=self._run_mission, args=(anchors,), daemon=True
        ).start()

    # ══════════════════════════════════════════════════════════════════════
    #  미션 실행 (MISSION EXECUTION)
    # ══════════════════════════════════════════════════════════════════════

    def _run_mission(self, anchors: list):
        """
        [백그라운드 스레드] 미션 진입점.
        MAVSDK 있으면 실제 비행, 없으면 더미 시뮬레이션.
        """
        if not HAS_MAVSDK:
            self._run_dummy_mission(anchors)
        else:
            asyncio.run(self._mission_coro(anchors))

    # ── 더미 시뮬레이션 (mavsdk 없을 때) ───────────────────────────────

    def _run_dummy_mission(self, anchors: list):
        """
        MAVSDK 없이 드론 이동을 시뮬레이션한다.
        ‣ 이륙: 홈 → 비행 고도로 상승 애니메이션
        ‣ 각 앵커: 이동 → 투하 → Gazebo 스폰
        ‣ 귀환: 앵커 → 홈 복귀 애니메이션
        드론 아이콘은 _update_drone_pos 를 통해 실시간 맵에 표시된다.
        """
        self._log("  [SIM] mavsdk 없음 → 더미 시뮬레이션")
        n_home, e_home = DRONE_HOME
        flight_alt     = abs(CFG["flight_alt"])   # 양수로 표시

        # ── 이륙 ─────────────────────────────────────────────────────────
        self._log("  [SIM] 이륙...")
        # 고도만 서서히 올리는 애니메이션 (10단계)
        for step in range(11):
            alt = flight_alt * (step / 10)
            self.root.after(0, lambda n=n_home, e=e_home, a=alt:
                            self._update_drone_pos(n, e, a))
            time.sleep(0.15)

        prev_n, prev_e = n_home, e_home

        # ── 각 앵커 이동 → 투하 ─────────────────────────────────────────
        for a in anchors:
            target_n, target_e = a["n"], a["e"]
            self._log(f"\n─ {a['name']} 이동  ({target_n:.1f}, {target_e:.1f})")
            self.root.after(0, lambda nm=a["name"]: self._mark_installing(nm))

            # 이동 애니메이션 (20단계)
            for step in range(21):
                t  = step / 20
                cn = prev_n + t * (target_n - prev_n)
                ce = prev_e + t * (target_e - prev_e)
                self.root.after(0, lambda n=cn, e=ce, alt=flight_alt:
                                self._update_drone_pos(n, e, alt))
                time.sleep(0.12)

            # 하강 애니메이션 (투하 고도까지)
            drop_alt = abs(CFG["drop_alt"])
            for step in range(11):
                t   = step / 10
                alt = flight_alt + t * (drop_alt - flight_alt)
                self.root.after(0, lambda n=target_n, e=target_e, a=alt:
                                self._update_drone_pos(n, e, a))
                time.sleep(0.1)

            # 투하
            self._log(f"  [SIM] {a['name']} 투하!")
            time.sleep(0.5)
            GazeboMonitor.spawn(a, world=CFG["gz_world"])
            self._log(f"  [SIM] Gazebo 스폰 → 폴링 확인 대기...")

            # 상승 복귀
            for step in range(11):
                t   = step / 10
                alt = drop_alt + t * (flight_alt - drop_alt)
                self.root.after(0, lambda n=target_n, e=target_e, a=alt:
                                self._update_drone_pos(n, e, a))
                time.sleep(0.1)

            prev_n, prev_e = target_n, target_e
            time.sleep(0.3)

        # ── 홈 복귀 ─────────────────────────────────────────────────────
        self._log("\n[SIM] 홈 복귀...")
        for step in range(21):
            t  = step / 20
            cn = prev_n + t * (n_home - prev_n)
            ce = prev_e + t * (e_home - prev_e)
            self.root.after(0, lambda n=cn, e=ce, alt=flight_alt:
                            self._update_drone_pos(n, e, alt))
            time.sleep(0.12)

        # 착륙 (고도 0으로 하강)
        for step in range(11):
            alt = flight_alt * (1 - step / 10)
            self.root.after(0, lambda n=n_home, e=e_home, a=alt:
                            self._update_drone_pos(n, e, a))
            time.sleep(0.15)

        self._log("✅ [SIM] 임무 완료!")
        self.root.after(0, lambda: self._clear_drone_pos())
        self.root.after(0, lambda: self.btn_confirm.config(state=tk.NORMAL))

    # ── 마킹 헬퍼 ───────────────────────────────────────────────────────

    def _mark_installing(self, name: str):
        """앵커를 '설치 중' 상태로 표시 (메인 스레드에서 호출)."""
        self.installing.add(name)
        self._draw_map()

    # ── 실제 MAVSDK 미션 코루틴 ────────────────────────────────────────

    async def _mission_coro(self, anchors: list):
        """
        MAVSDK 를 이용한 실제 드론 비행 미션.
        ‣ 드론 연결 → 이륙 → 웨이포인트 순회 → 앵커 투하 → 귀환 → 착륙
        ‣ 텔레메트리 위치는 별도 태스크(_telemetry_task)로 폴링
        """
        try:
            drone = System()
            await drone.connect(system_address=CFG["address"])
            self._log("  드론 연결 대기...")
            async for s in drone.core.connection_state():
                if s.is_connected:
                    self._log("  ✅ 연결됨"); break
            async for h in drone.telemetry.health():
                if h.is_global_position_ok and h.is_local_position_ok:
                    self._log("  ✅ 위치 OK"); break

            # 텔레메트리 위치 폴링 태스크 시작
            # (미션 내내 실행되며 드론 아이콘을 실시간 갱신)
            pos_task = asyncio.create_task(self._telemetry_task(drone))

            try:
                await drone.gripper.grab()
            except Exception as ex:
                self._log(f"  ⚠  그리퍼: {ex}")

            self._log("  이륙...")
            await drone.action.arm()
            await drone.offboard.set_position_ned(
                PositionNedYaw(0., 0., CFG["flight_alt"], 0.)
            )
            await drone.offboard.start()
            await asyncio.sleep(CFG["takeoff_wait"])

            for a in anchors:
                n, e = a["n"], a["e"]
                self._log(f"\n─ {a['name']} 이동")
                self.root.after(0, lambda nm=a["name"]: self._mark_installing(nm))

                await drone.offboard.set_position_ned(
                    PositionNedYaw(n, e, CFG["flight_alt"], 0.)
                )
                await asyncio.sleep(CFG["move_wait"])
                await drone.offboard.set_position_ned(
                    PositionNedYaw(n, e, CFG["drop_alt"], 0.)
                )
                await asyncio.sleep(2.0)

                self._log("  투하!")
                try:
                    await drone.gripper.release()
                except Exception:
                    await drone.action.set_actuator(CFG["gripper_actuator"], 1.)
                    await asyncio.sleep(1.5)
                    await drone.action.set_actuator(CFG["gripper_actuator"], 0.)

                await asyncio.sleep(CFG["drop_wait"])
                # Gazebo 스폰 → 2초 후 폴링이 감지 → 맵 자동 갱신
                GazeboMonitor.spawn(a, world=CFG["gz_world"])
                self._log(f"  Gazebo 스폰 → 폴링 확인 대기...")

                await drone.offboard.set_position_ned(
                    PositionNedYaw(n, e, CFG["flight_alt"], 0.)
                )
                await asyncio.sleep(3.0)

            self._log("\n홈 복귀...")
            await drone.offboard.set_position_ned(
                PositionNedYaw(0., 0., CFG["flight_alt"], 0.)
            )
            await asyncio.sleep(CFG["move_wait"])
            await drone.offboard.stop()
            await drone.action.land()
            self._log("✅ 임무 완료!")

        except Exception as ex:
            self._log(f"❌ {ex}")
        finally:
            # 텔레메트리 태스크 취소 & 드론 위치 초기화
            try:
                pos_task.cancel()
            except Exception:
                pass
            self.root.after(0, self._clear_drone_pos)
            self.root.after(0, lambda: self.btn_confirm.config(state=tk.NORMAL))

    async def _telemetry_task(self, drone):
        """
        [MAVSDK 비동기 태스크] 드론 로컬 위치를 구독하여 맵에 반영.
        ‣ position_velocity_ned 스트림 → (N, E, -D=고도) 추출
        ‣ root.after() 로 메인 스레드의 _update_drone_pos 호출
        """
        try:
            async for pos in drone.telemetry.position_velocity_ned():
                n   = pos.position.north_m
                e   = pos.position.east_m
                alt = -pos.position.down_m   # NED → 양수=위로 변환
                self.root.after(0, lambda nn=n, ee=e, aa=alt:
                                self._update_drone_pos(nn, ee, aa))
        except asyncio.CancelledError:
            pass   # 미션 종료 시 태스크 취소 — 정상 종료

    # ══════════════════════════════════════════════════════════════════════
    #  로그 헬퍼 (LOG HELPER)
    # ══════════════════════════════════════════════════════════════════════

    def _log(self, msg: str):
        """
        로그창에 메시지 추가.
        백그라운드 스레드에서 호출해도 안전하도록 root.after() 경유.
        """
        def _do():
            self.log.config(state=tk.NORMAL)
            self.log.insert(tk.END, msg + "\n")
            self.log.see(tk.END)
            self.log.config(state=tk.DISABLED)
        self.root.after(0, _do)


# ════════════════════════════════════════════════════════════════════════════
#  엔트리 포인트 (ENTRY POINT)
# ════════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    root = tk.Tk()
    app  = UWBApp(root)
    root.mainloop()