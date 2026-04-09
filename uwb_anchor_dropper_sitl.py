"""
╔══════════════════════════════════════════════════════════════════════════════╗
║              UWB 앵커 GUI 설치 도구 v5  —  로버 디포 + 스마트 재배치        ║
╠══════════════════════════════════════════════════════════════════════════════╣
║                                                                              ║
║  💡 핵심 개념 (CORE CONCEPT)                                                 ║
║  ─────────────────────────────────────────────────────────────────────────  ║
║                                                                              ║
║  [로버] ──→ 진행방향 ──→ [끝점]                                             ║
║    └─ UWB 디포를 싣고 경로 시작점 → 끝점 방향으로 계속 전진               ║
║                                                                              ║
║  드론 임무 흐름:                                                             ║
║    1) 로버(현재 위치)로 날아가서 UWB를 집는다                               ║
║    2) 설치 위치로 날아가서 내려놓는다                                        ║
║    3) 로버는 그 사이에 계속 전진 (다음 픽업 위치 갱신됨)                    ║
║                                                                              ║
║  재배치 로직 (디포 재고 소진 시):                                            ║
║    - 로버가 이미 지나친 뒤쪽 앵커 = 더 이상 필요 없는 앵커                  ║
║    - "경로 방향 투영값 < 현재 로버 투영값" → 로버 뒤에 있음                 ║
║    - 그 중에서 끝점(endpoint)과의 거리가 가장 먼 것 선택                    ║
║      (= 가장 뒤에 있어서 커버리지 필요성이 가장 낮음)                       ║
║    - 재활용 후보: EXISTING_ANCHORS + 이미 설치한 신규 앵커 모두 포함        ║
║                                                                              ║
║  ❓ 왜 "끝점에서 가장 먼 것"을 선택하나?                                    ║
║    로버가 이미 한참 지나간 앵커일수록 중복 커버리지가 심하고,               ║
║    그 앵커를 앞으로 옮길수록 전체 네트워크 공백이 최소화되기 때문.           ║
║                                                                              ║
║  📐 좌표계: NED (North-East-Down)  N↑ E→  alt 음수=위                       ║
║  🖥  실행:  python3 uwb_gui.py                                               ║
╚══════════════════════════════════════════════════════════════════════════════╝
"""

import os
# ── 반드시 모든 import 전에 설정 ─────────────────────────────────────────────
# mavsdk/gz.msgs10 둘 다 protobuf를 쓰는데, C extension 모드로 먼저 로드되면
# gz.msgs10(protobuf 3.x 생성 코드)이 "Descriptors cannot be created" 에러남.
# 환경변수를 여기서 설정하면 protobuf가 pure-python 모드로 초기화됨.
os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"

import tkinter as tk
from tkinter import scrolledtext
import asyncio
import math
import time
import threading
import subprocess

try:
    from mavsdk import System
    from mavsdk.offboard import OffboardError, PositionNedYaw
    HAS_MAVSDK = True
except ImportError:
    HAS_MAVSDK = False

try:
    from camera_detector import AnchorDetector, HAS_CV2, HAS_PIL
except ImportError:
    HAS_CV2 = HAS_PIL = False
    class AnchorDetector:          # 카메라 모듈 없을 때 더미
        source = "none"; ok = False
        def __init__(self, **kw): pass
        def get_frame(self): return None
        def detect_anchor(self, f=None): return None
        def annotate(self, f, d=None): return f
        def get_tk_image(self, f=None, w=200, h=150): return None
        def stop(self): pass
        @staticmethod
        def pixel_to_ned_offset(*a, **kw): return 0., 0.

try:
    from uwb_localizer import UWBLocalizer
    HAS_UWB = True
except ImportError:
    HAS_UWB = False
    class UWBLocalizer:            # UWB 모듈 없을 때 더미
        def __init__(self, **kw): pass
        def start(self): pass
        def stop(self): pass
        def get_position(self, max_age=2.0): return None
        def get_distances(self): return {}
        def get_quality(self): return {"sigma": (999,999,999)}
        def set_sitl_drone_pos(self, n, e, z): pass
        def set_drone_velocity(self, n, e, z): pass
        def set_baro_altitude(self, a): pass
        def add_sitl_anchor(self, n, e, z=0.): pass

from mavsdk.mocap import (VisionPositionEstimate as _VPE,
                          PositionBody as _PosBody,
                          AngleBody as _AngBody,
                          Covariance as _Cov)


# ════════════════════════════════════════════════════════════════════════════
#  전역 설정
# ════════════════════════════════════════════════════════════════════════════
# SITL  = Gazebo SITL (UDP, PX4 실제 제어)
# REAL  = 실제 드론 (시리얼)
# mavsdk 없으면 자동으로 tkinter 더미 애니메이션 fallback
MODE = "SITL"

CFG = {
    "address"         : "udpin://0.0.0.0:14540",           # SITL: Gazebo PX4
    "real_address"    : "serial:///dev/ttyACM0:57600",   # REAL: 실제 드론
    "flight_alt"      : -2.5,
    "drop_alt"        : -0.4,
    "takeoff_wait"    : 5.0,
    "drop_wait"       : 1.5,
    "gripper_actuator": 1,
    "gz_world"        : "default",
    # 카메라 설정
    "camera_source"   : "gz",     # "gz" | "v4l2" | "none"
    "camera_device"   : 0,        # v4l2 장치 번호
    "camera_topic"    : "/drone/downward_cam/image",
    "camera_fov"      : 90.0,     # 수평 FOV (degrees)
    "cam_align_tol"   : 0.15,     # 앵커 중심 정렬 허용 오차 (m)
    "cam_align_wait"  : 0.3,      # 정렬 확인 대기 (s)
    # UWB 설정
    "uwb_port"        : "/dev/ttyACM0",  # 실기체: DWM3001C 시리얼 포트
    "uwb_baud"        : 115200,
    "uwb_sitl"        : True,     # SITL 모드에서 거리 시뮬레이션 활성화
    # VISION_POSITION_ESTIMATE → PX4 EKF2 주입 설정
    # MAVSDK mocap API 사용: drone 연결 재사용, REAL/SITL 공통 동작
    "vision_rate_hz"  : 30,       # 주입 Hz (mocap.set_vision_position_estimate)
}

MAP_N           = (-2.0, 20.0)
MAP_E           = (-2.0, 20.0)
COVERAGE_RADIUS = 30.0
GZ_POLL_SEC     = 2.0

EXISTING_ANCHORS = [
    {"id": 1, "n":  2.0, "e":  2.0},
    {"id": 2, "n":  2.0, "e": 10.0},
    {"id": 3, "n": 10.0, "e":  5.0},
    {"id": 4, "n": 15.0, "e": 15.0},
]

ANCHOR_SPACING         = 3.5
ANCHOR_WIDTH           = 3.0
MIN_DIST_FROM_EXISTING = 8.0
DRONE_HOME             = (0.0, 0.0)
DEPOT_STOCK_DEFAULT    = 3   # 로버 초기 UWB 보유 수량

# ── 앵커 ID 전역 카운터 (충돌 방지) ─────────────────────────────────────────
_anchor_id_lock = threading.Lock()
_next_anchor_id = len(EXISTING_ANCHORS) + 1

def _alloc_anchor_id() -> int:
    global _next_anchor_id
    with _anchor_id_lock:
        aid = _next_anchor_id
        _next_anchor_id += 1
    return aid


# ════════════════════════════════════════════════════════════════════════════
#  색상 팔레트
# ════════════════════════════════════════════════════════════════════════════
C = {
    "bg"           : "#0a0f14",
    "panel"        : "#111820",
    "grid"         : "#1a2530",
    "border"       : "#1e3a4a",
    "home"         : "#ffd060",
    "text"         : "#c8d8e8",
    "dim"          : "#3a5060",
    "accent"       : "#4a9eff",
    "log_bg"       : "#080d12",
    "gz_ok"        : "#e05252",
    "new_anchor"   : "#52e0a0",
    "installing"   : "#ffd060",
    "cov_gz"       : "#e05252",
    "cov_new"      : "#52e0a0",
    "path_line"    : "#4a9eff",
    "path_border"  : "#2a4060",
    "btn_confirm"  : "#1a6a40",
    "btn_spawn"    : "#1a4a6a",
    "btn_clear"    : "#4a2a2a",
    "drone"        : "#00e5ff",
    "drone_trail"  : "#004455",
    # 로버
    "rover"        : "#ffaa00",
    "rover_trail"  : "#443300",
    "rover_zone"   : "#1a1200",
    # 재배치
    "relocated"    : "#606878",
    "reloc_src_hl" : "#ff6030",
    "reloc_arrow"  : "#ff8844",
}


# ════════════════════════════════════════════════════════════════════════════
#  Gazebo 모니터
# ════════════════════════════════════════════════════════════════════════════
class GazeboMonitor:

    def __init__(self, on_update, world="default"):
        self._on_update = on_update
        self._world     = world
        self._running   = False
        self.connected  = False

    def start(self):
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def stop(self):
        self._running = False

    def _loop(self):
        while self._running:
            self._on_update(self._fetch())
            time.sleep(GZ_POLL_SEC)

    def _fetch(self) -> set:
        try:
            r = subprocess.run(
                ["gz", "model", "--list"],
                capture_output=True, text=True, timeout=5
            )
            if r.returncode == 0:
                self.connected = True
                # "Available models:" 이후 "    - <name>" 형식 파싱
                models = set()
                in_list = False
                for ln in r.stdout.splitlines():
                    if "Available models:" in ln:
                        in_list = True
                        continue
                    if in_list:
                        name = ln.strip().lstrip("- ").strip()
                        if name:
                            models.add(name)
                return models
            self.connected = False
            return set()
        except Exception:
            self.connected = False
            return set()

    @staticmethod
    def spawn(anchor: dict, world: str = "default"):
        n, e = anchor["n"], anchor["e"]
        name = anchor.get("name", f"uwb_anchor_{anchor['id']}")
        sdf_xml = (
            f'<?xml version="1.0"?><sdf version="1.7">'
            f'<model name="{name}"><static>true</static>'
            f'<link name="link"><visual name="visual">'
            f'<geometry><cylinder><radius>0.12</radius><length>0.35</length>'
            f'</cylinder></geometry>'
            f'<material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material>'
            f'</visual></link></model></sdf>'
        )
        sdf_esc = sdf_xml.replace('"', '\\"')   # protobuf text 포맷 이스케이프
        req = (f'sdf: "{sdf_esc}" '
               f'pose {{ position {{ x: {e} y: {n} z: 0.175 }} }} name: "{name}"')
        try:
            subprocess.run(
                ["gz", "service", "-s", f"/world/{world}/create",
                 "--reqtype", "gz.msgs.EntityFactory",
                 "--reptype", "gz.msgs.Boolean",
                 "--timeout", "2000", "--req", req],
                capture_output=True, timeout=5
            )
        except Exception:
            pass

    @staticmethod
    def despawn(name: str, world: str = "default"):
        try:
            subprocess.run(
                ["gz", "service", "-s", f"/world/{world}/remove",
                 "--reqtype", "gz.msgs.Entity",
                 "--reptype", "gz.msgs.Boolean",
                 "--timeout", "2000", "--req", f'name: "{name}"'],
                capture_output=True, timeout=5
            )
        except Exception:
            pass


# ════════════════════════════════════════════════════════════════════════════
#  좌표 변환기
# ════════════════════════════════════════════════════════════════════════════
class CoordMapper:

    def __init__(self, w, h, pad=40):
        self.x0 = pad
        self.y0 = pad
        self.uw = w - pad * 2
        self.uh = h - pad * 2

    def w2c(self, n, e):
        px = self.x0 + (e - MAP_E[0]) / (MAP_E[1] - MAP_E[0]) * self.uw
        py = self.y0 + (1 - (n - MAP_N[0]) / (MAP_N[1] - MAP_N[0])) * self.uh
        return px, py

    def c2w(self, px, py):
        e = MAP_E[0] + (px - self.x0) / self.uw * (MAP_E[1] - MAP_E[0])
        n = MAP_N[0] + (1 - (py - self.y0) / self.uh) * (MAP_N[1] - MAP_N[0])
        return round(n, 2), round(e, 2)

    def m2px(self, meters):
        return meters / (MAP_E[1] - MAP_E[0]) * self.uw


# ════════════════════════════════════════════════════════════════════════════
#  앵커 배치 알고리즘
# ════════════════════════════════════════════════════════════════════════════

def _zigzag_candidates(n0, e0, n1, e1, spacing, width):
    total = math.hypot(n1 - n0, e1 - e0)
    if total < 0.1:
        return []
    dn, de = (n1-n0)/total, (e1-e0)/total
    pn, pe = -de, dn
    count  = max(1, round(total / spacing))
    result = []
    for i in range(count + 1):
        t  = i / count
        cn = n0 + t * (n1 - n0)
        ce = e0 + t * (e1 - e0)
        side = +1 if i % 2 == 0 else -1
        result.append((round(cn + side*width*pn, 2),
                       round(ce + side*width*pe, 2)))
    return result


def smart_place_anchors(n0, e0, n1, e1, spacing, width, existing, min_dist):
    candidates        = _zigzag_candidates(n0, e0, n1, e1, spacing, width)
    accepted, blocked = [], []
    new_gap           = spacing * 0.7
    for (cn, ce) in candidates:
        too_existing = any(math.hypot(cn-a["n"], ce-a["e"]) < min_dist
                           for a in existing)
        too_new      = any(math.hypot(cn-an, ce-ae) < new_gap
                           for (an, ae) in accepted)
        if too_existing or too_new:
            blocked.append((cn, ce))
        else:
            accepted.append((cn, ce))
    return accepted, blocked


# ════════════════════════════════════════════════════════════════════════════
#  경로 투영 유틸리티
# ════════════════════════════════════════════════════════════════════════════

def _make_proj(n0, e0, n1, e1):
    path_len = math.hypot(n1-n0, e1-e0)
    if path_len < 0.01:
        return (lambda an, ae: 0.0), 0.0
    pn = (n1-n0) / path_len
    pe = (e1-e0) / path_len
    return (lambda an, ae: (an-n0)*pn + (ae-e0)*pe), path_len


# ════════════════════════════════════════════════════════════════════════════
#  미션 계획 함수
# ════════════════════════════════════════════════════════════════════════════

def plan_mission(anchors: list, depot_stock: int,
                 n0: float, e0: float,
                 n1: float, e1: float,
                 existing_anchors: list) -> list:
    N = len(anchors)
    if N == 0:
        return []

    proj, path_len = _make_proj(n0, e0, n1, e1)

    steps           = []
    remaining_depot = depot_stock

    reloc_existing_ids = set()
    reloc_new_idxs     = set()
    installed_new = []

    for k, anchor in enumerate(anchors):

        rover_t = k / N if N > 1 else 0.0
        rover_n = n0 + rover_t * (n1 - n0)
        rover_e = e0 + rover_t * (e1 - e0)
        rover_p = rover_t * path_len

        if remaining_depot > 0:
            steps.append({
                "type"      : "from_depot",
                "anchor"    : anchor,
                "rover_pos" : (rover_n, rover_e),
                "rover_t"   : rover_t,
            })
            remaining_depot -= 1
            installed_new.append(anchor)   # 실제 설치 → 재배치 후보로 등록

        else:
            candidates = []

            for ea in existing_anchors:
                if ea["id"] in reloc_existing_ids:
                    continue
                if proj(ea["n"], ea["e"]) < rover_p:
                    dist = math.hypot(ea["n"]-n1, ea["e"]-e1)
                    candidates.append(("existing", ea, None, dist))

            for j, ia in enumerate(installed_new):
                if j in reloc_new_idxs:
                    continue
                if proj(ia["n"], ia["e"]) < rover_p:
                    dist = math.hypot(ia["n"]-n1, ia["e"]-e1)
                    candidates.append(("installed", ia, j, dist))

            if candidates:
                stype, src, sidx, _ = max(candidates, key=lambda c: c[3])

                if stype == "existing":
                    reloc_existing_ids.add(src["id"])
                else:
                    reloc_new_idxs.add(sidx)

                steps.append({
                    "type"        : "relocate",
                    "anchor"      : anchor,
                    "source"      : src,
                    "source_type" : stype,
                    "rover_pos"   : (rover_n, rover_e),
                    "rover_t"     : rover_t,
                })
                installed_new.append(anchor)   # 실제 설치 → 재배치 후보로 등록
            else:
                steps.append({
                    "type"   : "skip",
                    "anchor" : anchor,
                    "reason" : "로버 뒤 재배치 후보 없음 (모두 앞쪽이거나 소진)",
                })
                # skip된 앵커는 미설치이므로 installed_new에 추가하지 않음

    return steps


# ════════════════════════════════════════════════════════════════════════════
#  메인 GUI 클래스
# ════════════════════════════════════════════════════════════════════════════
class UWBApp:
    # ── 캔버스 크기 (확대) ────────────────────────────────────────────────
    CW        = 720   # 480 → 720
    CH        = 720   # 480 → 720
    TRAIL_MAX = 80
    # 우측 패널 너비
    PW        = 240   # 215 → 240

    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("UWB Anchor Installer  v5 — Rover Depot")
        self.root.configure(bg=C["bg"])
        self.root.resizable(True, True)   # 크기 조절 허용

        # 기본 상태
        self.line_start      = None
        self.line_end        = None
        self.preview         = []
        self.preview_blocked = []
        self.gz_models       = set()
        self.installing      = set()
        self.installed_new   = set()

        self.drone_pos   = None
        self.drone_trail = []
        self.drone_home  = list(DRONE_HOME)   # [n, e] — 우클릭으로 변경 가능

        self.rover_pos   = None
        self.rover_t     = 0.0
        self.rover_trail = []

        self.path_start = (0.0, 0.0)
        self.path_end   = (0.0, 0.0)

        self.depot_remaining         = DEPOT_STOCK_DEFAULT
        self.relocated_existing_ids  = set()
        self.reloc_new_positions     = []
        self.mission_steps           = []
        self._rover_gen              = 0
        self._stop_evt               = threading.Event()

        self.spacing_var     = tk.DoubleVar(value=ANCHOR_SPACING)
        self.width_var       = tk.DoubleVar(value=ANCHOR_WIDTH)
        self.min_dist_var    = tk.DoubleVar(value=MIN_DIST_FROM_EXISTING)
        self.coverage_var    = tk.BooleanVar(value=False)
        self.depot_stock_var = tk.IntVar(value=DEPOT_STOCK_DEFAULT)

        self._build_ui()
        self.gz_monitor = GazeboMonitor(on_update=self._on_gz_update,
                                         world=CFG["gz_world"])
        self.gz_monitor.start()

        # 카메라 초기화
        self.camera = AnchorDetector(
            source=CFG["camera_source"],
            gz_topic=CFG["camera_topic"],
            v4l2_device=CFG["camera_device"],
        )
        self._cam_photo = None   # GC 방지용 참조
        if self.camera.ok:
            self._log(f"  📷 카메라 연결: {CFG['camera_source']}")
        else:
            self._log(f"  ℹ 카메라 없음 ({CFG['camera_source']})")
        self.root.after(200, self._cam_refresh)   # UI 준비 후 시작

        # UWB 로컬라이저 초기화
        # SITL: 기존 앵커를 시뮬 앵커로 사용 (거리 노이즈 시뮬레이션)
        # REAL: DWM3001C 시리얼 읽기 + anchor_ned_map으로 NED 매핑
        sitl_anchors = [(a["n"], a["e"], 0.0) for a in EXISTING_ANCHORS]
        # 실기체용 NED 매핑 (DWM3001C 앵커 hex ID → NED 좌표)
        # 실제 앵커 ID는 DWM3001C가 보고하는 4자리 hex 값으로 교체 필요
        anchor_ned_map = {str(a["id"]): (a["n"], a["e"], 0.0)
                          for a in EXISTING_ANCHORS}
        use_sitl_uwb = (MODE == "SITL") and CFG.get("uwb_sitl", True)
        self.uwb = UWBLocalizer(
            port=CFG["uwb_port"],
            baud=CFG["uwb_baud"],
            sitl=use_sitl_uwb,
            sitl_anchors=sitl_anchors,
            anchor_ned_map=anchor_ned_map,
        )
        self.uwb.start()
        if HAS_UWB:
            src = "SITL 시뮬" if use_sitl_uwb else CFG["uwb_port"]
            self._log(f"  📡 UWB 로컬라이저 시작 ({src})")
        self.root.after(500, self._uwb_refresh)

        # Vision 주입은 미션 시작 시 _vision_inject_task 로 실행됨
        # (MAVSDK mocap API → drone 연결 재사용, REAL/SITL 공통)
        self._log(f"  🔭 Vision 주입: MAVSDK mocap  {CFG['vision_rate_hz']}Hz"
                  f"  (미션 시작 시 활성화)")

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ══════════════════════════════════════════════════════════════════════
    #  UI 구성
    # ══════════════════════════════════════════════════════════════════════

    def _build_ui(self):
        # ── 왼쪽: 캔버스 영역 ────────────────────────────────────────────
        left = tk.Frame(self.root, bg=C["bg"])
        left.pack(side=tk.LEFT, padx=(10, 4), pady=10, fill=tk.BOTH, expand=True)

        tk.Label(left, text="UWB Anchor Map  v5",
                 bg=C["bg"], fg=C["accent"],
                 font=("Courier", 14, "bold")).pack(anchor="w")

        self.gz_status_lbl = tk.Label(left, text="● Gazebo 연결 대기...",
                                       bg=C["bg"], fg="#606060",
                                       font=("Courier", 9))
        self.gz_status_lbl.pack(anchor="w", pady=(0, 4))

        # 맵 캔버스 — 고정 크기 (expand 금지: CoordMapper 좌표 어긋남 방지)
        self.canvas = tk.Canvas(left, width=self.CW, height=self.CH,
                                 bg=C["bg"], highlightthickness=1,
                                 highlightbackground=C["border"])
        self.canvas.pack()   # fill/expand 제거 → 항상 CW×CH 고정

        self.mapper = CoordMapper(self.CW, self.CH)

        self.canvas.bind("<ButtonPress-1>",   self._on_press)
        self.canvas.bind("<B1-Motion>",       self._on_drag)
        self.canvas.bind("<ButtonRelease-1>", self._on_release)
        self.canvas.bind("<ButtonPress-3>",   self._on_set_home)   # 우클릭 = 홈 설정
        # 캔버스 밖에서 마우스를 놓아도 릴리즈 감지
        self.root.bind("<ButtonRelease-1>",   self._on_release_global)
        self.root.bind("<Escape>",            lambda _: self._stop_mission())

        self._draw_map()

        # ── 오른쪽: 스크롤 가능한 컨트롤 패널 ───────────────────────────
        right_outer = tk.Frame(self.root, bg=C["panel"], width=self.PW,
                                highlightthickness=1,
                                highlightbackground=C["border"])
        right_outer.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10), pady=10)
        right_outer.pack_propagate(False)

        # 스크롤 가능한 내부 패널 구성
        panel_canvas = tk.Canvas(right_outer, bg=C["panel"],
                                  highlightthickness=0, width=self.PW - 18)
        panel_scroll = tk.Scrollbar(right_outer, orient=tk.VERTICAL,
                                     command=panel_canvas.yview)
        panel_canvas.configure(yscrollcommand=panel_scroll.set)

        panel_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        panel_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # 실제 컨트롤이 들어갈 inner frame
        self._panel_inner = tk.Frame(panel_canvas, bg=C["panel"])
        self._panel_win_id = panel_canvas.create_window(
            (0, 0), window=self._panel_inner, anchor="nw")

        def _on_inner_configure(event):
            panel_canvas.configure(
                scrollregion=panel_canvas.bbox("all"))

        def _on_canvas_configure(event):
            panel_canvas.itemconfig(self._panel_win_id,
                                     width=event.width)

        self._panel_inner.bind("<Configure>", _on_inner_configure)
        panel_canvas.bind("<Configure>",      _on_canvas_configure)

        # 패널 내부 마우스 휠
        def _panel_wheel(event):
            if event.num == 4:
                panel_canvas.yview_scroll(-1, "units")
            elif event.num == 5:
                panel_canvas.yview_scroll(1, "units")
            else:
                panel_canvas.yview_scroll(
                    int(-1 * (event.delta / 120)), "units")

        panel_canvas.bind("<MouseWheel>", _panel_wheel)
        panel_canvas.bind("<Button-4>",   _panel_wheel)
        panel_canvas.bind("<Button-5>",   _panel_wheel)
        self._panel_inner.bind("<MouseWheel>", _panel_wheel)
        self._panel_inner.bind("<Button-4>",   _panel_wheel)
        self._panel_inner.bind("<Button-5>",   _panel_wheel)

        self._build_panel(self._panel_inner)

    def _sep(self, p):
        tk.Frame(p, bg=C["border"], height=1).pack(fill=tk.X, padx=8, pady=3)

    def _build_panel(self, p):
        pad = {"padx": 12, "pady": 3}

        tk.Label(p, text="CONTROL", bg=C["panel"], fg=C["accent"],
                 font=("Courier", 11, "bold")).pack(pady=(12, 2))
        self._sep(p)

        mode_col = "#52e0a0" if MODE == "SITL" else "#ff8844"
        tk.Label(p, text=f"MODE: {MODE}",
                 bg=C["panel"], fg=mode_col,
                 font=("Courier", 10, "bold")).pack(anchor="w", padx=12, pady=(0, 1))
        self.home_lbl = tk.Label(
            p, text=f"🏠  홈: N={self.drone_home[0]:.1f} E={self.drone_home[1]:.1f}  (우클릭=변경)",
            bg=C["panel"], fg=C["home"], font=("Courier", 8))
        self.home_lbl.pack(anchor="w", padx=12, pady=(0, 3))
        self._sep(p)

        self.drone_pos_lbl = tk.Label(p, text="🚁  드론: 대기",
                                       bg=C["panel"], fg=C["drone"],
                                       font=("Courier", 9), justify=tk.LEFT)
        self.drone_pos_lbl.pack(anchor="w", padx=12, pady=(0, 1))
        self.rover_pos_lbl = tk.Label(p, text="🚗  로버: 대기 (디포)",
                                       bg=C["panel"], fg=C["rover"],
                                       font=("Courier", 9), justify=tk.LEFT)
        self.rover_pos_lbl.pack(anchor="w", padx=12, pady=(0, 1))
        self.uwb_pos_lbl = tk.Label(p, text="📡 UWB: 대기",
                                     bg=C["panel"], fg=C["dim"],
                                     font=("Courier", 9), justify=tk.LEFT)
        self.uwb_pos_lbl.pack(anchor="w", padx=12, pady=(0, 2))
        self._sep(p)

        def _slider(label, var, lo, hi, res, fg, cmd):
            tk.Label(p, text=label, bg=C["panel"], fg=C["text"],
                     font=("Courier", 9)).pack(anchor="w", **pad)
            lbl = tk.Label(p, text=f"{var.get():.1f}" if isinstance(lo, float)
                           else f"{var.get()}",
                           bg=C["panel"], fg=fg,
                           font=("Courier", 13, "bold"))
            lbl.pack()
            tk.Scale(p, from_=lo, to=hi, resolution=res,
                     orient=tk.HORIZONTAL, variable=var,
                     command=lambda _: cmd(),
                     bg=C["panel"], fg=C["text"], troughcolor=C["grid"],
                     highlightthickness=0, showvalue=False,
                     length=200).pack(**pad)
            self._sep(p)
            return lbl

        self.lbl_spacing  = _slider("앵커 간격 (m)",        self.spacing_var,     1.0, 10.0, 0.5, C["accent"],  self._slider_changed)
        self.lbl_width    = _slider("좌우 폭 (m)",          self.width_var,       0.5, 10.0, 0.5, "#e0a052",    self._slider_changed)
        self.lbl_min_dist = _slider("기존앵커 배제반경 (m)", self.min_dist_var,    0.0, 30.0, 0.5, "#a070e0",    self._slider_changed)
        self.lbl_depot    = _slider("로버 초기 UWB 수량",   self.depot_stock_var, 0,   20,   1,   C["rover"],   self._slider_changed)

        tk.Checkbutton(p, text=f"커버리지 원 ({COVERAGE_RADIUS:.0f}m)",
                        variable=self.coverage_var, command=self._draw_map,
                        bg=C["panel"], fg=C["text"], selectcolor=C["bg"],
                        activebackground=C["panel"],
                        font=("Courier", 9)).pack(anchor="w", padx=12, pady=4)
        self._sep(p)

        tk.Label(p, text="범례", bg=C["panel"], fg=C["dim"],
                 font=("Courier", 9)).pack(anchor="w", padx=12)
        for color, label in [
            (C["gz_ok"],     "기존 앵커"),
            (C["relocated"], "재배치된 앵커(원위치)"),
            (C["new_anchor"],"설치 예정"),
            (C["installing"],"설치 중"),
            (C["rover"],     "로버(디포)"),
            ("#505060",      "배제됨"),
            (C["drone"],     "드론"),
        ]:
            row = tk.Frame(p, bg=C["panel"])
            row.pack(anchor="w", padx=14, pady=1)
            ic = tk.Canvas(row, width=18, height=12,
                           bg=C["panel"], highlightthickness=0)
            ic.pack(side=tk.LEFT)
            ic.create_oval(2, 2, 10, 10, fill=color, outline="")
            tk.Label(row, text=f" {label}", bg=C["panel"],
                     fg=C["text"], font=("Courier", 9)).pack(side=tk.LEFT)
        self._sep(p)

        self.info_var = tk.StringVar(value="선 없음")
        tk.Label(p, textvariable=self.info_var,
                 bg=C["panel"], fg=C["new_anchor"],
                 font=("Courier", 9), justify=tk.LEFT,
                 wraplength=210).pack(padx=12, pady=2)
        self._sep(p)

        self.btn_confirm = tk.Button(
            p, text="▶  드론 미션 시작", command=self._confirm,
            bg=C["btn_confirm"], fg="white",
            font=("Courier", 10, "bold"), relief="flat",
            cursor="hand2", pady=8, state=tk.DISABLED)
        self.btn_confirm.pack(fill=tk.X, padx=12, pady=3)

        self.btn_stop = tk.Button(
            p, text="⏹  미션 중단", command=self._stop_mission,
            bg="#4a1a1a", fg=C["text"],
            font=("Courier", 10), relief="flat",
            cursor="hand2", pady=6, state=tk.DISABLED)
        self.btn_stop.pack(fill=tk.X, padx=12, pady=2)

        tk.Button(p, text="⚡ 미확인 앵커 Gazebo 스폰",
                  command=self._spawn_missing,
                  bg=C["btn_spawn"], fg="white",
                  font=("Courier", 9), relief="flat",
                  cursor="hand2", pady=6).pack(fill=tk.X, padx=12, pady=2)

        tk.Button(p, text="✕  초기화", command=self._clear,
                  bg=C["btn_clear"], fg=C["text"],
                  font=("Courier", 9), relief="flat",
                  cursor="hand2", pady=5).pack(fill=tk.X, padx=12, pady=2)
        self._sep(p)

        self.gz_summary = tk.Label(p, text="Gazebo: 연결 대기",
                                    bg=C["panel"], fg=C["dim"],
                                    font=("Courier", 8), wraplength=210,
                                    justify=tk.LEFT)
        self.gz_summary.pack(padx=12, pady=2)

        # ── 카메라 뷰 ──────────────────────────────────────────────────────
        self._sep(p)
        cam_row = tk.Frame(p, bg=C["panel"])
        cam_row.pack(anchor="w", padx=12, pady=(2, 0))
        tk.Label(cam_row, text="📷 CAM", bg=C["panel"], fg=C["dim"],
                 font=("Courier", 8)).pack(side=tk.LEFT)
        self.cam_status_lbl = tk.Label(cam_row,
                                        text=f"({CFG['camera_source']})",
                                        bg=C["panel"], fg=C["dim"],
                                        font=("Courier", 8))
        self.cam_status_lbl.pack(side=tk.LEFT, padx=4)
        self.cam_canvas = tk.Canvas(p, width=216, height=162,
                                     bg="#050a0e", highlightthickness=1,
                                     highlightbackground=C["border"],
                                     cursor="hand2")
        self.cam_canvas.pack(padx=8, pady=(2, 2))
        self.cam_canvas.bind("<Button-1>", lambda _: self._open_cam_window())
        tk.Label(p, text="클릭 → 큰 창", bg=C["panel"], fg=C["dim"],
                 font=("Courier", 7)).pack(pady=(0, 2))
        tk.Button(p, text="📷 카메라 창 열기", command=self._open_cam_window,
                  bg=C["btn_spawn"], fg="white",
                  font=("Courier", 9), relief="flat",
                  cursor="hand2", pady=4).pack(fill=tk.X, padx=12, pady=2)
        self._sep(p)

        tk.Label(p, text="LOG", bg=C["panel"], fg=C["dim"],
                 font=("Courier", 8)).pack(anchor="w", padx=12, pady=(6, 0))
        self.log = scrolledtext.ScrolledText(
            p, width=28, height=10,
            bg=C["log_bg"], fg=C["dim"],
            font=("Courier", 8), relief="flat", state=tk.DISABLED)
        self.log.pack(padx=8, pady=(0, 12), fill=tk.BOTH, expand=True)
        self.log.tag_configure("ok",   foreground="#52e0a0")
        self.log.tag_configure("warn", foreground="#ffd060")
        self.log.tag_configure("err",  foreground="#e05252")
        self.log.tag_configure("info", foreground=C["dim"])

    # ══════════════════════════════════════════════════════════════════════
    #  Gazebo 콜백
    # ══════════════════════════════════════════════════════════════════════

    def _on_gz_update(self, models: set):
        self.root.after(0, lambda: self._apply_gz(models))

    def _apply_gz(self, models: set):
        prev = self.gz_models
        self.gz_models = models
        names   = {f"uwb_anchor_{a['id']}" for a in EXISTING_ANCHORS}
        found   = names & models
        missing = names - models
        if self.gz_monitor.connected:
            gz_uwb = sorted(m for m in models if m.startswith("uwb_anchor_"))
            self.gz_status_lbl.config(
                text=f"● Gazebo 연결됨 | 기존앵커 {len(found)}/{len(EXISTING_ANCHORS)}",
                fg="#52e052" if not missing else "#e0a052")
            self.gz_summary.config(
                text=f"UWB 모델 {len(gz_uwb)}개:\n" +
                     (", ".join(gz_uwb) if gz_uwb else "없음"))
        else:
            self.gz_status_lbl.config(text="● Gazebo 미연결", fg="#e05050")
            self.gz_summary.config(text="Gazebo: 오프라인")
        newly_done = self.installing & models
        if newly_done:
            self.installed_new |= newly_done
            self.installing    -= newly_done
            for nm in newly_done:
                self._log(f"  ✅ Gazebo 확인: {nm}")
        if models != prev:
            self._draw_map()

    # ══════════════════════════════════════════════════════════════════════
    #  맵 렌더링
    # ══════════════════════════════════════════════════════════════════════

    def _draw_map(self):
        self.canvas.delete("all")
        m        = self.mapper
        show_cov = self.coverage_var.get()
        r_px     = m.m2px(COVERAGE_RADIUS)

        # 1) 그리드
        for axis in ("N", "E"):
            v    = MAP_N[0] if axis == "N" else MAP_E[0]
            vmax = MAP_N[1] if axis == "N" else MAP_E[1]
            while v <= vmax + 0.01:
                if axis == "N":
                    x0, y0 = m.w2c(v, MAP_E[0]); x1, y1 = m.w2c(v, MAP_E[1])
                else:
                    x0, y0 = m.w2c(MAP_N[0], v); x1, y1 = m.w2c(MAP_N[1], v)
                self.canvas.create_line(x0, y0, x1, y1, fill=C["grid"])
                if v == int(v) and int(v) % 5 == 0:
                    if axis == "N":
                        self.canvas.create_text(x0-20, y0, text=str(int(v)),
                                                 fill=C["dim"], font=("Courier", 8))
                    else:
                        self.canvas.create_text(x0, y0+14, text=str(int(v)),
                                                 fill=C["dim"], font=("Courier", 8))
                v = round(v + 2, 1)

        # 2) 커버리지 원
        if show_cov:
            for a in EXISTING_ANCHORS:
                cx, cy = m.w2c(a["n"], a["e"])
                self.canvas.create_oval(cx-r_px, cy-r_px, cx+r_px, cy+r_px,
                                         outline=C["cov_gz"], fill=C["cov_gz"],
                                         stipple="gray12", width=1)
            for (n, e) in self.preview:
                cx, cy = m.w2c(n, e)
                self.canvas.create_oval(cx-r_px, cy-r_px, cx+r_px, cy+r_px,
                                         outline=C["cov_new"], fill=C["cov_new"],
                                         stipple="gray12", width=1)

        # 3) 로버 지나간 구간
        self._draw_rover_zone()

        # 4) 홈 마커
        hx, hy = m.w2c(*self.drone_home)
        self.canvas.create_oval(hx-10, hy-10, hx+10, hy+10,
                                 outline=C["home"], width=2, fill=C["bg"])
        self.canvas.create_text(hx, hy, text="H",
                                 fill=C["home"], font=("Courier", 9, "bold"))
        self.canvas.create_text(hx, hy+16, text="우클릭=홈",
                                 fill=C["dim"], font=("Courier", 7))

        # 5) 드래그 경로선 + 코리더 경계
        if self.line_start and self.line_end:
            self.canvas.create_line(
                self.line_start[0], self.line_start[1],
                self.line_end[0],   self.line_end[1],
                fill=C["path_line"], width=1, dash=(6, 4))
            n0, e0 = m.c2w(*self.line_start)
            n1, e1 = m.c2w(*self.line_end)
            total  = math.hypot(n1-n0, e1-e0)
            if total > 0.1:
                dn, de = (n1-n0)/total, (e1-e0)/total
                pn, pe = -de, dn
                w = self.width_var.get()
                for side in (+1, -1):
                    lx0, ly0 = m.w2c(n0+side*w*pn, e0+side*w*pe)
                    lx1, ly1 = m.w2c(n1+side*w*pn, e1+side*w*pe)
                    self.canvas.create_line(lx0, ly0, lx1, ly1,
                                             fill=C["path_border"], width=1, dash=(3,5))

        # 6) 지그재그 연결선
        if len(self.preview) >= 2:
            pts = [m.w2c(n, e) for (n, e) in self.preview]
            for i in range(len(pts)-1):
                self.canvas.create_line(pts[i][0], pts[i][1],
                                         pts[i+1][0], pts[i+1][1],
                                         fill=C["new_anchor"], width=1, dash=(4,4))

        # 7) 재배치 화살표
        self._draw_reloc_arrows()

        # 8) 기존 앵커 (UWB 건강 상태 색상 반영)
        anch_health = self.uwb.get_quality().get("anchor_health", {})
        for a in EXISTING_ANCHORS:
            cx, cy = m.w2c(a["n"], a["e"])
            r = 9
            if a["id"] in self.relocated_existing_ids:
                self.canvas.create_oval(cx-r, cy-r, cx+r, cy+r,
                                         fill=C["relocated"], outline="#909090",
                                         width=1, stipple="gray50")
                self.canvas.create_text(cx, cy, text="↗",
                                         fill="#b0b8c0", font=("Courier", 10, "bold"))
                self.canvas.create_text(cx, cy-16, text=f"A{a['id']}→",
                                         fill=C["relocated"], font=("Courier", 8))
            else:
                col = C["gz_ok"]  # 기본: 빨강
                # SIM ID로 건강 상태 매핑 (SITL: SIM0, SIM1, ...)
                sim_id = f"SIM{list(EXISTING_ANCHORS).index(a)}"
                h = anch_health.get(sim_id, {})
                if h:
                    if not h.get("ok", True):
                        # NLOS/오배치 의심: 주황색 + 경고 링
                        col = "#ff8844"
                        self.canvas.create_oval(cx-r-4, cy-r-4, cx+r+4, cy+r+4,
                                                 outline="#ff4400", width=1,
                                                 dash=(3, 2))
                self.canvas.create_oval(cx-r, cy-r, cx+r, cy+r,
                                         fill=col, outline="white", width=1)
                self.canvas.create_text(cx, cy, text=str(a["id"]),
                                         fill="white", font=("Courier", 8, "bold"))
                lbl = f"A{a['id']}"
                if h and not h.get("ok", True):
                    lbl += "⚠"
                self.canvas.create_text(cx, cy-16, text=lbl,
                                         fill=col, font=("Courier", 8))

        # 9) 신규 앵커 (설치 완료 앵커에 건강 상태 표시)
        reloc_pos_set = set(self.reloc_new_positions)
        n_existing = len(EXISTING_ANCHORS)
        for i, (n, e) in enumerate(self.preview):
            cx, cy = m.w2c(n, e)
            r   = 8
            aid = n_existing + i + 1
            nm  = f"uwb_anchor_{aid}"

            if (n, e) in reloc_pos_set:
                s = 6
                self.canvas.create_line(cx-s, cy-s, cx+s, cy+s,
                                         fill=C["relocated"], width=1)
                self.canvas.create_line(cx-s, cy+s, cx+s, cy-s,
                                         fill=C["relocated"], width=1)
                continue

            if nm in self.installed_new:
                col = C["gz_ok"]
                lbl = "✓"
                # SITL 인덱스: 기존 앵커 다음부터
                sim_id = f"SIM{n_existing + i}"
                h = anch_health.get(sim_id, {})
                if h and not h.get("ok", True):
                    col = "#ff8844"
                    lbl = "⚠"
                    self.canvas.create_oval(cx-r-4, cy-r-4, cx+r+4, cy+r+4,
                                             outline="#ff4400", width=1,
                                             dash=(3, 2))
            elif nm in self.installing:
                col, lbl = C["installing"], "↓"
            else:
                col, lbl = C["new_anchor"], "L" if i % 2 == 0 else "R"

            self.canvas.create_oval(cx-r, cy-r, cx+r, cy+r,
                                     fill=col, outline="white", width=1)
            self.canvas.create_text(cx, cy, text=lbl,
                                     fill=C["bg"], font=("Courier", 8, "bold"))

        # 10) 배제 후보 ×
        for (n, e) in self.preview_blocked:
            bx, by = m.w2c(n, e)
            s = 6
            self.canvas.create_line(bx-s, by-s, bx+s, by+s, fill="#505060", width=1)
            self.canvas.create_line(bx-s, by+s, bx+s, by-s, fill="#505060", width=1)

        # 11) 로버
        self._draw_rover()

        # 12) 드론
        self._draw_drone()

        # 13) 커버리지 텍스트
        if show_cov:
            self.canvas.create_text(self.CW-8, self.CH-8,
                                     text=f"커버리지 {COVERAGE_RADIUS:.0f} m",
                                     anchor="se", fill=C["dim"], font=("Courier", 8))

    def _draw_rover_zone(self):
        if not self.line_start or not self.line_end:
            return
        if self.rover_t <= 0.01:
            return
        m  = self.mapper
        n0, e0 = self.path_start
        n1, e1 = self.path_end
        rn = n0 + self.rover_t * (n1 - n0)
        re = e0 + self.rover_t * (e1 - e0)
        sx, sy = m.w2c(n0, e0)
        rx, ry = m.w2c(rn, re)
        self.canvas.create_line(sx, sy, rx, ry, fill=C["rover_zone"], width=10)
        mx, my = (sx+rx)/2, (sy+ry)/2
        self.canvas.create_text(mx+8, my-10, text="지나간 구간",
                                 fill=C["dim"], font=("Courier", 8), anchor="w")

    def _draw_reloc_arrows(self):
        m = self.mapper
        for step in self.mission_steps:
            if step["type"] != "relocate":
                continue
            src = step["source"]
            dst = step["anchor"]
            sx, sy = m.w2c(src["n"], src["e"])
            dx, dy = m.w2c(dst["n"], dst["e"])
            self.canvas.create_line(sx, sy, dx, dy,
                                     fill=C["reloc_arrow"], width=1,
                                     dash=(4, 3), arrow=tk.LAST,
                                     arrowshape=(9, 11, 4))
            r = 13
            self.canvas.create_oval(sx-r, sy-r, sx+r, sy+r,
                                     outline=C["reloc_src_hl"], width=1,
                                     dash=(3, 2))

    def _draw_rover(self):
        m = self.mapper
        for pt in self.rover_trail:
            tx, ty = m.w2c(pt[0], pt[1])
            self.canvas.create_oval(tx-2, ty-2, tx+2, ty+2,
                                     fill=C["rover_trail"], outline="")
        if self.rover_pos:
            rn, re = self.rover_pos
        elif self.line_start:
            rn, re = self.path_start
        else:
            return
        rx, ry = m.w2c(rn, re)
        r = 10
        self.canvas.create_rectangle(rx-r, ry-r, rx+r, ry+r,
                                      fill=C["rover"], outline="white", width=1)
        self.canvas.create_text(rx, ry, text="R",
                                 fill=C["bg"], font=("Courier", 9, "bold"))
        self.canvas.create_text(rx, ry-r-10,
                                 text=f"디포 {self.depot_remaining}",
                                 fill=C["rover"], font=("Courier", 8))

    def _draw_drone(self):
        if not self.drone_pos:
            return
        m = self.mapper
        dn, de, dalt = self.drone_pos
        for pt in self.drone_trail:
            tx, ty = m.w2c(pt[0], pt[1])
            self.canvas.create_oval(tx-2, ty-2, tx+2, ty+2,
                                     fill=C["drone_trail"], outline="")
        cx, cy = m.w2c(dn, de)
        r = 8
        # UWB EKF 위치 불확실도 타원 (3σ)
        qual = self.uwb.get_quality()
        sn, se, _ = qual["sigma"]
        if sn < 5. and se < 5.:   # 합리적인 범위일 때만 표시
            rx = int(m.m2px(se * 3))
            ry = int(m.m2px(sn * 3))
            if rx > 2 and ry > 2:
                self.canvas.create_oval(cx-rx, cy-ry, cx+rx, cy+ry,
                                         outline="#2a4a8a", width=1,
                                         dash=(4, 3))
        self.canvas.create_oval(cx-r, cy-r, cx+r, cy+r,
                                 outline=C["drone"], width=2, fill="")
        self.canvas.create_line(cx-r-3, cy, cx+r+3, cy, fill=C["drone"], width=1)
        self.canvas.create_line(cx, cy-r-3, cx, cy+r+3, fill=C["drone"], width=1)
        self.canvas.create_text(cx, cy-r-14,
                                 text=f"🚁 ({dn:.1f},{de:.1f}) {abs(dalt):.1f}m",
                                 fill=C["drone"], font=("Courier", 8, "bold"))

    # ══════════════════════════════════════════════════════════════════════
    #  위치 업데이트
    # ══════════════════════════════════════════════════════════════════════

    def _update_drone_pos(self, n, e, alt):
        self.drone_pos = (n, e, alt)
        self.drone_trail.append((n, e))
        if len(self.drone_trail) > self.TRAIL_MAX:
            self.drone_trail.pop(0)
        # 더미 시뮬(MAVSDK 없음) + SITL UWB: 애니메이션 위치로 거리 시뮬 갱신
        # alt 는 양수(위 방향), NED에서 위 = 음수이므로 -alt 변환
        if MODE == "SITL":
            self.uwb.set_sitl_drone_pos(n, e, -alt)
        # 더미 시뮬에서도 baro 고도 보정 (EKF Z 발산 방지)
        self.uwb.set_baro_altitude(alt)
        self.drone_pos_lbl.config(
            text=f"🚁  N:{n:+.1f}  E:{e:+.1f}  Alt:{alt:.1f}m")
        self._draw_map()

    def _update_rover_pos(self, t: float):
        n0, e0 = self.path_start
        n1, e1 = self.path_end
        self.rover_t   = min(max(t, 0.0), 1.0)
        rn = n0 + self.rover_t * (n1 - n0)
        re = e0 + self.rover_t * (e1 - e0)
        self.rover_pos = (rn, re)
        self.rover_trail.append((rn, re))
        if len(self.rover_trail) > self.TRAIL_MAX:
            self.rover_trail.pop(0)
        self.rover_pos_lbl.config(
            text=f"🚗  N:{rn:+.1f}  E:{re:+.1f}  디포:{self.depot_remaining}개")
        self._draw_map()

    def _clear_drone_pos(self):
        self.drone_pos = None; self.drone_trail = []
        self.drone_pos_lbl.config(text="🚁  드론: 대기")
        self._draw_map()

    def _clear_rover(self):
        self.rover_pos = None; self.rover_t = 0.0; self.rover_trail = []
        self.rover_pos_lbl.config(text="🚗  로버: 대기 (디포)")
        self._draw_map()

    # ══════════════════════════════════════════════════════════════════════
    #  마우스 이벤트
    # ══════════════════════════════════════════════════════════════════════

    def _on_set_home(self, ev):
        n, e = self.mapper.c2w(ev.x, ev.y)
        self.drone_home = [n, e]
        self.home_lbl.config(
            text=f"🏠  홈: N={n:.1f} E={e:.1f}  (우클릭=변경)")
        self._draw_map()
        self._log(f"  🏠 홈 위치 설정: N={n:.1f} E={e:.1f}")

    def _on_press(self, ev):
        self.line_start = (ev.x, ev.y)
        self.line_end   = None
        self.preview    = []
        self.btn_confirm.config(state=tk.DISABLED)
        self._draw_map()

    def _on_drag(self, ev):
        self.line_end = (ev.x, ev.y)
        self._update_preview()
        # 드래그 중 미리보기가 생기면 즉시 버튼 활성화
        self.btn_confirm.config(state=tk.NORMAL if self.preview else tk.DISABLED)

    def _on_release(self, ev):
        self.line_end = (ev.x, ev.y)
        self._update_preview()
        self.btn_confirm.config(state=tk.NORMAL if self.preview else tk.DISABLED)

    def _on_release_global(self, ev):
        """캔버스 밖에서 마우스를 놓아도 버튼 상태 갱신."""
        if ev.widget is not self.canvas and self.line_start is not None:
            self.btn_confirm.config(state=tk.NORMAL if self.preview else tk.DISABLED)

    def _update_preview(self):
        if not (self.line_start and self.line_end):
            return
        n0, e0 = self.mapper.c2w(*self.line_start)
        n1, e1 = self.mapper.c2w(*self.line_end)
        self.path_start = (n0, e0)
        self.path_end   = (n1, e1)

        accepted, blocked = smart_place_anchors(
            n0, e0, n1, e1,
            self.spacing_var.get(), self.width_var.get(),
            EXISTING_ANCHORS, self.min_dist_var.get())
        self.preview         = accepted
        self.preview_blocked = blocked

        # 미리보기용 임시 ID (충돌 위험 없음, 실제 미션 시 _alloc_anchor_id 재배정)
        anchors_tmp = [
            {"id": len(EXISTING_ANCHORS)+i+1, "n": n, "e": e,
             "name": f"uwb_anchor_preview_{i}"}
            for i, (n, e) in enumerate(self.preview)]
        self.mission_steps = plan_mission(
            anchors_tmp, self.depot_stock_var.get(),
            n0, e0, n1, e1, EXISTING_ANCHORS)

        self._draw_map()

        L     = sum(1 for i in range(len(self.preview)) if i % 2 == 0)
        R     = len(self.preview) - L
        ndep  = sum(1 for s in self.mission_steps if s["type"] == "from_depot")
        nrel  = sum(1 for s in self.mission_steps if s["type"] == "relocate")
        nskip = sum(1 for s in self.mission_steps if s["type"] == "skip")
        dist  = math.hypot(n1-n0, e1-e0)
        self.info_var.set(
            f"경로: {dist:.1f} m\n"
            f"채택: {len(self.preview)}개  (L:{L} R:{R})\n"
            f"배제: {len(blocked)}개\n"
            f"디포픽업: {ndep}  재배치: {nrel}\n"
            + (f"⚠ 미설치: {nskip}개\n" if nskip else "")
            + f"간격 {self.spacing_var.get():.1f}m  폭 {self.width_var.get():.1f}m")

    def _slider_changed(self):
        self.lbl_spacing.config(text=f"{self.spacing_var.get():.1f} m")
        self.lbl_width.config(text=f"{self.width_var.get():.1f} m")
        self.lbl_min_dist.config(text=f"{self.min_dist_var.get():.1f} m")
        self.lbl_depot.config(text=f"{self.depot_stock_var.get()} 개")
        self.depot_remaining = self.depot_stock_var.get()
        self._update_preview()

    # ══════════════════════════════════════════════════════════════════════
    #  버튼 콜백
    # ══════════════════════════════════════════════════════════════════════

    def _stop_mission(self):
        self._stop_evt.set()
        self._log("⏹ 중단 요청됨...")
        self.btn_stop.config(state=tk.DISABLED)

    def _spawn_missing(self):
        missing = [a for a in EXISTING_ANCHORS
                   if f"uwb_anchor_{a['id']}" not in self.gz_models]
        if not missing:
            self._log("  ℹ 모든 기존 앵커 Gazebo 확인됨"); return
        self._log(f"  ⚡ {len(missing)}개 스폰 시도...")
        def _do():
            for a in missing:
                GazeboMonitor.spawn(dict(a, name=f"uwb_anchor_{a['id']}"),
                                    world=CFG["gz_world"])
                self._log(f"    → uwb_anchor_{a['id']} 스폰")
        threading.Thread(target=_do, daemon=True).start()

    def _clear(self):
        # 진행 중인 미션 중단
        self._stop_evt.set()
        self.btn_stop.config(state=tk.DISABLED)
        self.btn_confirm.config(state=tk.DISABLED)

        # Gazebo에 실제로 존재하는 앵커만 삭제
        to_despawn = (self.installed_new | self.installing) & self.gz_models
        if to_despawn:
            world = CFG["gz_world"]
            def _do():
                for nm in to_despawn:
                    GazeboMonitor.despawn(nm, world=world)
                    self._log(f"  🗑 Gazebo 삭제: {nm}")
            threading.Thread(target=_do, daemon=True).start()

        self.line_start = self.line_end = None
        self.preview = []; self.preview_blocked = []
        self.installing = set(); self.installed_new = set()
        self.info_var.set("선 없음")
        self.relocated_existing_ids = set()
        self.reloc_new_positions    = []
        self.mission_steps          = []
        self.depot_remaining        = self.depot_stock_var.get()
        self._clear_drone_pos(); self._clear_rover()
        self._draw_map()

    def _confirm(self):
        if not self.preview:
            return
        n0, e0 = self.path_start; n1, e1 = self.path_end
        # 미션 시작 시 유니크 ID 배정 (재실행/재배치 후 충돌 방지)
        def _make_anchor(n, e):
            aid = _alloc_anchor_id()
            return {"id": aid, "n": n, "e": e, "name": f"uwb_anchor_{aid}"}
        anchors = [_make_anchor(n, e) for (n, e) in self.preview]
        steps = plan_mission(anchors, self.depot_stock_var.get(),
                              n0, e0, n1, e1, EXISTING_ANCHORS)

        self.depot_remaining        = self.depot_stock_var.get()
        self.relocated_existing_ids = set()
        self.reloc_new_positions    = []
        self.mission_steps          = steps
        self.rover_trail            = []
        self.installing             = set()
        self.installed_new          = set()

        ndep  = sum(1 for s in steps if s["type"] == "from_depot")
        nrel  = sum(1 for s in steps if s["type"] == "relocate")
        nskip = sum(1 for s in steps if s["type"] == "skip")
        self._stop_evt.clear()
        self.btn_confirm.config(state=tk.DISABLED)
        self.btn_stop.config(state=tk.NORMAL)
        self._log("▶ 미션 시작")
        self._log(f"  총 {len(steps)}스텝 | 디포픽업 {ndep} | 재배치 {nrel} | 건너뜀 {nskip}")
        threading.Thread(target=self._run_mission, args=(steps,),
                          daemon=True).start()

    # ══════════════════════════════════════════════════════════════════════
    #  미션 실행
    # ══════════════════════════════════════════════════════════════════════

    def _run_mission(self, steps):
        if not HAS_MAVSDK:
            self._log("⚠ mavsdk 없음 → 더미 시뮬레이션")
            self._run_dummy_mission(steps)
        elif MODE == "SITL":
            asyncio.run(self._mission_coro(steps, address=CFG["address"]))
        else:  # REAL
            asyncio.run(self._mission_coro(steps, address=CFG["real_address"]))

    def _run_dummy_mission(self, steps: list):
        self._stop_evt.clear()   # 방어용: 이전 중단 이벤트 잔류 방지
        self._log("  [SIM] 더미 시뮬레이션 모드")
        n_home, e_home = self.drone_home
        flight_alt     = abs(CFG["flight_alt"])
        drop_alt       = abs(CFG["drop_alt"])
        N = len(steps)

        ndep  = sum(1 for s in steps if s["type"] == "from_depot")
        nrel  = sum(1 for s in steps if s["type"] == "relocate")
        nskip = sum(1 for s in steps if s["type"] == "skip")
        self._log(f"  스텝: 디포={ndep} 재배치={nrel} 건너뜀={nskip}")

        self._log("  [SIM] 이륙...")
        self.root.after(0, lambda: self._update_rover_pos(0.0))
        for i in range(11):
            if self._stop_evt.is_set():
                self._log("⏹ [SIM] 이륙 중 중단됨")
                self.root.after(0, self._clear_drone_pos)
                self.root.after(0, lambda: self.btn_confirm.config(state=tk.NORMAL))
                self.root.after(0, lambda: self.btn_stop.config(state=tk.DISABLED))
                return
            alt = flight_alt * (i / 10)
            self.root.after(0, lambda n=n_home, e=e_home, a=alt:
                            self._update_drone_pos(n, e, a))
            time.sleep(0.12)

        prev_n, prev_e = n_home, e_home

        for idx, step in enumerate(steps):
            if self._stop_evt.is_set():
                self._log("⏹ [SIM] 미션 중단됨"); break
            atype  = step["type"]
            anchor = step["anchor"]
            t_n, t_e  = anchor["n"], anchor["e"]
            rover_t   = step.get("rover_t", 0.0)

            if atype == "skip":
                self._log(f"\n⚠ [{idx+1}/{N}] {anchor['name']} — {step.get('reason','')}")
                self.root.after(0, lambda t=rover_t: self._update_rover_pos(t))
                continue

            self._start_rover_move(rover_t)

            if atype == "from_depot":
                pick_n, pick_e = step["rover_pos"]
                self._log(f"\n─ [{idx+1}/{N}] 디포 픽업 @ 로버({pick_n:.1f},{pick_e:.1f})")
                prev_n, prev_e = self._sim_fly_pick(
                    prev_n, prev_e, pick_n, pick_e,
                    flight_alt, drop_alt, label="로버에서 집기")
                self.root.after(0, self._depot_consume)

            elif atype == "relocate":
                src   = step["source"]
                stype = step["source_type"]
                src_n, src_e = src["n"], src["e"]
                src_label = (f"A{src['id']}" if "id" in src
                             else f"신규({src_n:.1f},{src_e:.1f})")
                self._log(f"\n─ [{idx+1}/{N}] 재배치: {src_label} "
                           f"({src_n:.1f},{src_e:.1f}) → ({t_n:.1f},{t_e:.1f})")
                self._log(f"    [이유] 로버 뒤쪽 + 끝점에서 가장 멀리 있는 앵커")

                if stype == "existing":
                    def on_grab(sid=src["id"]):
                        self.root.after(0, lambda: self._mark_relocated_existing(sid))
                        GazeboMonitor.despawn(f"uwb_anchor_{sid}", world=CFG["gz_world"])
                else:
                    def on_grab(sn=src_n, se=src_e, nm=src["name"]):
                        self.root.after(0, lambda: self._mark_relocated_new(sn, se))
                        GazeboMonitor.despawn(nm, world=CFG["gz_world"])
                        self.root.after(0, lambda: self._remove_from_installed(nm))

                prev_n, prev_e = self._sim_fly_pick(
                    prev_n, prev_e, src_n, src_e,
                    flight_alt, drop_alt, label=f"{src_label} 집기",
                    on_grab=on_grab)

            self._log(f"  → 설치 @ ({t_n:.1f},{t_e:.1f})")
            self.root.after(0, lambda nm=anchor["name"]: self._mark_installing(nm))
            prev_n, prev_e = self._sim_fly_drop(
                prev_n, prev_e, t_n, t_e, flight_alt, drop_alt, anchor)

        if not self._stop_evt.is_set():
            self._log("\n[SIM] 홈 복귀...")
            self._sim_move(prev_n, prev_e, n_home, e_home, flight_alt, steps=21)
            for i in range(11):
                if self._stop_evt.is_set():
                    break
                alt = flight_alt * (1 - i/10)
                self.root.after(0, lambda n=n_home, e=e_home, a=alt:
                                self._update_drone_pos(n, e, a))
                time.sleep(0.12)

        self._log("✅ [SIM] 임무 완료!" if not self._stop_evt.is_set() else "⏹ [SIM] 중단됨")
        self.root.after(0, self._clear_drone_pos)
        self.root.after(0, lambda: self.btn_confirm.config(state=tk.NORMAL))
        self.root.after(0, lambda: self.btn_stop.config(state=tk.DISABLED))

    def _sim_move(self, n0, e0, n1, e1, alt, steps=20, delay=0.09):
        for i in range(steps + 1):
            if self._stop_evt.is_set():
                return
            t  = i / steps
            cn = n0 + t * (n1-n0); ce = e0 + t * (e1-e0)
            self.root.after(0, lambda n=cn, e=ce, a=alt:
                            self._update_drone_pos(n, e, a))
            time.sleep(delay)

    def _start_rover_move(self, target_t: float):
        self._rover_gen += 1
        gen = self._rover_gen
        threading.Thread(target=self._sim_move_rover_to,
                         args=(target_t, gen), daemon=True).start()

    def _sim_move_rover_to(self, target_t: float, gen: int = 0,
                           steps: int = 20, delay: float = 0.09):
        start_t = self.rover_t
        for i in range(steps + 1):
            if self._rover_gen != gen:
                break
            t = start_t + (i / steps) * (target_t - start_t)
            self.root.after(0, lambda tt=t: self._update_rover_pos(tt))
            time.sleep(delay)

    def _sim_descend_ascend(self, n, e, flight_alt, drop_alt, on_grab=None):
        for i in range(11):
            if self._stop_evt.is_set():
                return
            alt = flight_alt + (i/10) * (drop_alt - flight_alt)
            self.root.after(0, lambda nn=n, ee=e, a=alt:
                            self._update_drone_pos(nn, ee, a))
            time.sleep(0.07)
        if on_grab:
            on_grab()
        time.sleep(0.35)
        for i in range(11):
            if self._stop_evt.is_set():
                return
            alt = drop_alt + (i/10) * (flight_alt - drop_alt)
            self.root.after(0, lambda nn=n, ee=e, a=alt:
                            self._update_drone_pos(nn, ee, a))
            time.sleep(0.07)

    def _sim_fly_pick(self, n0, e0, dest_n, dest_e,
                       flight_alt, drop_alt, label="집기", on_grab=None):
        self._sim_move(n0, e0, dest_n, dest_e, flight_alt)
        self._log(f"  [SIM] {label}...")
        self._sim_descend_ascend(dest_n, dest_e, flight_alt, drop_alt, on_grab=on_grab)
        return dest_n, dest_e

    def _sim_fly_drop(self, n0, e0, dest_n, dest_e,
                       flight_alt, drop_alt, anchor):
        self._sim_move(n0, e0, dest_n, dest_e, flight_alt)
        for i in range(11):
            if self._stop_evt.is_set():
                return dest_n, dest_e
            alt = flight_alt + (i/10) * (drop_alt - flight_alt)
            self.root.after(0, lambda n=dest_n, e=dest_e, a=alt:
                            self._update_drone_pos(n, e, a))
            time.sleep(0.07)
        self._log(f"  [SIM] {anchor['name']} 투하!")
        time.sleep(0.35)
        GazeboMonitor.spawn(anchor, world=CFG["gz_world"])
        self.root.after(0, lambda nm=anchor["name"]: self._mark_installed(nm))
        # UWB SITL: 신규 앵커 동적 등록 (투하 위치 오차 포함)
        if MODE == "SITL":
            self.uwb.add_sitl_anchor(dest_n, dest_e, 0.)
        for i in range(11):
            if self._stop_evt.is_set():
                return dest_n, dest_e
            alt = drop_alt + (i/10) * (flight_alt - drop_alt)
            self.root.after(0, lambda n=dest_n, e=dest_e, a=alt:
                            self._update_drone_pos(n, e, a))
            time.sleep(0.07)
        return dest_n, dest_e

    def _mark_installing(self, name: str):
        self.installing.add(name); self._draw_map()

    def _mark_installed(self, name: str):
        """Gazebo 폴링 없이 즉시 installed 상태로 전환 (SIM/오프라인용)."""
        self.installing.discard(name)
        self.installed_new.add(name)
        self._draw_map()

    def _mark_relocated_existing(self, anchor_id: int):
        self.relocated_existing_ids.add(anchor_id); self._draw_map()

    def _mark_relocated_new(self, orig_n: float, orig_e: float):
        self.reloc_new_positions.append((orig_n, orig_e)); self._draw_map()

    def _remove_from_installed(self, name: str):
        self.installed_new.discard(name); self._draw_map()

    def _depot_consume(self):
        if self.depot_remaining > 0:
            self.depot_remaining -= 1
        rn, re = self.rover_pos if self.rover_pos else (0, 0)
        self.rover_pos_lbl.config(
            text=f"🚗  N:{rn:+.1f}  E:{re:+.1f}  디포:{self.depot_remaining}개")
        self._draw_map()

    # ── REAL 모드 비행 헬퍼 ──────────────────────────────────────────────────

    async def _wait_reach(self, drone, n, e, tol=0.5, timeout=20.0):
        """
        목표 NED 좌표에 tol 미터 이내로 도달할 때까지 대기.
        REAL 모드(GPS 없음): UWB 위치 기반.
        SITL 모드: PX4 텔레메트리 기반.
        """
        if HAS_UWB:
            # SITL/REAL 모두 UWB 위치로 도달 확인 (GPS 불필요)
            t0 = time.monotonic()
            while time.monotonic() - t0 < timeout:
                pos = self.uwb.get_position()
                if pos:
                    dist = math.hypot(pos[0] - n, pos[1] - e)
                    if dist < tol:
                        return
                await asyncio.sleep(0.1)
            self._log(f"  ⚠ UWB 이동 타임아웃 ({timeout:.0f}s) — 계속 진행")
            return
        # UWB 없을 때 fallback: PX4 텔레메트리
        t0 = asyncio.get_running_loop().time()
        async for pv in drone.telemetry.position_velocity_ned():
            dist = math.hypot(pv.position.north_m - n, pv.position.east_m - e)
            if dist < tol:
                break
            if asyncio.get_running_loop().time() - t0 > timeout:
                self._log(f"  ⚠ 이동 타임아웃 ({timeout:.0f}s) — 계속 진행")
                break

    async def _cam_align(self, drone, cur_n, cur_e, alt: float, timeout=8.0):
        """
        카메라로 앵커를 찾아 드론을 정렬.
        카메라 없거나 감지 실패 시 조용히 통과.
        반환: (정렬된 n, 정렬된 e)
        """
        if not self.camera.ok:
            self._log("  ⚠ 카메라 없음 — UWB 위치로만 픽업 진행", "warn")
            return cur_n, cur_e
        self._log("  📷 카메라 정렬 시도...")
        t0  = asyncio.get_running_loop().time()
        n, e = cur_n, cur_e
        while asyncio.get_running_loop().time() - t0 < timeout:
            frame = self.camera.get_frame()
            det   = self.camera.detect_anchor(frame)
            if det is None:
                await asyncio.sleep(0.1)
                continue
            cx, cy, iw, ih = det
            dn, de = AnchorDetector.pixel_to_ned_offset(
                cx, cy, iw, ih, abs(alt), CFG["camera_fov"])
            if abs(dn) < CFG["cam_align_tol"] and abs(de) < CFG["cam_align_tol"]:
                self._log(f"  📷 정렬 완료: 오프셋 Δn={dn:.2f} Δe={de:.2f}")
                return n, e
            # 오프셋 보정 이동
            n += dn; e += de
            await drone.offboard.set_position_ned(
                PositionNedYaw(n, e, alt, 0.))
            await asyncio.sleep(CFG["cam_align_wait"])
        self._log("  ⚠ 카메라 정렬 타임아웃 — 추정 위치로 픽업")
        return n, e

    async def _real_fly_pick(self, drone, dest_n, dest_e, on_grab=None):
        await drone.offboard.set_position_ned(
            PositionNedYaw(dest_n, dest_e, CFG["flight_alt"], 0.))
        await self._wait_reach(drone, dest_n, dest_e)
        # 중간 고도에서 카메라 정렬 (앵커 위 정확히 위치)
        mid_alt = CFG["flight_alt"] / 2
        await drone.offboard.set_position_ned(
            PositionNedYaw(dest_n, dest_e, mid_alt, 0.))
        await asyncio.sleep(1.5)
        dest_n, dest_e = await self._cam_align(drone, dest_n, dest_e, mid_alt)
        # 정렬된 위치로 최종 하강
        await drone.offboard.set_position_ned(
            PositionNedYaw(dest_n, dest_e, CFG["drop_alt"], 0.))
        await asyncio.sleep(1.5)
        if on_grab:
            on_grab()
        try:
            await drone.gripper.grab()
        except Exception:
            await drone.action.set_actuator(CFG["gripper_actuator"], 0.)
        await asyncio.sleep(0.5)
        await drone.offboard.set_position_ned(
            PositionNedYaw(dest_n, dest_e, CFG["flight_alt"], 0.))
        await asyncio.sleep(2.0)

    async def _verify_anchor_coverage(self, expected_count: int,
                                       timeout: float = 5.0):
        """
        앵커 투하 후 UWB 거리 측정 수가 증가했는지 확인.
        expected_count: 투하 전 앵커 수. 투하 후 +1 이상이면 성공.
        """
        t0 = asyncio.get_running_loop().time()
        while asyncio.get_running_loop().time() - t0 < timeout:
            n = len(self.uwb.get_distances())
            if n > expected_count:
                self._log(f"  ✅ UWB 앵커 수신 확인 ({n}개)")
                return True
            await asyncio.sleep(0.3)
        self._log(f"  ⚠ 신규 앵커 UWB 미수신 ({timeout:.0f}s 내) — 투하 실패 가능")
        return False

    async def _real_fly_drop(self, drone, dest_n, dest_e, anchor):
        await drone.offboard.set_position_ned(
            PositionNedYaw(dest_n, dest_e, CFG["flight_alt"], 0.))
        await self._wait_reach(drone, dest_n, dest_e)
        await drone.offboard.set_position_ned(
            PositionNedYaw(dest_n, dest_e, CFG["drop_alt"], 0.))
        await asyncio.sleep(2.0)
        try:
            await drone.gripper.release()
        except Exception:
            await drone.action.set_actuator(CFG["gripper_actuator"], 1.)
            await asyncio.sleep(1.5)
            await drone.action.set_actuator(CFG["gripper_actuator"], 0.)
        await asyncio.sleep(CFG["drop_wait"])
        n_before = len(self.uwb.get_distances())
        GazeboMonitor.spawn(anchor, world=CFG["gz_world"])
        self.uwb.add_sitl_anchor(dest_n, dest_e, 0.)
        await drone.offboard.set_position_ned(
            PositionNedYaw(dest_n, dest_e, CFG["flight_alt"], 0.))
        # 신규 앵커에서 UWB 수신 확인 (비동기, 이륙 중 병렬)
        asyncio.ensure_future(
            self._verify_anchor_coverage(n_before))
        await asyncio.sleep(3.0)

    async def _vision_inject_task(self, drone):
        """
        UWB 위치를 MAVSDK mocap API로 PX4 EKF2에 주입.
        SITL(UDP) / REAL(serial) 모두 동일한 drone 연결 재사용.
        """
        NAN    = float('nan')
        period = 1.0 / max(CFG["vision_rate_hz"], 1)
        try:
            while True:
                t0  = time.monotonic()
                pos = self.uwb.get_position()
                if pos:
                    n, e, z    = pos
                    sn, se, sz = self.uwb.get_quality()["sigma"]
                    # 21-element upper triangular covariance (NED 위치 3×3, 자세 3×3)
                    # 자세는 UWB로 알 수 없음 → 1.0 rad 보수적
                    cov = [sn**2, NAN, NAN, NAN, NAN, NAN,
                                  se**2, NAN, NAN, NAN, NAN,
                                        sz**2, NAN, NAN, NAN,
                                              1.0, NAN, NAN,
                                                   1.0, NAN,
                                                        1.0]
                    try:
                        await drone.mocap.set_vision_position_estimate(
                            _VPE(
                                time_usec     = int(time.monotonic() * 1e6),
                                position_body = _PosBody(float(n), float(e), float(-z)),
                                angle_body    = _AngBody(0., 0., 0.),
                                pose_covariance = _Cov(cov),
                            )
                        )
                    except Exception:
                        pass
                await asyncio.sleep(max(0., period - (time.monotonic() - t0)))
        except asyncio.CancelledError:
            pass

    async def _mission_coro(self, steps: list, address: str = None):
        pos_task    = None   # finally 블록에서 NameError 방지
        vision_task = None
        try:
            drone = System()
            await drone.connect(system_address=address or CFG["address"])
            self._log("  드론 연결 대기...")
            async for s in drone.core.connection_state():
                if s.is_connected:
                    self._log("  ✅ 연결됨"); break

            # PX4 파라미터: GPS-denied + UWB vision 위치 소스
            try:
                await drone.param.set_param_int("NAV_RCL_ACT", 0)
                await drone.param.set_param_int("COM_RCL_EXCEPT", 4)
                # GPS 없이 arm 허용
                await drone.param.set_param_int("COM_ARM_WO_GPS", 1)
                # vision 수신 지연 보정 (pymavlink UDP 지연)
                await drone.param.set_param_float("EKF2_EV_DELAY", 25.0)
                # vision 최소 품질 기준 완화 (covariance 허용 범위)
                await drone.param.set_param_float("EKF2_EVP_NOISE", 0.1)

                # PX4 v1.14+ 신API: EKF2_AID_MASK → EKF2_EV_CTRL + EKF2_GPS_CTRL
                # 구버전(v1.13 이하)은 EKF2_AID_MASK 사용
                # 참고: https://github.com/PX4/PX4-Autopilot/issues/17969
                try:
                    # v1.14+: EKF2_EV_CTRL bit0=수평위치 bit1=수직위치
                    await drone.param.set_param_int("EKF2_EV_CTRL", 3)
                    # GPS 융합 완전 비활성화
                    await drone.param.set_param_int("EKF2_GPS_CTRL", 0)
                    # 고도 기준: 3=EV(vision), GPS 없는 환경에서 EKF Z 안정화
                    await drone.param.set_param_int("EKF2_HGT_REF", 3)
                    self._log("  ✅ PX4 v1.14+ API: EKF2_EV_CTRL=3, GPS_CTRL=0, HGT_REF=3")
                except Exception:
                    # v1.13 이하 fallback
                    await drone.param.set_param_int("EKF2_AID_MASK", 8)
                    await drone.param.set_param_int("NAV_GNSS_MASK", 0)
                    self._log("  ✅ PX4 v1.13 API: EKF2_AID_MASK=8, NAV_GNSS_MASK=0")
                self._log("  ✅ 파라미터: GPS off, vision only, ARM_WO_GPS=1")
            except Exception as pe:
                self._log(f"  ⚠ 파라미터 설정 실패: {pe}")

            # telemetry + vision 동시 시작
            # telemetry: PX4 위치 → UWB 시뮬 갱신 + 드론 아이콘 이동
            # vision   : UWB 위치 → PX4 EKF2 주입 (MAVSDK mocap, REAL/SITL 공통)
            pos_task    = asyncio.create_task(self._telemetry_task(drone))
            vision_task = asyncio.create_task(self._vision_inject_task(drone))
            self._log(f"  🔭 Vision 주입 시작: MAVSDK mocap  {CFG['vision_rate_hz']}Hz")

            # EKF2_EV_CTRL 변경 후 vision 메시지가 PX4에 도달 + EKF 수렴 대기
            self._log("  UWB vision 수렴 대기 (3s)...")
            await asyncio.sleep(3.0)

            # is_local_position_ok 타임아웃 (Vision 없으면 무한 대기 방지)
            self._log("  로컬 포지션 대기...")
            t_health = asyncio.get_running_loop().time()
            async for h in drone.telemetry.health():
                if h.is_local_position_ok:
                    self._log("  ✅ 로컬 포지션 OK (UWB vision 수신됨)"); break
                if asyncio.get_running_loop().time() - t_health > 15.0:
                    self._log("  ⚠ 로컬 포지션 타임아웃 — vision 미수신"
                              " (VisionPositionInjector 확인 필요)")
                    break

            await drone.action.arm()
            await drone.offboard.set_position_ned(
                PositionNedYaw(0., 0., CFG["flight_alt"], 0.))
            await drone.offboard.start()
            self._log("  이륙...")
            self.root.after(0, lambda: self._update_rover_pos(0.0))
            await asyncio.sleep(CFG["takeoff_wait"])

            N = len(steps)
            for idx, step in enumerate(steps):
                if self._stop_evt.is_set():
                    self._log("⏹ 미션 중단됨"); break
                atype   = step["type"]
                anchor  = step["anchor"]
                t_n, t_e = anchor["n"], anchor["e"]
                rover_t  = step.get("rover_t", 0.0)

                if atype == "skip":
                    self._log(f"\n⚠ [{idx+1}/{N}] {anchor['name']} — {step.get('reason','')}")
                    self.root.after(0, lambda t=rover_t: self._update_rover_pos(t))
                    continue

                self._start_rover_move(rover_t)

                if atype == "from_depot":
                    p_n, p_e = step["rover_pos"]
                    self._log(f"\n─ [{idx+1}/{N}] 디포 픽업 @ 로버({p_n:.1f},{p_e:.1f})")
                    await self._real_fly_pick(drone, p_n, p_e)
                    self.root.after(0, self._depot_consume)

                elif atype == "relocate":
                    src   = step["source"]
                    stype = step["source_type"]
                    src_n, src_e = src["n"], src["e"]
                    src_label = (f"A{src['id']}" if "id" in src
                                 else f"신규({src_n:.1f},{src_e:.1f})")
                    self._log(f"\n─ [{idx+1}/{N}] 재배치: {src_label} "
                               f"({src_n:.1f},{src_e:.1f}) → ({t_n:.1f},{t_e:.1f})")
                    self._log(f"    [이유] 로버 뒤쪽 + 끝점에서 가장 멀리 있는 앵커")

                    if stype == "existing":
                        def on_grab(sid=src["id"]):
                            self.root.after(0, lambda: self._mark_relocated_existing(sid))
                            GazeboMonitor.despawn(f"uwb_anchor_{sid}", world=CFG["gz_world"])
                    else:
                        def on_grab(sn=src_n, se=src_e, nm=src["name"]):
                            self.root.after(0, lambda: self._mark_relocated_new(sn, se))
                            GazeboMonitor.despawn(nm, world=CFG["gz_world"])
                            self.root.after(0, lambda: self._remove_from_installed(nm))

                    await self._real_fly_pick(drone, src_n, src_e, on_grab=on_grab)

                self._log(f"  → 설치 @ ({t_n:.1f},{t_e:.1f})")
                self.root.after(0, lambda nm=anchor["name"]: self._mark_installing(nm))
                await self._real_fly_drop(drone, t_n, t_e, anchor)
                self.root.after(0, lambda nm=anchor["name"]: self._mark_installed(nm))

            if not self._stop_evt.is_set():
                h_n, h_e = self.drone_home
                self._log("\n홈 복귀...")
                await drone.offboard.set_position_ned(
                    PositionNedYaw(h_n, h_e, CFG["flight_alt"], 0.))
                await self._wait_reach(drone, h_n, h_e)
                await drone.offboard.stop()
                await drone.action.land()
                self._log("✅ 임무 완료!")
            else:
                self._log("⏹ 중단됨 → 긴급 착지")
                await drone.offboard.stop()
                await drone.action.land()

        except Exception as ex:
            self._log(f"❌ {ex}")
            try:
                await drone.offboard.stop()
            except Exception:
                pass
            try:
                self._log("  긴급 착지 시도...")
                await drone.action.land()
                async for in_air in drone.telemetry.in_air():
                    if not in_air:
                        self._log("  착지 완료"); break
            except Exception:
                pass
        finally:
            if pos_task:
                pos_task.cancel()
            if vision_task:
                vision_task.cancel()
            self.root.after(0, self._clear_drone_pos)
            self.root.after(0, lambda: self.btn_confirm.config(state=tk.NORMAL))
            self.root.after(0, lambda: self.btn_stop.config(state=tk.DISABLED))

    async def _telemetry_task(self, drone):
        try:
            async for pv in drone.telemetry.position_velocity_ned():
                n   = pv.position.north_m
                e   = pv.position.east_m
                alt = -pv.position.down_m   # 양수 = 위
                vn  = pv.velocity.north_m_s
                ve  = pv.velocity.east_m_s
                vz  = -pv.velocity.down_m_s  # NED down → 위 양수
                # SITL: Gazebo 텔레메트리 → UWB 거리 시뮬 + 속도 보조
                if MODE == "SITL":
                    self.uwb.set_sitl_drone_pos(n, e, -alt)  # NED: 위=-
                    self.uwb.set_drone_velocity(vn, ve, -vz)  # NED down
                # 기압계 고도로 EKF Z 보정 (SITL/REAL 공통)
                # 앵커가 평면 배치된 환경에서 Z 발산 방지
                self.uwb.set_baro_altitude(alt)
                self.root.after(0, lambda nn=n, ee=e, aa=alt:
                                self._update_drone_pos(nn, ee, aa))
        except asyncio.CancelledError:
            pass

    # ══════════════════════════════════════════════════════════════════════
    #  로그
    # ══════════════════════════════════════════════════════════════════════

    def _open_cam_window(self):
        """독립 카메라 창 열기 (640×480, 항상 최상위)."""
        win = tk.Toplevel(self.root)
        win.title("📷 카메라 뷰 — 앵커 감지")
        win.configure(bg=C["bg"])
        win.resizable(False, False)
        win.attributes("-topmost", True)

        W, H = 640, 480
        canvas = tk.Canvas(win, width=W, height=H,
                           bg="#050a0e", highlightthickness=0)
        canvas.pack()

        status = tk.Label(win, text="대기중...", bg=C["bg"], fg=C["dim"],
                          font=("Courier", 9))
        status.pack(pady=4)

        photo_ref = [None]   # GC 방지

        def _refresh():
            if not win.winfo_exists():
                return
            frame = self.camera.get_frame()
            det   = self.camera.detect_anchor(frame)

            if frame is None:
                # 카메라 없음 → 안내 텍스트
                canvas.delete("all")
                canvas.create_rectangle(0, 0, W, H, fill="#050a0e")
                canvas.create_text(W//2, H//2 - 20,
                    text="카메라 없음",
                    fill="#3a5060", font=("Courier", 18, "bold"))
                canvas.create_text(W//2, H//2 + 10,
                    text=f"source: {CFG['camera_source']}",
                    fill="#3a5060", font=("Courier", 11))
                if CFG["camera_source"] == "gz":
                    canvas.create_text(W//2, H//2 + 35,
                        text="Gazebo를 먼저 실행하세요",
                        fill="#3a5060", font=("Courier", 10))
                status.config(text="카메라 연결 없음", fg=C["dim"])
            else:
                ann = self.camera.annotate(frame, det)
                photo = self.camera.get_tk_image(ann, w=W, h=H)
                if photo:
                    canvas.delete("all")
                    canvas.create_image(0, 0, anchor="nw", image=photo)
                    photo_ref[0] = photo
                if det:
                    cx, cy = det[0], det[1]
                    status.config(
                        text=f"앵커 감지 ● 픽셀({cx},{cy})",
                        fg="#52e0a0")
                else:
                    status.config(text="탐색 중...", fg=C["rover"])
            win.after(66, _refresh)

        _refresh()

    def _cam_refresh(self):
        """66ms(15fps)마다 패널 소형 캔버스 갱신."""
        frame = self.camera.get_frame()
        det   = self.camera.detect_anchor(frame)
        if frame is None:
            # 카메라 없음 → 안내 텍스트
            self.cam_canvas.delete("all")
            self.cam_canvas.create_text(108, 70,
                text="NO CAMERA", fill="#2a3a4a",
                font=("Courier", 12, "bold"))
            self.cam_canvas.create_text(108, 90,
                text=f"({CFG['camera_source']})", fill="#2a3a4a",
                font=("Courier", 8))
            self.cam_status_lbl.config(text="없음", fg=C["dim"])
        else:
            ann   = self.camera.annotate(frame, det)
            photo = self.camera.get_tk_image(ann, w=216, h=162)
            if photo:
                self.cam_canvas.delete("all")
                self.cam_canvas.create_image(0, 0, anchor="nw", image=photo)
                self._cam_photo = photo
            if det:
                self.cam_status_lbl.config(
                    text=f"감지 ({det[0]},{det[1]})", fg="#52e0a0")
            else:
                self.cam_status_lbl.config(
                    text="탐색중", fg=C["rover"])
        self.root.after(66, self._cam_refresh)

    def _uwb_refresh(self):
        """500ms마다 UWB 위치 + 품질 표시 갱신."""
        pos   = self.uwb.get_position()
        dists = self.uwb.get_distances()
        qual  = self.uwb.get_quality()
        sn, se, sz = qual["sigma"]

        if pos:
            self.uwb_pos_lbl.config(
                text=(f"📡 N:{pos[0]:.2f} E:{pos[1]:.2f} Z:{pos[2]:.2f}"
                      f"  σ({sn:.2f},{se:.2f})"),
                fg="#a0d8ff")
        elif dists:
            self.uwb_pos_lbl.config(
                text=f"📡 UWB: 측정중 ({len(dists)}앵커)",
                fg=C["rover"])
        else:
            self.uwb_pos_lbl.config(text="📡 UWB: 대기", fg=C["dim"])

        # 앵커 건강 경고 로그 (새로 나빠진 앵커만)
        health = qual.get("anchor_health", {})
        for aid, h in health.items():
            prev = getattr(self, "_last_anchor_health", {}).get(aid, {})
            if not h["ok"] and prev.get("ok", True):
                self._log(f"  ⚠ 앵커 {aid} 품질 저하:"
                          f" mean={h['mean']:+.3f}m std={h['std']:.3f}m"
                          f" (NLOS/오배치 의심)", "warn")
        self._last_anchor_health = health

        self.root.after(500, self._uwb_refresh)

    def _on_close(self):
        self._stop_evt.set()
        self.gz_monitor.stop()
        self.camera.stop()
        self.uwb.stop()
        self.root.destroy()

    def _log(self, msg: str, tag: str = "info"):
        # 메시지 내용으로 태그 자동 결정
        if tag == "info":
            if msg.startswith(("✅", "▶", "완료")):
                tag = "ok"
            elif msg.startswith(("⚠", "⏹", "타임아웃")):
                tag = "warn"
            elif msg.startswith("❌"):
                tag = "err"
        def _do():
            self.log.config(state=tk.NORMAL)
            self.log.insert(tk.END, msg + "\n", tag)
            self.log.see(tk.END)
            self.log.config(state=tk.DISABLED)
        self.root.after(0, _do)


# ════════════════════════════════════════════════════════════════════════════
#  엔트리 포인트
# ════════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    root = tk.Tk()
    # 최소 창 크기 지정
    root.minsize(900, 760)
    app  = UWBApp(root)
    root.mainloop()