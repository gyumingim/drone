"""
UWB 앵커 보정 (Calibration) - 모드 통합 버전
=============================================

알고리즘:
  0. 이륙 전 지상에서 전 앵커까지 UWB 2D 기준거리 측정
  1. 각 앵커 위로 그라디언트 탐색 이동
     - 수평거리 = sqrt(UWB_dist² - 고도²) < ABOVE_THRESHOLD
  2. 앵커 위에서 나머지 앵커들과 3D 거리 측정 → 2D 변환
     - d2D(Ai, Aj) = sqrt(d3D(tag_above_Ai, Aj)² - 고도²)
  3. 완전 거리 행렬 구성: [HOME, A1, A2, A3, A4]
  4. Classical MDS → 상대 2D 위치 (NED 무관 임의 좌표계)
  5. 드론 진행 방향(HOME → A1 bearing)으로 NED 좌표계 정렬
  6. 보정된 앵커 위치 출력 (설치 코드 대비 오차 포함)

SITL 실행:
  터미널1) cd PX4-Autopilot && PX4_GZ_MODEL=x500 make px4_sitl gz_x500
  터미널2) python3 uwb_anchor_calibration.py

실제 드론:
  MODE = "REAL" 로 변경 후 실행

의존성:
  pip install mavsdk numpy
"""

import asyncio
import math
import subprocess
import numpy as np
from typing import Optional
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw


# ═══════════════════════════════════════════
#  ★ 모드 선택
# ═══════════════════════════════════════════

MODE = "SITL"   # "SITL" | "REAL"


# ═══════════════════════════════════════════
#  앵커 초기 위치 (설치 코드 실행 결과 입력)
#  SITL 에서는 이 값이 "실제 설치 위치"로 시뮬레이션됨
# ═══════════════════════════════════════════

ANCHOR_POSITIONS = [
    {"id": 1, "n": 3.0,  "e": 0.0},
    {"id": 2, "n": 5.5,  "e": 3.0},
    {"id": 3, "n": 2.0,  "e": 6.0},
    {"id": 4, "n": -1.0, "e": 3.5},
]


# ═══════════════════════════════════════════
#  공통 설정
# ═══════════════════════════════════════════

ABOVE_THRESHOLD  = 0.25   # 앵커 위 판단 수평거리 임계값 (m)
N_DIST_SAMPLES   = 5      # 거리 측정 평균 샘플 수
GRAD_STEP_INIT   = 0.5    # 그라디언트 초기 스텝 (m)
GRAD_STEP_MIN    = 0.08   # 그라디언트 최소 스텝 (m)
GRAD_MAX_ITER    = 200    # 그라디언트 최대 반복 횟수
GRAD_SHRINK      = 0.6    # 개선 없을 때 스텝 축소 비율

# MDS 반전 보정 (결과가 좌우 대칭으로 나올 경우 True 로 변경)
MDS_REFLECT_E    = False  # E축(좌우) 반전


# ═══════════════════════════════════════════
#  모드별 설정
# ═══════════════════════════════════════════

_SITL_CFG = {
    "address"        : "udp://:14540",
    "flight_alt"     : -2.5,    # 순항 고도 NED (음수=위)
    "step_wait"      : 1.5,     # 그라디언트 한 스텝 대기 (초)
    "settle_wait"    : 1.0,     # 앵커 위 안정화 대기 (초)
    "takeoff_wait"   : 5.0,
    "return_wait"    : 4.0,
    "uwb_noise_std"  : 0.02,    # UWB 시뮬 노이즈 표준편차 (m)
    "spawn_markers"  : True,    # ★ Gazebo 마커 스폰 여부
    "gz_world"       : "default",  # Gazebo 월드 이름
}

_REAL_CFG = {
    "address"        : "serial:///dev/ttyUSB0:57600",
    "flight_alt"     : -8.0,
    "step_wait"      : 3.0,
    "settle_wait"    : 2.5,
    "takeoff_wait"   : 8.0,
    "return_wait"    : 8.0,
    "uwb_port"       : "/dev/ttyUSB1",
    "uwb_baud"       : 115200,
    "spawn_markers"  : False,   # 실제 환경엔 Gazebo 없음
}

CFG = _SITL_CFG if MODE == "SITL" else _REAL_CFG


# ═══════════════════════════════════════════
#  ★ Gazebo 앵커 마커 스폰 (dropper 코드 방식 동일)
# ═══════════════════════════════════════════

def spawn_gazebo_marker(anchor: dict):
    """
    SITL 모드에서 Gazebo에 앵커를 빨간 원기둥으로 시각화.
    - CFG["spawn_markers"] == False 이면 자동 스킵 (REAL 모드 안전)
    - 좌표 변환: NED(n, e) → Gazebo(x=e, y=n)
    """
    if not CFG.get("spawn_markers", False):
        return

    aid  = anchor["id"]
    n, e = anchor["n"], anchor["e"]
    name = f"uwb_anchor_{aid}"
    world = CFG.get("gz_world", "default")

    # SDF 문자열 (이스케이프 주의 - dropper 코드 방식 그대로)
    sdf = (
        '<?xml version=\\"1.0\\"?>'
        '<sdf version=\\"1.7\\">'
        f'<model name=\\"{name}\\">'
        '<static>true</static>'
        '<link name=\\"link\\">'
        '<visual name=\\"visual\\">'
        '<geometry><cylinder>'
        '<radius>0.12</radius><length>0.35</length>'
        '</cylinder></geometry>'
        '<material>'
        '<ambient>1 0 0 1</ambient>'
        '<diffuse>1 0 0 1</diffuse>'
        '</material>'
        '</visual>'
        # 충돌체 없음 - 드론 비행 방해 방지
        '</link>'
        '</model>'
        '</sdf>'
    )

    # NED → Gazebo 좌표: x=East, y=North, z=0.175 (원기둥 중심)
    req = (
        f'sdf: "{sdf}" '
        f'pose {{ position {{ x: {e} y: {n} z: 0.175 }} }} '
        f'name: "{name}"'
    )

    cmd = [
        "gz", "service",
        "-s", f"/world/{world}/create",
        "--reqtype", "gz.msgs.EntityFactory",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", "2000",
        "--req", req,
    ]

    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            print(f"    🔴 Gazebo 마커 스폰: {name}  (N={n}, E={e})")
        else:
            err = result.stderr.strip() or result.stdout.strip()
            print(f"    ⚠️  마커 스폰 실패 [{name}]: {err}")
    except FileNotFoundError:
        print("    ⚠️  'gz' 명령어를 찾을 수 없음 (Gazebo 미설치?)")
    except subprocess.TimeoutExpired:
        print(f"    ⚠️  {name} 스폰 타임아웃")


def spawn_all_anchors():
    """ANCHOR_POSITIONS 전체를 Gazebo에 스폰."""
    if not CFG.get("spawn_markers", False):
        return
    print("\n  [SITL] 앵커 Gazebo 마커 스폰 중...")
    for anchor in ANCHOR_POSITIONS:
        spawn_gazebo_marker(anchor)
    print("  ✅ 마커 스폰 완료\n")


def despawn_gazebo_marker(anchor: dict):
    """
    스폰된 Gazebo 마커 제거 (선택적으로 사용).
    임무 종료 후 정리할 때 호출.
    """
    if not CFG.get("spawn_markers", False):
        return

    aid   = anchor["id"]
    name  = f"uwb_anchor_{aid}"
    world = CFG.get("gz_world", "default")

    req = f'name: "{name}"'
    cmd = [
        "gz", "service",
        "-s", f"/world/{world}/remove",
        "--reqtype", "gz.msgs.Entity",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", "2000",
        "--req", req,
    ]
    try:
        subprocess.run(cmd, capture_output=True, text=True, timeout=5)
        print(f"    🗑  Gazebo 마커 제거: {name}")
    except Exception:
        pass  # 제거 실패는 무시


# ═══════════════════════════════════════════
#  UWB 인터페이스
# ═══════════════════════════════════════════

class UWBReader:
    """UWB 거리 측정 추상 인터페이스"""

    async def get_distance(self, anchor_id: int) -> float:
        raise NotImplementedError

    async def get_avg_distance(self, anchor_id: int,
                               n: int = N_DIST_SAMPLES) -> float:
        samples = []
        for _ in range(n):
            samples.append(await self.get_distance(anchor_id))
            await asyncio.sleep(0.05)
        return float(np.mean(samples))

    async def get_avg_distances(self, anchor_ids: list,
                                n: int = N_DIST_SAMPLES) -> dict:
        return {aid: await self.get_avg_distance(aid, n) for aid in anchor_ids}


class SITLUWBReader(UWBReader):
    """
    SITL 시뮬레이션 UWB.
    ANCHOR_POSITIONS 를 '진짜 위치'로 간주, 3D 거리 + 노이즈 반환.
    """

    def __init__(self, true_anchors: list, noise_std: float = 0.02):
        self._anchors  = {a["id"]: a for a in true_anchors}
        self._noise    = noise_std
        self._n = 0.0
        self._e = 0.0
        self._d = 0.0

    def update_pos(self, n: float, e: float, d: float):
        self._n, self._e, self._d = n, e, d

    async def get_distance(self, anchor_id: int) -> float:
        a    = self._anchors[anchor_id]
        dn   = self._n - a["n"]
        de   = self._e - a["e"]
        dz   = -self._d
        dist = math.sqrt(dn**2 + de**2 + dz**2)
        return max(0.01, dist + np.random.normal(0, self._noise))


class RealUWBReader(UWBReader):
    """실제 UWB 하드웨어 인터페이스 (모듈에 맞게 구현 필요)."""

    def __init__(self, port: str, baud: int):
        self._port = port
        self._baud = baud
        print(f"  ⚠️  RealUWBReader: {port}@{baud} 초기화 (구현 필요)")

    async def get_distance(self, anchor_id: int) -> float:
        raise NotImplementedError(
            "실제 UWB 하드웨어 드라이버를 구현하세요.\n"
            "모듈 문서를 참고하여 get_distance() 를 완성해주세요."
        )


def create_uwb_reader() -> UWBReader:
    if MODE == "SITL":
        return SITLUWBReader(ANCHOR_POSITIONS, noise_std=CFG["uwb_noise_std"])
    return RealUWBReader(CFG["uwb_port"], CFG["uwb_baud"])


# ═══════════════════════════════════════════
#  드론 텔레메트리 헬퍼
# ═══════════════════════════════════════════

async def get_ned_pos(drone: System) -> tuple:
    async for pv in drone.telemetry.position_velocity_ned():
        return pv.position.north_m, pv.position.east_m, pv.position.down_m


# ═══════════════════════════════════════════
#  수학 유틸
# ═══════════════════════════════════════════

def horiz_dist(d3d: float, alt_m: float) -> float:
    return math.sqrt(max(0.0, d3d**2 - alt_m**2))


def bearing_deg(n: float, e: float) -> float:
    return math.degrees(math.atan2(e, n)) % 360


# ═══════════════════════════════════════════
#  앵커 위 탐색 - 그라디언트 하강
# ═══════════════════════════════════════════

async def navigate_to_anchor(drone, uwb, target_id, flight_alt_ned):
    alt_m = abs(flight_alt_ned)
    step  = GRAD_STEP_INIT
    label = f"앵커{target_id}"

    for iteration in range(GRAD_MAX_ITER):
        n, e, d = await get_ned_pos(drone)
        if isinstance(uwb, SITLUWBReader):
            uwb.update_pos(n, e, d)

        d3d = await uwb.get_avg_distance(target_id, n=3)
        hd  = horiz_dist(d3d, alt_m)

        if hd < ABOVE_THRESHOLD:
            brg = bearing_deg(n, e)
            print(f"    ✅ {label} 위 도달 | N={n:.3f} E={e:.3f} | hd={hd:.3f}m")
            return n, e, brg

        # ─ 4방향 탐색 ─────────────────────────────────────────────
        directions = [(step,0.), (-step,0.), (0.,step), (0.,-step)]
        best_hd, best_dn, best_de = hd, 0., 0.

        for dn, de in directions:
            if isinstance(uwb, SITLUWBReader):
                # ★ SITL: 실제 이동 없이 가상 위치로만 계산
                uwb.update_pos(n + dn, e + de, flight_alt_ned)
                cand_d3d = await uwb.get_avg_distance(target_id, n=2)
                uwb.update_pos(n, e, d)   # 원래 위치로 복원
            else:
                # REAL: 기존대로 실제 이동 후 측정
                await drone.offboard.set_position_ned(
                    PositionNedYaw(n+dn, e+de, flight_alt_ned, 0.)
                )
                await asyncio.sleep(CFG["step_wait"] * 0.4)
                cand_d3d = await uwb.get_avg_distance(target_id, n=2)

            cand_hd = horiz_dist(cand_d3d, alt_m)
            if cand_hd < best_hd:
                best_hd, best_dn, best_de = cand_hd, dn, de

        # ─ 이동 결정 (한 번만 이동) ───────────────────────────────
        if best_hd < hd:
            await drone.offboard.set_position_ned(
                PositionNedYaw(n+best_dn, e+best_de, flight_alt_ned, 0.)
            )
            await asyncio.sleep(CFG["step_wait"])   # 충분히 대기
        else:
            step = max(GRAD_STEP_MIN, step * GRAD_SHRINK)
            if step == GRAD_STEP_MIN:
                print(f"    ⚠️  {label} 수렴 한계 (hd={hd:.3f}m)")
                return n, e, bearing_deg(n, e)

    n, e, _ = await get_ned_pos(drone)
    return n, e, bearing_deg(n, e)

# ═══════════════════════════════════════════
#  거리 측정
# ═══════════════════════════════════════════

async def measure_ground_distances(uwb: UWBReader, anchor_ids: list) -> dict:
    print("\n  [지상 거리 측정]")
    if isinstance(uwb, SITLUWBReader):
        uwb.update_pos(0.0, 0.0, 0.0)

    dists = await uwb.get_avg_distances(anchor_ids, n=N_DIST_SAMPLES)
    for aid, d in dists.items():
        print(f"    HOME → 앵커{aid}: {d:.4f}m")
    return dists


async def measure_above_anchor(
    uwb: UWBReader,
    drone: System,
    above_id: int,
    other_ids: list,
    flight_alt_ned: float,
) -> dict:
    alt_m  = abs(flight_alt_ned)
    n, e, d = await get_ned_pos(drone)
    if isinstance(uwb, SITLUWBReader):
        uwb.update_pos(n, e, d)

    raw    = await uwb.get_avg_distances(other_ids, n=N_DIST_SAMPLES)
    d2d_map = {}
    for aid, d3d in raw.items():
        d2d = horiz_dist(d3d, alt_m)
        d2d_map[aid] = d2d
        print(f"    앵커{above_id} → 앵커{aid}: 3D={d3d:.4f}m  2D={d2d:.4f}m")
    return d2d_map


# ═══════════════════════════════════════════
#  거리 행렬 구성
# ═══════════════════════════════════════════

def build_distance_matrix(
    anchor_ids: list,
    ground_dists: dict,
    above_dists: dict,
) -> tuple:
    n_nodes = 1 + len(anchor_ids)
    D       = np.zeros((n_nodes, n_nodes))
    labels  = ["HOME"] + [f"A{aid}" for aid in anchor_ids]
    idx     = {aid: i + 1 for i, aid in enumerate(anchor_ids)}

    for aid, d in ground_dists.items():
        i = idx[aid]
        D[0, i] = D[i, 0] = d

    for above_id, others in above_dists.items():
        i = idx[above_id]
        for other_id, d2d in others.items():
            j = idx[other_id]
            if D[i, j] == 0:
                D[i, j] = D[j, i] = d2d
            else:
                mean_d  = (D[i, j] + d2d) / 2
                D[i, j] = D[j, i] = mean_d

    return D, labels


def print_distance_matrix(D: np.ndarray, labels: list):
    col_w = 9
    header = f"{'':>6}" + "".join(f"{l:>{col_w}}" for l in labels)
    print(f"  {header}")
    for i, row_label in enumerate(labels):
        row = "".join(f"{D[i,j]:>{col_w}.3f}" for j in range(len(labels)))
        print(f"  {row_label:>6}{row}")


# ═══════════════════════════════════════════
#  Classical MDS
# ═══════════════════════════════════════════

def classical_mds(D: np.ndarray, dims: int = 2) -> np.ndarray:
    n    = D.shape[0]
    D2   = D ** 2
    H    = np.eye(n) - np.ones((n, n)) / n
    B    = -0.5 * H @ D2 @ H

    eigvals, eigvecs = np.linalg.eigh(B)
    order   = np.argsort(eigvals)[::-1]
    eigvals = eigvals[order]
    eigvecs = eigvecs[:, order]

    pos_eig = np.maximum(eigvals[:dims], 0)
    X       = eigvecs[:, :dims] * np.sqrt(pos_eig)
    return X


def mds_rmse(D: np.ndarray, X: np.ndarray) -> float:
    n, errs = D.shape[0], []
    for i in range(n):
        for j in range(i + 1, n):
            if D[i, j] > 0:
                d_mds = np.linalg.norm(X[i] - X[j])
                errs.append((d_mds - D[i, j]) ** 2)
    return math.sqrt(np.mean(errs)) if errs else 0.0


# ═══════════════════════════════════════════
#  좌표 정렬: MDS → NED
# ═══════════════════════════════════════════

def align_mds_to_ned(
    X_mds: np.ndarray,
    home_idx: int,
    first_anchor_idx: int,
    bearing_home_to_first: float,
    reflect_e: bool = False,
) -> np.ndarray:
    X = X_mds - X_mds[home_idx]

    fa        = X[first_anchor_idx]
    theta_mds = math.atan2(fa[1], fa[0])
    phi_rad   = math.radians(bearing_home_to_first)

    alpha     = phi_rad - theta_mds
    cos_a, sin_a = math.cos(alpha), math.sin(alpha)
    R = np.array([[cos_a, -sin_a],
                  [sin_a,  cos_a]])

    X_rot = (R @ X.T).T

    if reflect_e:
        X_rot[:, 1] *= -1

    return X_rot


# ═══════════════════════════════════════════
#  드론 연결 / 이륙 / 귀환
# ═══════════════════════════════════════════

async def connect_drone() -> System:
    drone = System()
    await drone.connect(system_address=CFG["address"])
    print(f"  드론 연결 대기 중... ({CFG['address']})")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("  ✅ 연결됨")
            break
    print("  위치 추정 대기 중...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_local_position_ok:
            print("  ✅ 위치 추정 완료")
            break
    return drone


async def takeoff(drone: System):
    print("\n이륙...")
    await drone.action.arm()
    await drone.offboard.set_position_ned(
        PositionNedYaw(0.0, 0.0, CFG["flight_alt"], 0.0)
    )
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"  ❌ Offboard 실패: {e}")
        await drone.action.disarm()
        raise
    await asyncio.sleep(CFG["takeoff_wait"])
    print("  ✅ 이륙 완료")


async def return_home_and_land(drone: System):
    print("\n홈 복귀 중...")
    await drone.offboard.set_position_ned(
        PositionNedYaw(0.0, 0.0, CFG["flight_alt"], 0.0)
    )
    await asyncio.sleep(CFG["return_wait"])
    await drone.offboard.stop()
    await drone.action.land()
    print("  ✅ 착륙 완료")


# ═══════════════════════════════════════════
#  보정 결과 출력
# ═══════════════════════════════════════════

def print_result(
    anchor_ids: list,
    calibrated_ne: np.ndarray,
    original_anchors: list,
):
    orig = {a["id"]: a for a in original_anchors}

    print("\n" + "═" * 70)
    print("  UWB 앵커 보정 결과")
    print("═" * 70)
    print(f"  {'앵커':<6} {'원래 N':>9} {'원래 E':>9}   →   "
          f"{'보정 N':>9} {'보정 E':>9}   {'오차':>7}")
    print("─" * 70)

    for i, aid in enumerate(anchor_ids):
        o      = orig[aid]
        cal_n  = calibrated_ne[i, 0]
        cal_e  = calibrated_ne[i, 1]
        err    = math.hypot(cal_n - o["n"], cal_e - o["e"])
        marker = "  ← !" if err > 0.3 else ""
        print(f"  A{aid:<5}  {o['n']:>9.4f} {o['e']:>9.4f}   →   "
              f"{cal_n:>9.4f} {cal_e:>9.4f}   {err:>6.4f}m{marker}")

    print("═" * 70)
    print("\n  ※ 보정 좌표 (코드 붙여넣기용):")
    print("  ANCHOR_POSITIONS = [")
    for i, aid in enumerate(anchor_ids):
        print(f'    {{"id": {aid}, '
              f'"n": {calibrated_ne[i,0]:.4f}, '
              f'"e": {calibrated_ne[i,1]:.4f}}},')
    print("  ]")
    print()
    print("  ※ MDS 반전 의심 시: MDS_REFLECT_E = True 후 재실행")


# ═══════════════════════════════════════════
#  메인
# ═══════════════════════════════════════════

async def main():
    print("=" * 60)
    print(f"  UWB 앵커 보정  [모드: {MODE}]")
    print("=" * 60)

    anchor_ids = [a["id"] for a in ANCHOR_POSITIONS]
    uwb        = create_uwb_reader()

    # ── 1. Gazebo 마커 스폰 (SITL 전용) ──────────────────────
    # dropper 코드와 동일한 방식으로, 이륙 전에 먼저 스폰
    spawn_all_anchors()

    # ── 2. 드론 연결 ──────────────────────────────────────────
    print("\n[1] 드론 연결...")
    drone = await connect_drone()

    # ── 3. 지상 거리 측정 (이륙 전) ──────────────────────────
    print("\n[2] 이륙 전 지상 UWB 거리 측정...")
    ground_dists = await measure_ground_distances(uwb, anchor_ids)

    # ── 4. 이륙 ───────────────────────────────────────────────
    await takeoff(drone)

    # ── 5. 각 앵커 순차 방문 ─────────────────────────────────
    print(f"\n[3] 앵커 {len(anchor_ids)}개 순차 방문 & 거리 측정...")

    above_dists     = {}
    bearing_records = {}

    for anchor_id in anchor_ids:
        print(f"\n{'─'*10} 앵커{anchor_id} {'─'*10}")
        other_ids = [aid for aid in anchor_ids if aid != anchor_id]

        final_n, final_e, brg = await navigate_to_anchor(
            drone, uwb, anchor_id, CFG["flight_alt"]
        )
        bearing_records[anchor_id] = brg

        print(f"  앵커{anchor_id} 위 안정화 중...")
        await asyncio.sleep(CFG["settle_wait"])

        n, e, d = await get_ned_pos(drone)
        if isinstance(uwb, SITLUWBReader):
            uwb.update_pos(n, e, d)

        print(f"  앵커{anchor_id} 위에서 나머지 앵커 거리 측정:")
        d2d_map = await measure_above_anchor(
            uwb, drone, anchor_id, other_ids, CFG["flight_alt"]
        )
        above_dists[anchor_id] = d2d_map
        print(f"  ✅ 앵커{anchor_id} 측정 완료")

    # ── 6. 거리 행렬 구성 ─────────────────────────────────────
    print("\n[4] 거리 행렬 구성...")
    D, labels = build_distance_matrix(anchor_ids, ground_dists, above_dists)
    print(f"  노드: {labels}")
    print_distance_matrix(D, labels)

    # ── 7. Classical MDS ──────────────────────────────────────
    print("\n[5] Classical MDS 자기 위치 추정...")
    X_mds   = classical_mds(D, dims=2)
    rmse    = mds_rmse(D, X_mds)
    print(f"  MDS 재현 RMSE: {rmse:.5f}m")
    if rmse > 0.1:
        print("  ⚠️  RMSE 높음 - 거리 측정 품질 확인 필요")

    # ── 8. NED 좌표계 정렬 ────────────────────────────────────
    print("\n[6] NED 좌표계 정렬...")
    home_idx         = 0
    first_anchor_idx = 1

    brg_to_first = bearing_records[anchor_ids[0]]
    print(f"  HOME → 앵커{anchor_ids[0]} bearing: {brg_to_first:.2f}°")

    X_aligned = align_mds_to_ned(
        X_mds,
        home_idx              = home_idx,
        first_anchor_idx      = first_anchor_idx,
        bearing_home_to_first = brg_to_first,
        reflect_e             = MDS_REFLECT_E,
    )

    calibrated_ne = X_aligned[1:]   # HOME 제외

    # ── 9. 귀환 ───────────────────────────────────────────────
    await return_home_and_land(drone)

    # ── 10. 결과 출력 ─────────────────────────────────────────
    print_result(anchor_ids, calibrated_ne, ANCHOR_POSITIONS)


if __name__ == "__main__":
    asyncio.run(main())