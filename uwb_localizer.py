"""
uwb_localizer.py — DWM3001C UWB 태그 시리얼 리더 + EKF 측위

DWM3001C 텍스트 모드 출력:
  DIST,3,AN0,8182,2.18,0.00,0.00,0.83,...,POS,x,y,z

측위 우선순위:
  1) DWM 온칩 POS → NED 변환
  2) EKF — 원시 거리 직접 융합 (속도 상태 + 자이로/속도 보조)
  3) trilaterate() — EKF 미초기화 시 초기 추정

SITL 노이즈 모델 (현실 반영):
  ① 최대 가시거리 제한: LOS 60m
  ② 앙각 사각지대: 앵커 바로 위 > 82° 신호 끊김, > 65° 노이즈 3배
  ③ 지면 멀티패스: 고도 < 1.5m 시 주기적 바이어스
  ④ 앵커 수 비례 Hz: 4앵커 → ~6Hz (DWM3001C TWR 슬롯)
  ⑤ 순차 측정 (Sequential Ranging): 각 앵커 측정 시각 3ms 간격
  ⑥ 앵커 투하 위치 오차: ±15cm Gaussian 착지 오차
  ⑦ 앵커 기울어짐: ±5cm 높이 불확실도
  ⑧ 속도 보조 EKF: 비행 컨트롤러 속도로 prediction 개선
  ⑨ 앵커 잔차 추적: 앵커별 측정 품질 모니터링 (자기보정 기반)

좌표계: NED  (North↑  East→  Down↓, alt 음수 = 위)
"""

import math
import random
import threading
import time
import struct

try:
    import serial
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False

try:
    import numpy as np
    from scipy.optimize import minimize
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False


# ════════════════════════════════════════════════════════════════
#  SITL 물리 상수
# ════════════════════════════════════════════════════════════════

MAX_RANGE_LOS      = 60.0   # m — LOS 최대 측정거리 (DWM3001C 사양)
ELEV_DEAD_ZONE     = 82.0   # deg — 앵커 바로 위 사각지대 (안테나 null)
ELEV_HIGH_NOISE    = 65.0   # deg — 이 이상부터 노이즈 점진 증가
GROUND_MULTIPATH_H = 1.5    # m — 지면 멀티패스 영향 고도 (NED abs)
TWR_SLOT_SEC       = 0.003  # s — DWM3001C TWR 1회 슬롯 시간
TWR_EPOCH_OVERHEAD = 0.020  # s — 에포크 처리 오버헤드


# ════════════════════════════════════════════════════════════════
#  Trilateration (EKF 초기화용 fallback)
# ════════════════════════════════════════════════════════════════

def trilaterate(anchors_dist: list) -> tuple | None:
    """
    anchors_dist: [(n, e, z, dist_m), ...]  최소 4개 권장
    반환: (n, e, z), 실패 시 None
    """
    if not HAS_SCIPY or len(anchors_dist) < 3:
        return None
    pts = np.array([[a[0], a[1], a[2]] for a in anchors_dist])
    ds  = np.array([a[3] for a in anchors_dist])
    x0  = np.mean(pts, axis=0)

    def cost(x):
        return sum((np.linalg.norm(x - p) - d) ** 2
                   for p, d in zip(pts, ds))

    res = minimize(cost, x0, method="Nelder-Mead",
                   options={"xatol": 0.01, "fatol": 0.01, "maxiter": 2000})
    if res.success or res.fun < 0.1:
        return float(res.x[0]), float(res.x[1]), float(res.x[2])
    return None


def trilaterate_2d(anchors_dist: list, z: float = 0.0) -> tuple | None:
    """Z 고정 2D 삼변측량. anchors_dist: [(n, e, z_anch, dist_m), ...]"""
    if not HAS_SCIPY or len(anchors_dist) < 3:
        return None
    pts = np.array([[a[0], a[1]] for a in anchors_dist])
    ds  = np.array([math.sqrt(max(a[3]**2 - (z - a[2])**2, 0.01))
                    for a in anchors_dist])
    x0  = np.mean(pts, axis=0)

    def cost(x):
        return sum((np.linalg.norm(x - p) - d) ** 2
                   for p, d in zip(pts, ds))

    res = minimize(cost, x0, method="Nelder-Mead",
                   options={"xatol": 0.005, "fatol": 0.005, "maxiter": 2000})
    if res.success or res.fun < 0.1:
        return float(res.x[0]), float(res.x[1]), z
    return None


def compute_gdop(anchors: list, pos: tuple) -> float:
    """GDOP 계산. 낮을수록 앵커 기하학이 좋음. 계산 불가 시 999."""
    if not HAS_SCIPY or len(anchors) < 4:
        return 999.0
    try:
        pn, pe, pz = pos
        H = []
        for (an, ae, az) in anchors:
            d = math.sqrt((an-pn)**2 + (ae-pe)**2 + (az-pz)**2)
            if d < 0.01:
                continue
            H.append([(an-pn)/d, (ae-pe)/d, (az-pz)/d, 1.0])
        if len(H) < 4:
            return 999.0
        A = np.array(H)
        Q = np.linalg.inv(A.T @ A)
        return float(math.sqrt(np.trace(Q)))
    except Exception:
        return 999.0


# ════════════════════════════════════════════════════════════════
#  EKF — 거리 측정값 직접 융합 + 속도 보조
# ════════════════════════════════════════════════════════════════

class UWBPositionEKF:
    """
    Extended Kalman Filter: UWB 거리 측정 직접 융합.

    상태: x = [n, e, z, vn, ve, vz]
    거리 측정: h_i(x) = ||x[:3] - anchor_i||
    속도 측정: h_v(x) = x[3:6]  (비행 컨트롤러 속도 직접 주입)

    특징:
      - 속도 보조 예측: FC 속도로 position propagation 개선
      - Sequential 이상치 거부: 3σ gate
      - 앵커별 잔차 로그 → 앵커 건강 모니터링
    """

    def __init__(self,
                 pos0: tuple = (0., 0., 0.),
                 sigma_pos: float = 0.05,
                 sigma_proc_pos: float = 0.1,
                 sigma_proc_vel: float = 0.5,
                 outlier_gate: float = 3.0):
        self.initialized   = False
        self._outlier_gate = outlier_gate
        self._r            = sigma_pos ** 2
        self._qp           = sigma_proc_pos
        self._qv           = sigma_proc_vel
        self._t            = None

        if HAS_SCIPY:
            self.x = np.array([pos0[0], pos0[1], pos0[2],
                                0., 0., 0.], dtype=float)
            self.P = np.diag([5., 5., 5., 1., 1., 1.])

    def _ok(self):
        return HAS_SCIPY and hasattr(self, 'x')

    def predict(self, dt: float):
        """등속도 예측 (외부 속도 없을 때)."""
        if not self._ok() or dt <= 0:
            return
        dt = min(dt, 1.0)
        self.x[0] += dt * self.x[3]
        self.x[1] += dt * self.x[4]
        self.x[2] += dt * self.x[5]
        F = np.eye(6)
        F[0, 3] = F[1, 4] = F[2, 5] = dt
        Q = np.diag([self._qp * dt**2] * 3 + [self._qv * dt] * 3)
        self.P = F @ self.P @ F.T + Q

    def predict_with_vel(self, dt: float, vel: np.ndarray):
        """
        속도 보조 예측 — 비행 컨트롤러 속도(IMU 적분)를 직접 사용.
        내부 속도 상태보다 FC 속도가 신뢰도 높을 때 호출.
        """
        if not self._ok() or dt <= 0:
            return
        dt = min(dt, 1.0)
        # 위치: 알려진 속도로 전파
        self.x[0] += dt * vel[0]
        self.x[1] += dt * vel[1]
        self.x[2] += dt * vel[2]
        # 속도 상태를 FC 속도로 업데이트 (속도 불확실도 감소)
        self.x[3:6] = vel
        F = np.eye(6)
        F[0, 3] = F[1, 4] = F[2, 5] = dt
        # 속도를 알기 때문에 vel 프로세스 노이즈 훨씬 작게
        Q = np.diag([self._qp * dt**2] * 3 + [0.02 * dt] * 3)
        self.P = F @ self.P @ F.T + Q

    def update(self, anchor_pos: np.ndarray, measured_dist: float) -> float:
        """
        단일 거리 측정으로 갱신.
        반환: 혁신량 (innovation), 이상치 거부 시 None
        """
        if not self._ok():
            return None
        dp        = self.x[:3] - anchor_pos
        pred_dist = float(np.linalg.norm(dp))
        if pred_dist < 0.01:
            pred_dist = 0.01
        H     = np.zeros(6)
        H[:3] = dp / pred_dist
        S     = float(H @ self.P @ H) + self._r
        innov = measured_dist - pred_dist
        # 3σ 이상치 거부
        if abs(innov) > self._outlier_gate * math.sqrt(S):
            return None
        K      = self.P @ H / S
        self.x += K * innov
        self.P  = (np.eye(6) - np.outer(K, H)) @ self.P
        return innov

    def update_velocity(self, vel: np.ndarray, sigma_vel: float = 0.1):
        """비행 컨트롤러 속도로 속도 상태 직접 보정."""
        if not self._ok():
            return
        for i in range(3):
            H    = np.zeros(6)
            H[3+i] = 1.0
            S    = float(H @ self.P @ H) + sigma_vel ** 2
            K    = self.P @ H / S
            self.x += K * (vel[i] - self.x[3+i])
            self.P  = (np.eye(6) - np.outer(K, H)) @ self.P

    def initialize_from_trilateration(self, pos: tuple):
        if not self._ok():
            return
        self.x[:3] = [pos[0], pos[1], pos[2]]
        self.x[3:] = 0.
        self.P     = np.diag([1., 1., 1., 0.5, 0.5, 0.5])
        self.initialized = True
        self._t    = time.monotonic()

    def get_position(self) -> tuple:
        if not self._ok():
            return (0., 0., 0.)
        return float(self.x[0]), float(self.x[1]), float(self.x[2])

    def get_velocity(self) -> tuple:
        if not self._ok():
            return (0., 0., 0.)
        return float(self.x[3]), float(self.x[4]), float(self.x[5])

    def get_pos_uncertainty(self) -> tuple:
        """위치 1σ 불확실도 (m)."""
        if not self._ok():
            return (999., 999., 999.)
        return (float(math.sqrt(abs(self.P[0, 0]))),
                float(math.sqrt(abs(self.P[1, 1]))),
                float(math.sqrt(abs(self.P[2, 2]))))


# ════════════════════════════════════════════════════════════════
#  DWM3001C 파서
# ════════════════════════════════════════════════════════════════

def parse_dist_line(line: str) -> dict | None:
    """
    'DIST,3,AN0,8182,2.18,0.00,0.00,0.83,...,POS,x,y,z' 파싱
    반환: {'distances': {id: {dist_m, pos}}, 'pos': (x,y,z) or None}
    """
    if not line.startswith("DIST,"):
        return None
    try:
        parts       = line.split(",")
        num_anchors = int(parts[1])
        distances   = {}
        idx         = 2
        for _ in range(num_anchors):
            anchor_id = parts[idx + 1]
            ax = float(parts[idx + 2])
            ay = float(parts[idx + 3])
            az = float(parts[idx + 4])
            dist = float(parts[idx + 5])
            distances[anchor_id] = {
                "name"  : parts[idx],
                "dist_m": dist,
                "pos"   : (ax, ay, az),
            }
            idx += 6
        pos = None
        if idx < len(parts) and parts[idx] == "POS":
            pos = (float(parts[idx+1]),
                   float(parts[idx+2]),
                   float(parts[idx+3]))
        return {"distances": distances, "pos": pos}
    except (IndexError, ValueError):
        return None


def parse_tlv_response(data: bytes) -> dict | None:
    """DWM3001C TLV 바이너리 응답 파싱."""
    result = {"pos": None, "distances": {}}
    i = 0
    while i + 2 <= len(data):
        t      = data[i]
        length = data[i + 1]
        if i + 2 + length > len(data):
            break
        val = data[i + 2: i + 2 + length]
        if t == 0x41 and length >= 13:
            x, y, z = struct.unpack_from("<iii", val, 0)
            qf = val[12]
            result["pos"] = (x/1000., y/1000., z/1000., qf)
        elif t == 0x49 and length >= 1:
            cnt = val[0]
            off = 1
            for _ in range(cnt):
                if off + 20 > len(val):
                    break
                addr    = struct.unpack_from("<H", val, off)[0]
                dist_mm = struct.unpack_from("<I", val, off + 2)[0]
                qf      = val[off + 6]
                ax, ay, az = struct.unpack_from("<iii", val, off + 7)
                result["distances"][f"{addr:04X}"] = {
                    "dist_m": dist_mm / 1000.,
                    "pos"   : (ax/1000., ay/1000., az/1000.),
                    "qf"    : qf,
                }
                off += 20
        i += 2 + length
    return result if (result["pos"] or result["distances"]) else None


# ════════════════════════════════════════════════════════════════
#  UWBLocalizer
# ════════════════════════════════════════════════════════════════

class UWBLocalizer:
    """
    DWM3001C 시리얼 읽기 + EKF 측위.

    get_position()  → (n, e, z) NED m
    get_distances() → {anchor_id: dist_m}
    get_quality()   → GDOP, σ, 속도, 앵커 건강
    set_drone_velocity(vn, ve, vz) → EKF 예측 개선
    add_sitl_anchor(n, e, z, uncertainty) → 미션 중 신규 앵커 추가
    """

    def __init__(self,
                 port: str = "/dev/ttyACM0",
                 baud: int = 115200,
                 mode: str = "text",
                 sitl: bool = False,
                 sitl_anchors: list = None,
                 anchor_ned_map: dict = None,
                 sitl_drop_rate: float = 0.15,
                 sitl_nlos_rate: float = 0.05,
                 ekf_sigma_pos: float = 0.05,
                 ekf_sigma_proc: float = 0.3):

        self.port  = port
        self.baud  = baud
        self.mode  = mode
        self.sitl  = sitl
        self._anchor_ned_map = anchor_ned_map or {}

        self._lock        = threading.Lock()
        self._pos         = None
        self._dists       = {}
        self._raw_anchors = {}
        self._running     = False
        self._ser         = None

        # SITL 기본 앵커 목록 [(n, e, z), ...]
        # — 각각 독립적인 deploy offset 보유
        self._sitl_anchors       = list(sitl_anchors or [])
        self._sitl_drone         = (0., 0., 0.)
        self._drone_vel          = (0., 0., 0.)   # FC 속도 (속도 보조용)
        self._sitl_drop          = sitl_drop_rate
        self._sitl_nlos          = sitl_nlos_rate

        # 앵커 투하 오차: [(dn, de, dz), ...]  인덱스로 sitl_anchors와 대응
        # 기존 앵커는 오차 없음 (미리 설치된 위치 정확), 신규 투하만 오차 부여
        self._anchor_deploy_offsets = [(0., 0., 0.)] * len(self._sitl_anchors)

        # EKF
        self._ekf = UWBPositionEKF(
            sigma_pos      = ekf_sigma_pos,
            sigma_proc_pos = ekf_sigma_proc,
            sigma_proc_vel = ekf_sigma_proc * 3,
        )

        # 품질 지표
        self._gdop    = 999.
        self._n_used  = 0

        # 앵커별 잔차 히스토리 {anchor_id: [innov, ...]}
        # — 앵커 건강 모니터링 + 자기보정 기반 데이터
        self._anchor_residuals: dict[str, list] = {}
        self._RESIDUAL_HISTORY = 60  # 최근 N회 잔차 보관

    # ── 시작/종료 ─────────────────────────────────────────────────

    def start(self):
        self._running = True
        if self.sitl:
            threading.Thread(target=self._sitl_loop, daemon=True).start()
        elif HAS_SERIAL:
            try:
                self._ser = serial.Serial(self.port, self.baud, timeout=1)
                threading.Thread(target=self._serial_loop, daemon=True).start()
                print(f"[UWB] 시리얼 시작: {self.port} @ {self.baud}")
            except Exception as e:
                print(f"[UWB] 시리얼 열기 실패: {e}")
        else:
            print("[UWB] pyserial 없음 (pip install pyserial)")

    def stop(self):
        self._running = False
        if self._ser:
            self._ser.close()

    # ── 데이터 조회 ───────────────────────────────────────────────

    def get_position(self) -> tuple | None:
        with self._lock:
            return self._pos

    def get_distances(self) -> dict:
        with self._lock:
            return dict(self._dists)

    def get_quality(self) -> dict:
        """
        {
          'gdop'        : float,
          'sigma'       : (sn, se, sz),
          'velocity'    : (vn, ve, vz),
          'n_anchors'   : int,
          'anchor_health': {id: {'mean': float, 'std': float, 'ok': bool}},
        }
        """
        health = {}
        for aid, hist in self._anchor_residuals.items():
            if len(hist) < 5:
                continue
            mean = sum(hist) / len(hist)
            std  = math.sqrt(sum((x-mean)**2 for x in hist) / len(hist))
            # 평균 잔차 > 0.3m = NLOS/오배치 의심
            # std > 0.5m = 불안정한 측정
            health[aid] = {
                "mean": round(mean, 3),
                "std" : round(std,  3),
                "ok"  : abs(mean) < 0.3 and std < 0.5,
            }
        return {
            "gdop"         : self._gdop,
            "sigma"        : self._ekf.get_pos_uncertainty(),
            "velocity"     : self._ekf.get_velocity(),
            "n_anchors"    : self._n_used,
            "anchor_health": health,
        }

    # ── 외부 상태 주입 ────────────────────────────────────────────

    def set_sitl_drone_pos(self, n: float, e: float, z: float):
        """SITL: 드론 위치 갱신 (NED, z 음수 = 위)."""
        self._sitl_drone = (n, e, z)

    def set_drone_velocity(self, vn: float, ve: float, vz: float):
        """
        비행 컨트롤러 속도 주입 (NED m/s).
        EKF predict_with_vel() 에서 사용 → 측정 사이 구간 예측 정확도 향상.
        """
        self._drone_vel = (vn, ve, vz)

    def add_sitl_anchor(self, n: float, e: float, z: float = 0.,
                        pos_uncertainty: float = 0.15,
                        tilt_uncertainty: float = 0.05):
        """
        SITL 앵커 동적 추가 (미션 중 투하 시 호출).

        pos_uncertainty  : 수평 착지 오차 1σ (m), 기본 ±15cm
        tilt_uncertainty : 수직 오차 1σ (m),  기본 ±5cm (기울어짐)

        실제 착지 위치 = (n, e, z) + Gaussian 오차
        but 시스템이 아는 위치는 (n, e, z) — 이 차이가 측위 바이어스 유발
        """
        with self._lock:
            self._sitl_anchors.append((n, e, z))
            dn = random.gauss(0., pos_uncertainty)
            de = random.gauss(0., pos_uncertainty)
            dz = random.gauss(0., tilt_uncertainty)
            self._anchor_deploy_offsets.append((dn, de, dz))
            idx = len(self._sitl_anchors) - 1
        print(f"[UWB] 앵커 SIM{idx} 추가 "
              f"@ ({n:.2f},{e:.2f},{z:.2f})"
              f"  실제 착지 오차: Δ({dn:.3f},{de:.3f},{dz:.3f})m")

    # ── 시리얼 루프 ───────────────────────────────────────────────

    def _serial_loop(self):
        buf = b""
        while self._running:
            try:
                if self.mode == "text":
                    line = (self._ser.readline()
                            .decode("utf-8", errors="ignore").strip())
                    if line:
                        self._process_text(line)
                else:
                    chunk  = self._ser.read(256)
                    buf   += chunk
                    result = parse_tlv_response(buf)
                    if result:
                        self._apply_result(result)
                        buf = b""
            except Exception as e:
                if self._running:
                    print(f"[UWB] 시리얼 에러: {e}")
                    time.sleep(0.5)

    def _process_text(self, line: str):
        result = parse_dist_line(line)
        if result:
            self._apply_result(result)

    # ── 결과 적용 ─────────────────────────────────────────────────

    def _apply_result(self, result: dict):
        distances = result.get("distances", {})
        raw_pos   = result.get("pos")

        with self._lock:
            for aid, info in distances.items():
                self._dists[aid] = info["dist_m"]
                self._raw_anchors[aid] = info.get("pos")
            self._n_used = len(distances)

        # 위치 결정
        pos = None
        if raw_pos:
            pos = self._uwb_to_ned(raw_pos[0], raw_pos[1], raw_pos[2])
            if not self._ekf.initialized and HAS_SCIPY:
                self._ekf.initialize_from_trilateration(pos)
            elif HAS_SCIPY:
                self._ekf_fuse(distances)
                pos = self._ekf.get_position()
        else:
            pos = self._run_ekf(distances)

        if pos:
            with self._lock:
                self._pos = pos

        # GDOP
        if pos and HAS_SCIPY:
            al = [self._get_anchor_ned(aid, info.get("pos"))
                  for aid, info in distances.items()]
            al = [a for a in al if a]
            if len(al) >= 4:
                self._gdop = compute_gdop(al, pos)

    def _run_ekf(self, distances: dict):
        if not HAS_SCIPY:
            return self._do_trilaterate(distances)
        if not self._ekf.initialized:
            ad = self._build_anchors_dist(distances)
            p  = (trilaterate(ad) if len(ad) >= 4
                  else trilaterate_2d(ad, self._pos[2] if self._pos else 0.))
            if p:
                self._ekf.initialize_from_trilateration(p)
            else:
                return None
        self._ekf_fuse(distances)
        return self._ekf.get_position()

    def _ekf_fuse(self, distances: dict):
        """EKF: predict (속도 보조) → 거리 업데이트 → 잔차 기록."""
        now = time.monotonic()
        dt  = (now - self._ekf._t) if self._ekf._t else 0.1
        self._ekf._t = now

        vn, ve, vz = self._drone_vel
        spd = math.sqrt(vn**2 + ve**2 + vz**2)
        if spd > 0.05:
            # 속도 보조 예측
            self._ekf.predict_with_vel(dt, np.array([vn, ve, vz]))
        else:
            self._ekf.predict(dt)

        for aid, info in distances.items():
            ap = self._get_anchor_ned(aid, info.get("pos"))
            if not ap:
                continue
            innov = self._ekf.update(np.array(ap, dtype=float),
                                     info["dist_m"])
            # 잔차 기록 (앵커 건강 모니터링용)
            if innov is not None:
                hist = self._anchor_residuals.setdefault(aid, [])
                hist.append(innov)
                if len(hist) > self._RESIDUAL_HISTORY:
                    hist.pop(0)

    def _do_trilaterate(self, distances: dict):
        ad = self._build_anchors_dist(distances)
        if len(ad) >= 4:
            return trilaterate(ad)
        elif len(ad) >= 3:
            z = self._pos[2] if self._pos else 0.
            return trilaterate_2d(ad, z)
        return None

    def _build_anchors_dist(self, distances: dict) -> list:
        out = []
        for aid, info in distances.items():
            ap = self._get_anchor_ned(aid, info.get("pos"))
            if ap:
                out.append((*ap, info["dist_m"]))
        return out

    def _get_anchor_ned(self, anchor_id: str, uwb_pos=None):
        if anchor_id in self._anchor_ned_map:
            return self._anchor_ned_map[anchor_id]
        if uwb_pos:
            return float(uwb_pos[0]), float(uwb_pos[1]), float(uwb_pos[2])
        return None

    def _uwb_to_ned(self, x, y, z):
        return (float(x), float(y), float(z))

    # ── SITL 시뮬 루프 ────────────────────────────────────────────

    def _sitl_loop(self):
        """
        현실적 UWB SITL 시뮬레이션.

        ① 최대 측정거리 (LOS 60m)
        ② 앙각 사각지대 (> 82°)
        ③ 지면 멀티패스 (고도 < 1.5m)
        ④ 앵커 수 비례 Hz (DWM3001C TWR 슬롯)
        ⑤ 순차 측정 — 각 앵커 3ms 간격, 드론 위치 변화 반영
        ⑥⑦ 앵커 투하 오차 / 기울어짐 (deploy_offsets)
        """
        while self._running:
            with self._lock:
                anchors  = list(self._sitl_anchors)
                offsets  = list(self._anchor_deploy_offsets)
            n_anch = len(anchors)
            if n_anch == 0:
                time.sleep(0.05)
                continue

            dn0, de0, dz0 = self._sitl_drone
            vn,  ve,  vz  = self._drone_vel

            distances = {}

            for i, (an, ae, az) in enumerate(anchors):
                # ⑤ 순차 측정: 이 앵커는 epoch 시작으로부터 i*3ms 후 측정
                t_off = i * TWR_SLOT_SEC
                dn = dn0 + vn * t_off
                de = de0 + ve * t_off
                dz = dz0 + vz * t_off

                # ① 패킷 드롭
                if random.random() < self._sitl_drop:
                    continue

                # ⑥⑦ 실제 앵커 위치 = 계획 위치 + 투하 오차
                off = offsets[i] if i < len(offsets) else (0., 0., 0.)
                true_an = an + off[0]
                true_ae = ae + off[1]
                true_az = az + off[2]

                horiz     = math.sqrt((dn-true_an)**2 + (de-true_ae)**2)
                vert_diff = abs(dz - true_az)
                true_dist = math.sqrt(horiz**2 + vert_diff**2)

                # ① 최대 가시거리
                if true_dist > MAX_RANGE_LOS:
                    continue

                # ② 앙각 계산
                elev_deg = math.degrees(
                    math.atan2(vert_diff, max(horiz, 0.001)))

                # ② 사각지대
                if elev_deg > ELEV_DEAD_ZONE:
                    continue

                # 기본 노이즈 σ (거리 비례)
                sigma = 0.03 + 0.005 * true_dist

                # ② 앙각 높을수록 노이즈 증가
                if elev_deg > ELEV_HIGH_NOISE:
                    ratio  = ((elev_deg - ELEV_HIGH_NOISE)
                              / (ELEV_DEAD_ZONE - ELEV_HIGH_NOISE))
                    sigma *= 1. + 4. * ratio

                # ③ 지면 멀티패스 (NED: dz 음수 = 위, abs(dz) = 고도)
                drone_height = abs(dz)
                multipath_bias = 0.
                if drone_height < GROUND_MULTIPATH_H:
                    # UWB 중심 파장 ~15cm 기준 간섭 패턴
                    phase = 2 * math.pi * horiz / 0.15
                    amp   = 0.05 * (1. - drone_height / GROUND_MULTIPATH_H)
                    multipath_bias = amp * math.sin(phase)

                # NLOS 바이어스
                nlos_bias = 0.
                if random.random() < self._sitl_nlos:
                    nlos_bias = random.uniform(0.2, 1.5)

                noisy = (true_dist
                         + random.gauss(0., sigma)
                         + nlos_bias
                         + multipath_bias)

                distances[f"SIM{i}"] = {
                    "dist_m": max(noisy, 0.01),
                    "pos"   : (an, ae, az),  # 시스템이 아는 위치 (오차 포함 X)
                    "nlos"  : nlos_bias > 0,
                }

            if distances:
                self._apply_result({"distances": distances, "pos": None})

            # ④ 에포크 주기: 앵커 수 비례 (4앵커 → ~6Hz)
            epoch_period = (n_anch * TWR_SLOT_SEC
                            + TWR_EPOCH_OVERHEAD
                            + random.uniform(-0.005, 0.01))
            time.sleep(max(epoch_period, 0.02))


# ════════════════════════════════════════════════════════════════
#  단독 테스트
# ════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    print("=== UWB Localizer SITL 테스트 (현실 노이즈 + EKF) ===\n")

    # 앵커 4개: 높이 0.5m
    anchors = [(0., 0., 0.5), (10., 0., 0.5),
               (10., 10., 0.5), (0., 10., 0.5)]

    loc = UWBLocalizer(sitl=True, sitl_anchors=anchors,
                       sitl_drop_rate=0.15, sitl_nlos_rate=0.05)
    TRUE = (3., 4., -2.5)    # 실제 드론 위치 (NED)
    VEL  = (0.3, 0.1, 0.)    # 속도 m/s

    loc.set_sitl_drone_pos(*TRUE)
    loc.set_drone_velocity(*VEL)
    loc.start()

    print(f"  실제: N={TRUE[0]:.2f} E={TRUE[1]:.2f} Z={TRUE[2]:.2f}  "
          f"속도: vn={VEL[0]:.1f} ve={VEL[1]:.1f}")
    print()

    for step in range(10):
        time.sleep(0.4)
        pos  = loc.get_position()
        qual = loc.get_quality()
        if pos:
            err = math.sqrt((pos[0]-TRUE[0])**2 + (pos[1]-TRUE[1])**2)
            sn, se, sz = qual["sigma"]
            vn, ve, vz = qual["velocity"]
            print(f"  [{step+1:2d}] pos=({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f})"
                  f"  Δxy={err:.2f}m"
                  f"  σ=({sn:.2f},{se:.2f},{sz:.2f})"
                  f"  vel=({vn:.2f},{ve:.2f})"
                  f"  GDOP={qual['gdop']:.1f}"
                  f"  {qual['n_anchors']}anch")
        else:
            nd = len(loc.get_distances())
            print(f"  [{step+1:2d}] 위치 미확정 ({nd}앵커 수신)")

    # 앵커 1개 동적 추가 (임무 중 투하 시뮬)
    print("\n  → 앵커 투하 시뮬 (SIM4 추가)...")
    loc.add_sitl_anchor(5., 5., 0.5)
    time.sleep(1.0)

    pos  = loc.get_position()
    qual = loc.get_quality()
    print(f"\n  앵커 추가 후: {qual['n_anchors']}앵커"
          f"  GDOP={qual['gdop']:.1f}")
    for aid, h in qual["anchor_health"].items():
        ok = "✓" if h["ok"] else "⚠"
        print(f"    {ok} {aid}: mean={h['mean']:+.3f}m  std={h['std']:.3f}m")

    loc.stop()
    print("\n테스트 완료")
