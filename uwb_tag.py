"""
UWB direct tag connection — manual trilateration with Z constraint.

GrowSpace tag 'les' output format:
    ID[ax,ay,az]=dist ... le_us=N est[x,y,z,qf]

Anchor positions are embedded in each line — no manual config needed.
Z is solved by initialising the solver BELOW the anchors.
"""

import math
import re
import threading
import time
from collections import deque

import serial
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy.optimize import least_squares

# ── config ────────────────────────────────────────────────────────────────────
UWB_PORT   = '/dev/ttyUSB0'
UWB_BAUD   = 115200
TRAIL_LEN  = 80
UWB_QF_MIN = 40   # firmware QF threshold (0-100)

COLORS = ['#e74c3c', '#3498db', '#2ecc71', '#f39c12',
          '#9b59b6', '#1abc9c', '#e67e22', '#34495e']


# ── parser ───────────────────────────────────────────────────────────────────
# les format:  'ID[ax,ay,az]=dist ... le_us=N est[x,y,z,qf]'
# DIST format: 'DIST,N,AN0,id,ax,ay,az,dist,...,POS,x,y,z,qf'

_RE_ANCHOR = re.compile(
    r'([0-9A-Fa-f]+)\[([-\d.]+),([-\d.]+),([-\d.]+)\]=([\d.]+)'
)
_RE_EST = re.compile(r'est\[([-\d.]+),([-\d.]+),([-\d.]+),(\d+)\]')


def parse_les(line: str):
    """
    Returns (anchor_info, fw_pos) or (None, None).
    anchor_info: {id: {'pos': (ax,ay,az), 'dist_m': float}}
    fw_pos: (x, y, z, qf)
    """
    anchor_info = {}
    for m in _RE_ANCHOR.finditer(line):
        aid = m.group(1).upper()
        anchor_info[aid] = {
            'pos':    (float(m.group(2)), float(m.group(3)), float(m.group(4))),
            'dist_m': float(m.group(5)),
        }
    if not anchor_info:
        return None, None

    fw_pos = None
    m = _RE_EST.search(line)
    if m:
        fw_pos = (float(m.group(1)), float(m.group(2)),
                  float(m.group(3)), int(m.group(4)))
    return anchor_info, fw_pos


def parse_dist(line: str):
    """
    DIST,<N>,AN0,<id>,<ax>,<ay>,<az>,<dist>,...,POS,<x>,<y>,<z>,<qf>
    Returns (anchor_info, fw_pos) or (None, None).
    """
    if not line.startswith('DIST,'):
        return None, None
    parts = line.split(',')
    try:
        n = int(parts[1])
        anchor_info = {}
        idx = 2
        for _ in range(n):
            # ANx, id, ax, ay, az, dist
            aid = parts[idx + 1].upper()
            ax = float(parts[idx + 2])
            ay = float(parts[idx + 3])
            az = float(parts[idx + 4])
            dist = float(parts[idx + 5])
            anchor_info[aid] = {'pos': (ax, ay, az), 'dist_m': dist}
            idx += 6
        fw_pos = None
        if idx < len(parts) and parts[idx] == 'POS':
            x = float(parts[idx + 1])
            y = float(parts[idx + 2])
            z = float(parts[idx + 3])
            qf = int(parts[idx + 4])
            fw_pos = (x, y, z, qf)
        return anchor_info, fw_pos
    except (ValueError, IndexError):
        return None, None


def parse_line(line: str):
    """Try DIST format first, then les format."""
    anchor_info, fw_pos = parse_dist(line)
    if anchor_info:
        return anchor_info, fw_pos
    return parse_les(line)


# ── trilateration ─────────────────────────────────────────────────────────────

def trilaterate_z_constrained(anchor_info: dict) -> tuple | None:
    """
    anchor_info: {anchor_id: {'pos': (ax,ay,az), 'dist_m': float}}
    Returns (x, y, z) with z guaranteed below all anchor heights.
    """
    pts = np.array([v['pos']   for v in anchor_info.values()])
    dst = np.array([v['dist_m'] for v in anchor_info.values()])

    if len(pts) < 3:
        return None

    x0 = float(np.mean(pts[:, 0]))
    y0 = float(np.mean(pts[:, 1]))
    # start well below anchors → least_squares converges to the lower root
    z0 = float(np.min(pts[:, 2])) - 1.5

    def residuals(p):
        return np.linalg.norm(pts - p, axis=1) - dst

    r = least_squares(residuals, [x0, y0, z0], method='lm')
    x, y, z = r.x

    # safety clamp: if solver drifted above anchors, mirror it down
    anchor_z_min = float(np.min(pts[:, 2]))
    if z > anchor_z_min:
        z = anchor_z_min - abs(z - anchor_z_min)

    return float(x), float(y), float(z)


# ── UWBTag ────────────────────────────────────────────────────────────────────

class UWBTag:
    """
    Reads DIST lines from UWB tag, computes position via trilateration.
    Same public API as UWBListener in manual_control.py:
        start() / stop()
        get_drone_pos() → (x, y, z) relative to origin, or None
        get_tags()      → dict for visualizer
    conn: pymavlink connection for VISION injection (None = visualizer-only mode)
    """

    def __init__(self, conn=None):
        self.conn = conn
        self._lock   = threading.RLock()
        self._stop   = threading.Event()
        self._thread = None

        self._origin   = None      # (x, y, z) at first fix
        self._pos_rel  = None      # (dx, dy, dz) relative to origin
        self._pos_abs  = None      # (x, y, z) absolute UWB frame
        self._anchors  = {}        # {id: {'pos': ..., 'dist_m': ...}} last batch
        self._fw_pos   = None      # (x, y, z, qf) firmware estimate
        self._trail    = deque(maxlen=TRAIL_LEN)
        self._ts       = 0.0
        self._dbg_cnt  = 0

    # ── public API ─────────────────────────────────────────────────────────────

    def start(self):
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()

    def get_drone_pos(self):
        """Returns (x, y, z) relative to origin (ENU), or None."""
        with self._lock:
            return self._pos_rel

    def get_tags(self):
        """Visualizer-compatible snapshot."""
        with self._lock:
            if self._pos_abs is None:
                return {}
            x, y, z = self._pos_abs
            qf = self._fw_pos[3] if self._fw_pos else 0
            return {
                'DRONE': {
                    'x': x, 'y': y, 'z': z,
                    'qf': qf,
                    'ts': self._ts,
                    'trail': list(self._trail),
                    'fw_pos': self._fw_pos,
                    'anchors': {
                        aid: {**v} for aid, v in self._anchors.items()
                    },
                }
            }

    # ── VISION injection ───────────────────────────────────────────────────────

    def _inject_vision(self, x, y, z):
        if self.conn is None:
            return
        # covariance: upper-triangle of 6x6 (pos x,y,z + rot r,p,y)
        # indices 0,6,11 = position variances; 15,18,20 = angle variances
        cov = [0.0] * 21
        cov[0]  = 0.01   # x variance (0.1m std)
        cov[6]  = 0.01   # y variance
        cov[11] = 0.01   # z variance
        cov[15] = 0.1    # roll variance (unused — we send 0)
        cov[18] = 0.1    # pitch variance
        cov[20] = 0.1    # yaw variance
        ts = int(time.time() * 1e6)  # microseconds, as reference impl does
        self.conn.mav.vision_position_estimate_send(
            ts, x, y, z, 0.0, 0.0, 0.0, cov, 0,
        )
        self._dbg_cnt += 1
        if self._dbg_cnt % 10 == 0:
            print(f"  [UWB] vision #{self._dbg_cnt}"
                  f"  pos=({x:.2f},{y:.2f},{z:.2f})")

    # ── serial loop ────────────────────────────────────────────────────────────

    def _run(self):
        rx = ok = 0
        last_stat = time.time()
        NO_DATA_TIMEOUT = 3.0   # seconds before falling back to les polling
        POLL_HZ = 5

        while not self._stop.is_set():
            try:
                with serial.Serial(UWB_PORT, UWB_BAUD, timeout=1.0) as ser:
                    print(f"[UWB] port open: {UWB_PORT} @ {UWB_BAUD}")
                    print("[UWB] listening for auto-stream...")

                    last_data = time.time()
                    polling = False
                    last_poll = 0.0

                    while not self._stop.is_set():
                        # if no data for 3s, switch to les polling mode
                        now = time.time()
                        if not polling and now - last_data > NO_DATA_TIMEOUT:
                            print("[UWB] no stream detected — switching to les polling")
                            ser.write(b'\r\n')
                            time.sleep(0.1)
                            ser.reset_input_buffer()
                            polling = True

                        if polling and now - last_poll >= 1.0 / POLL_HZ:
                            ser.write(b'les\r\n')
                            last_poll = now

                        raw_bytes = ser.readline()
                        if not raw_bytes:
                            continue

                        raw = raw_bytes.decode('ascii', errors='ignore').strip()
                        if not raw or raw in ('dwm>', 'les', 'lec', 'lep'):
                            continue

                        # any real data line resets the no-stream timer
                        last_data = time.time()
                        rx += 1
                        if rx <= 5:
                            print(f"[RAW #{rx:02d}] {raw!r}")

                        anchor_info, fw_pos = parse_line(raw)
                        if not anchor_info:
                            continue
                        ok += 1

                        if fw_pos and fw_pos[3] < UWB_QF_MIN:
                            continue

                        if len(anchor_info) < 3:
                            continue

                        pos = trilaterate_z_constrained(anchor_info)
                        if pos is None:
                            continue

                        if not all(math.isfinite(v) for v in pos):
                            continue

                        x, y, z = pos
                        now = time.time()

                        with self._lock:
                            self._pos_abs = (x, y, z)
                            self._fw_pos  = fw_pos
                            self._ts      = now
                            self._trail.append((x, y))
                            self._anchors = anchor_info

                            if self._origin is None:
                                self._origin = (x, y, z)
                                print(f"[UWB] origin locked: "
                                      f"({x:.2f},{y:.2f},{z:.2f})")
                                self._send_global_origin()

                            ox, oy, oz = self._origin
                            rel_x = x - ox
                            rel_y = y - oy
                            rel_z = -(z - oz)
                            self._pos_rel = (rel_x, rel_y, rel_z)

                        self._inject_vision(rel_x, rel_y, rel_z)

                        now2 = time.time()
                        if now2 - last_stat >= 5.0:
                            fw = (f"fw=({fw_pos[0]:.2f},{fw_pos[1]:.2f},"
                                  f"{fw_pos[2]:.2f}) qf={fw_pos[3]}"
                                  if fw_pos else "fw=N/A")
                            print(f"[UWB] rx={rx} ok={ok} "
                                  f"anchors={len(anchor_info)} "
                                  f"trilat=({x:.2f},{y:.2f},{z:.2f}) "
                                  f"height={-z:.2f}m  {fw}")
                            last_stat = now2

            except serial.SerialException as e:
                print(f"[UWB] error: {e} — retry in 3s")
                time.sleep(3.0)

    def _send_global_origin(self):
        if self.conn is None:
            return
        # lat=0 lon=0 alt=0 — dummy indoor origin
        self.conn.mav.set_gps_global_origin_send(
            self.conn.target_system, 0, 0, 0,
        )
        # also send SET_HOME_POSITION — required by some FC builds
        self.conn.mav.set_home_position_send(
            self.conn.target_system,
            0, 0, 0,        # lat, lon, alt (int32 * 1e7 / mm)
            0.0, 0.0, 0.0,  # local NED position
            [1, 0, 0, 0],   # quaternion (identity)
            0.0, 0.0, 0.0,  # approach vector
            int(time.time() * 1e3),
        )
        print("[UWB] global origin + home position sent")


# ── standalone visualizer ─────────────────────────────────────────────────────

def _update(frame, ax_info, ax_map, uwb):
    now  = time.time()
    tags = uwb.get_tags()

    # ── info panel ───────────────────────────────────────────────────────────
    ax_info.cla()
    ax_info.set_title("Position + anchor distances")
    ax_info.axis('off')

    if not tags:
        ax_info.text(0.5, 0.5, 'Waiting for data...',
                     ha='center', va='center', fontsize=12, color='gray',
                     transform=ax_info.transAxes)
    else:
        info = tags['DRONE']
        age  = now - info['ts']
        fw = info.get('fw_pos')
        fw_str = (f"fw:     ({fw[0]:+.3f}, {fw[1]:+.3f}, {fw[2]:+.3f}) qf={fw[3]}"
                  if fw else "fw:     N/A")
        dz = abs(-info['z'] - (-fw[2])) if fw else 0.0
        pos_lines = [
            f"Trilat: ({info['x']:+.3f}, {info['y']:+.3f}, {-info['z']:+.3f}m)",
            fw_str,
            f"ΔZ (trilat vs fw): {dz:.3f} m",
            f"Age:    {age:.2f} s",
            "",
            "Anchor distances:",
        ]
        for aid, av in sorted(info.get('anchors', {}).items()):
            ax, ay, az = av['pos']
            pos_lines.append(
                f"  {aid}: {av['dist_m']:.3f} m"
                f"  @ ({ax:.2f},{ay:.2f},{az:.2f})"
            )
        ax_info.text(0.05, 0.95, '\n'.join(pos_lines),
                     transform=ax_info.transAxes,
                     va='top', fontsize=9, family='monospace')

    # ── 2D map ───────────────────────────────────────────────────────────────
    ax_map.cla()
    ax_map.set_title("XY trajectory")
    ax_map.set_xlabel("X (m)")
    ax_map.set_ylabel("Y (m)")
    ax_map.set_aspect('equal')
    ax_map.grid(True, linestyle='--', alpha=0.4)

    if not tags:
        ax_map.text(0.5, 0.5, 'Waiting for data...',
                    ha='center', va='center', fontsize=12, color='gray',
                    transform=ax_map.transAxes)
        return

    info  = tags['DRONE']
    fresh = (now - info['ts']) < 1.0
    trail = info['trail']

    # draw anchor positions
    for i, (aid, av) in enumerate(sorted(info.get('anchors', {}).items())):
        ax, ay, _ = av['pos']
        ax_map.plot(ax, ay, 's', color='#555555', markersize=10, zorder=5)
        ax_map.text(ax + 0.1, ay + 0.1, aid, fontsize=7, color='#555555')

    # draw trail
    if len(trail) > 1:
        tx, ty = zip(*trail)
        ax_map.plot(tx, ty, '-', color=COLORS[0], alpha=0.35, linewidth=1)

    # draw drone
    mc = COLORS[0] if fresh else '#aaaaaa'
    ax_map.plot(info['x'], info['y'], 'o', color=mc,
                markersize=12, markeredgecolor='white',
                markeredgewidth=1.5, zorder=10)
    ax_map.text(info['x'] + 0.1, info['y'] + 0.1,
                f"({info['x']:.2f}, {info['y']:.2f})\n"
                f"H={-info['z']:.2f}m",
                fontsize=8, color=mc)

    all_x = [info['x']] + [v['pos'][0] for v in info['anchors'].values()]
    all_y = [info['y']] + [v['pos'][1] for v in info['anchors'].values()]
    if all_x and all_y:
        margin = 1.5
        try:
            ax_map.set_xlim(min(all_x) - margin, max(all_x) + margin)
            ax_map.set_ylim(min(all_y) - margin, max(all_y) + margin)
        except ValueError:
            pass


def main():
    uwb = UWBTag(conn=None)   # no FC — visualizer only
    uwb.start()

    fig, (ax_info, ax_map) = plt.subplots(1, 2, figsize=(13, 5))
    fig.suptitle("UWB Tag — direct trilateration", fontsize=13, fontweight='bold')
    fig.ani = animation.FuncAnimation(
        fig, _update, fargs=(ax_info, ax_map, uwb),
        interval=200, cache_frame_data=False,
    )
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()
