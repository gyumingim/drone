"""
GrowSpace UWB Listener - real-time tag position visualizer

UWB data via UWBTag (uwb_tag.py) — auto-stream les format, trilateration.
FC comparison: connects to flight controller via MAVLink and overlays
LOCAL_POSITION_NED (x,y) on the same map to verify UWB <-> EKF alignment.
"""

import math
import threading
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from uwb_tag import UWBTag

try:
    from pymavlink import mavutil
    _MAVLINK_OK = True
except ImportError:
    _MAVLINK_OK = False
    print("[FC] pymavlink not found — FC overlay disabled")

FC_CONN = '/dev/ttyACM0'
FC_BAUD = 57600
STALE_SEC = 1.0

COLORS = ['#e74c3c', '#3498db', '#2ecc71', '#f39c12',
          '#9b59b6', '#1abc9c', '#e67e22', '#34495e']

FC_COLOR = '#f39c12'
UWB_COLOR = '#e74c3c'

# ── FC shared state ──────────────────────────────────────────────────────────
_fc_pos = None       # (x, y, z, ts) or None — raw LOCAL_NED
_fc_lock = threading.Lock()

# ── UWB origin offset (first 5 readings avg) ─────────────────────────────────
_uwb_init_samples = []   # raw (x, y) accumulator
_uwb_offset = None       # (ox, oy) added to FC to align frames
_uwb_offset_lock = threading.Lock()


# ── FC MAVLink thread ────────────────────────────────────────────────────────

def _fc_connect():
    candidates = [FC_CONN, '/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyACM2']
    for p in dict.fromkeys(candidates):
        try:
            print(f"[FC] trying {p}...")
            conn = mavutil.mavlink_connection(p, baud=FC_BAUD)
            conn.wait_heartbeat(timeout=3)
            print(f"[FC] connected on {p} (sysid={conn.target_system})")
            return conn
        except Exception:
            pass
    return None


def _fc_read_thread(conn):
    """Read LOCAL_POSITION_NED from an already-connected FC."""
    global _fc_pos
    for stream in [
        mavutil.mavlink.MAV_DATA_STREAM_POSITION,
        mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
        mavutil.mavlink.MAV_DATA_STREAM_EXTRA3,
    ]:
        conn.mav.request_data_stream_send(
            conn.target_system, conn.target_component, stream, 10, 1,
        )
    _cnt = 0
    while True:
        try:
            msg = conn.recv_match(blocking=True, timeout=1.0)
            if msg is None:
                continue
            if msg.get_type() == 'LOCAL_POSITION_NED':
                with _fc_lock:
                    _fc_pos = (msg.x, msg.y, msg.z, time.time())
                _cnt += 1
                if _cnt % 20 == 0:
                    print(f"[FC] LOCAL_NED"
                          f"  x={msg.x:.2f}"
                          f"  y={msg.y:.2f}"
                          f"  z={msg.z:.2f}")
        except Exception as e:
            print(f"[FC] read error: {e}")
            with _fc_lock:
                _fc_pos = None
            break


# ── plot ─────────────────────────────────────────────────────────────────────

def _build_figure():
    fig = plt.figure(figsize=(16, 6))
    fig.suptitle("UWB vs FC LOCAL_NED — Frame Alignment Check",
                 fontsize=13, fontweight='bold')
    ax_info = fig.add_subplot(1, 3, 1)
    ax_map = fig.add_subplot(1, 3, 2)
    ax_drift = fig.add_subplot(1, 3, 3)
    ax_map.set_aspect('equal')
    ax_map.grid(True, linestyle='--', alpha=0.4)
    return fig, ax_info, ax_map, ax_drift


def _update(frame, ax_info, ax_map, ax_drift, uwb):
    global _uwb_offset

    now = time.time()
    tags = uwb.get_tags()  # {'DRONE': {...}} or {}

    with _fc_lock:
        fc = _fc_pos

    # ── collect first 5 UWB samples to compute frame offset ──────────────────
    with _uwb_offset_lock:
        if _uwb_offset is None and tags:
            info = tags['DRONE']
            if (now - info['ts']) < STALE_SEC:
                _uwb_init_samples.append((info['x'], info['y']))
                if len(_uwb_init_samples) >= 5:
                    ox = sum(s[0] for s in _uwb_init_samples) / 5
                    oy = sum(s[1] for s in _uwb_init_samples) / 5
                    _uwb_offset = (ox, oy)
                    print(f"[VIS] UWB offset locked: "
                          f"({ox:.3f}, {oy:.3f}) — adding to FC LOCAL_NED")
        offset = _uwb_offset

    # ── info panel ───────────────────────────────────────────────────────────
    ax_info.cla()
    ax_info.set_title("UWB trilateration")
    ax_info.axis('off')

    if tags:
        info = tags['DRONE']
        fw = info.get('fw_pos')
        age = now - info['ts']
        fw_str = (
            f"fw: ({fw[0]:+.2f},{fw[1]:+.2f},{fw[2]:+.2f}) qf={fw[3]}"
            if fw else "fw: N/A"
        )
        lines = [
            f"Trilat: ({info['x']:+.3f}, {info['y']:+.3f},"
            f" {-info['z']:+.3f} m)",
            fw_str,
            f"Age:    {age:.2f} s",
            "",
            "Anchor distances:",
        ]
        for aid, av in sorted(info.get('anchors', {}).items()):
            ax, ay, az = av['pos']
            lines.append(
                f"  {aid}: {av['dist_m']:.3f} m"
                f"  @ ({ax:.2f},{ay:.2f},{az:.2f})"
            )
        ax_info.text(0.05, 0.95, '\n'.join(lines),
                     transform=ax_info.transAxes,
                     va='top', fontsize=8, family='monospace')
    else:
        ax_info.text(0.5, 0.6, 'Waiting for UWB...',
                     ha='center', va='center', fontsize=11, color='gray',
                     transform=ax_info.transAxes)

    # FC status below
    if fc:
        fc_age = now - fc[3]
        fc_str = (f"FC LOCAL_NED\n"
                  f"x={fc[0]:+.2f}  y={fc[1]:+.2f}\n"
                  f"z={fc[2]:+.2f}  age={fc_age:.1f}s")
        fc_color = FC_COLOR if fc_age < STALE_SEC else '#aaaaaa'
    else:
        fc_str = "FC: not connected"
        fc_color = '#aaaaaa'
    ax_info.text(0.5, 0.05, fc_str,
                 ha='center', va='bottom', fontsize=9, color=fc_color,
                 transform=ax_info.transAxes,
                 bbox=dict(boxstyle='round',
                           facecolor=FC_COLOR + '22',
                           edgecolor=FC_COLOR))

    # ── 2D map ───────────────────────────────────────────────────────────────
    ax_map.cla()
    ax_map.set_title("XY overlay  o UWB  s FC LOCAL_NED")
    ax_map.set_xlabel("X (m)")
    ax_map.set_ylabel("Y (m)")
    ax_map.set_aspect('equal')
    ax_map.grid(True, linestyle='--', alpha=0.4)

    all_x, all_y = [], []

    if tags:
        info = tags['DRONE']
        fresh = (now - info['ts']) < STALE_SEC
        mc = UWB_COLOR if fresh else '#aaaaaa'

        for aid, av in info.get('anchors', {}).items():
            ax_p, ay_p, _ = av['pos']
            ax_map.plot(ax_p, ay_p, 's',
                        color='#555555', markersize=9, zorder=5)
            ax_map.text(ax_p + 0.1, ay_p + 0.1, aid,
                        fontsize=7, color='#555555')
            all_x.append(ax_p)
            all_y.append(ay_p)

        trail = info['trail']
        if len(trail) > 1:
            tx, ty = zip(*trail)
            ax_map.plot(tx, ty, '-', color=UWB_COLOR,
                        alpha=0.35, linewidth=1)

        ax_map.plot(info['x'], info['y'], 'o',
                    color=mc, markersize=12,
                    markeredgecolor='white', markeredgewidth=1.5, zorder=10)
        ax_map.text(info['x'] + 0.12, info['y'] + 0.12,
                    f"UWB\n({info['x']:.2f},{info['y']:.2f})",
                    fontsize=7, color=mc)
        all_x.append(info['x'])
        all_y.append(info['y'])

    if fc and (now - fc[3]) < STALE_SEC:
        # apply UWB origin offset so FC and UWB share the same frame
        if offset:
            fx = fc[0] + offset[0]
            fy = fc[1] + offset[1]
        else:
            fx, fy = fc[0], fc[1]
        ax_map.plot(fx, fy, 's',
                    color=FC_COLOR, markersize=13,
                    markeredgecolor='white', markeredgewidth=1.5, zorder=11)
        ax_map.text(fx + 0.12, fy - 0.25,
                    f"FC\n({fx:.2f},{fy:.2f})",
                    fontsize=7, color=FC_COLOR)
        all_x.append(fx)
        all_y.append(fy)

        if tags:
            info = tags['DRONE']
            ax_map.plot([info['x'], fx], [info['y'], fy],
                        '--', color='#888888',
                        linewidth=1, alpha=0.6, zorder=5)

    if all_x and all_y:
        margin = 3.0
        try:
            ax_map.set_xlim(min(all_x) - margin, max(all_x) + margin)
            ax_map.set_ylim(min(all_y) - margin, max(all_y) + margin)
        except ValueError:
            pass
    else:
        ax_map.text(0.5, 0.5, 'Waiting for data...',
                    ha='center', va='center', fontsize=11, color='gray',
                    transform=ax_map.transAxes)

    # ── drift panel ──────────────────────────────────────────────────────────
    ax_drift.cla()
    ax_drift.set_title("UWB vs FC DRIFT")
    ax_drift.axis('off')

    if fc and tags:
        info = tags['DRONE']
        fc_age = now - fc[3]
        uwb_age = now - info['ts']
        # use offset-adjusted FC coords for drift calculation
        if offset:
            fc_ax = fc[0] + offset[0]
            fc_ay = fc[1] + offset[1]
        else:
            fc_ax, fc_ay = fc[0], fc[1]
        dx = info['x'] - fc_ax
        dy = info['y'] - fc_ay
        dist = math.sqrt(dx**2 + dy**2)

        if offset is None:
            align = "calibrating..."
            color = '#aaaaaa'
        elif dist < 0.3:
            align = "GOOD"
            color = '#27ae60'
        elif dist < 0.8:
            align = "OK"
            color = '#e67e22'
        else:
            align = "MISALIGNED"
            color = '#e74c3c'

        n = len(_uwb_init_samples)
        lines = [
            f"UWB  x = {info['x']:+.3f} m",
            f"UWB  y = {info['y']:+.3f} m",
            f"UWB age = {uwb_age:.1f} s",
            "",
            f"FC   x = {fc_ax:+.3f} m  (raw={fc[0]:+.2f})",
            f"FC   y = {fc_ay:+.3f} m  (raw={fc[1]:+.2f})",
            f"FC   z = {fc[2]:+.3f} m",
            f"FC  age = {fc_age:.1f} s",
            f"offset = ({offset[0]:+.2f},{offset[1]:+.2f})"
            if offset else f"offset = calibrating ({n}/5)",
            "",
            f"dx = {dx:+.3f} m",
            f"dy = {dy:+.3f} m",
            f"dist = {dist:.3f} m",
            "",
            f"Alignment: {align}",
        ]
        ax_drift.text(0.05, 0.95, '\n'.join(lines),
                      transform=ax_drift.transAxes,
                      va='top', fontsize=9, family='monospace', color=color)
    elif not fc:
        ax_drift.text(0.5, 0.6,
                      'FC not connected\n\nCheck FC_CONN\nor FC power',
                      ha='center', va='center', fontsize=10, color='gray',
                      transform=ax_drift.transAxes)
    else:
        ax_drift.text(0.5, 0.6, 'Waiting for UWB data...',
                      ha='center', va='center', fontsize=10, color='gray',
                      transform=ax_drift.transAxes)


def main():
    fc_conn = None
    if _MAVLINK_OK:
        fc_conn = _fc_connect()
        if fc_conn is None:
            print("[FC] not found — running UWB-only mode")

    # pass conn to UWBTag so VISION is injected → EKF gets position
    uwb = UWBTag(conn=fc_conn)
    uwb.start()

    if fc_conn is not None:
        t_fc = threading.Thread(
            target=_fc_read_thread, args=(fc_conn,), daemon=True)
        t_fc.start()

    fig, ax_info, ax_map, ax_drift = _build_figure()
    fig.ani = animation.FuncAnimation(
        fig, _update, fargs=(ax_info, ax_map, ax_drift, uwb),
        interval=200, cache_frame_data=False,
    )
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()
