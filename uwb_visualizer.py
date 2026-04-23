"""
GrowSpace UWB Listener - real-time tag position visualizer
Listener lep output: POS,<idx>,<tag_id>,<x>,<y>,<z>,<qf>
"""

import threading
import time
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

UWB_PORT   = '/dev/ttyUSB0'
UWB_BAUD   = 115200
STALE_SEC  = 1.0
TRAIL_LEN  = 50   # position history length per tag

COLORS = ['#e74c3c', '#3498db', '#2ecc71', '#f39c12',
          '#9b59b6', '#1abc9c', '#e67e22', '#34495e']

# ── shared state ──────────────────────────────────────────────────────────────
# {tag_id: {'x', 'y', 'z', 'qf', 'ts', 'trail': deque}}
_tags = {}
_lock = threading.Lock()


# ── parser ────────────────────────────────────────────────────────────────────

def _parse_pos(line: str):
    """POS,<idx>,<tag_id>,<x>,<y>,<z>,<qf> -> (tag_id, x, y, z, qf) or None"""
    try:
        parts = line.strip().split(',')
        if parts[0] != 'POS' or len(parts) < 7:
            return None
        tag_id = parts[2]
        x  = float(parts[3])
        y  = float(parts[4])
        z  = float(parts[5])
        qf = int(parts[6])
        import math
        if not all(math.isfinite(v) for v in (x, y, z)):
            return None
        return tag_id, x, y, z, qf
    except (ValueError, IndexError):
        return None


# ── serial thread ─────────────────────────────────────────────────────────────

def _serial_thread():
    _rx_count = 0
    _parse_ok  = 0
    _last_stat = time.time()

    while True:
        try:
            with serial.Serial(UWB_PORT, UWB_BAUD, timeout=1.0) as ser:
                print(f"[UWB] port open: {UWB_PORT} @ {UWB_BAUD}")
                ser.write(b'lep\r\n')
                print("[UWB] sent 'lep' command")
                time.sleep(0.1)
                while True:
                    line = ser.readline().decode('ascii', errors='ignore')
                    if line:
                        _rx_count += 1
                        # print every raw line for first 20 lines
                        if _rx_count <= 20:
                            print(f"[RAW #{_rx_count:02d}] {line.rstrip()!r}")

                    result = _parse_pos(line)
                    if result:
                        _parse_ok += 1
                        tag_id, x, y, z, qf = result
                        now = time.time()
                        with _lock:
                            if tag_id not in _tags:
                                print(f"[UWB] new tag: {tag_id}")
                                _tags[tag_id] = {
                                    'x': x, 'y': y, 'z': z,
                                    'qf': qf, 'ts': now,
                                    'trail': deque(maxlen=TRAIL_LEN),
                                }
                            _tags[tag_id].update({'x': x, 'y': y, 'z': z,
                                                  'qf': qf, 'ts': now})
                            _tags[tag_id]['trail'].append((x, y))

                    # stats every 5s
                    now2 = time.time()
                    if now2 - _last_stat >= 5.0:
                        print(f"[UWB] rx={_rx_count} parsed={_parse_ok} tags={len(_tags)}")
                        _last_stat = now2

        except serial.SerialException as e:
            print(f"[UWB] error: {e} — retry in 3s")
            time.sleep(3.0)


# ── plot ──────────────────────────────────────────────────────────────────────

def _build_figure():
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    fig.suptitle("GrowSpace UWB Listener — Live Monitor", fontsize=13, fontweight='bold')
    axes[1].set_aspect('equal')
    axes[1].grid(True, linestyle='--', alpha=0.4)
    return fig, axes[0], axes[1]


def _update(frame, ax_info, ax_map):
    now = time.time()

    with _lock:
        tags = {k: {**v, 'trail': list(v['trail'])} for k, v in _tags.items()}

    # ── info table ────────────────────────────────────────────────────────────
    ax_info.cla()
    ax_info.set_title("Tag status")
    ax_info.axis('off')

    headers = ['Tag ID', 'X (m)', 'Y (m)', 'Height (m)', 'QF', 'Age (s)']
    rows = []
    for tag_id, info in sorted(tags.items()):
        age = now - info['ts']
        rows.append([
            tag_id,
            f"{info['x']:.2f}",
            f"{info['y']:.2f}",
            f"{-info['z']:.2f}",   # UWB Z is down-positive → negate to show height
            str(info['qf']),
            f"{age:.1f}",
        ])

    if rows:
        table = ax_info.table(
            cellText=rows,
            colLabels=headers,
            loc='center',
            cellLoc='center',
        )
        table.auto_set_font_size(False)
        table.set_fontsize(10)
        table.scale(1.2, 2.0)

        for i, (tag_id, _) in enumerate(sorted(tags.items())):
            color = COLORS[i % len(COLORS)]
            for j in range(len(headers)):
                table[i + 1, j].set_facecolor(color + '33')  # transparent tint
    else:
        ax_info.text(0.5, 0.5, 'Waiting for data...',
                     ha='center', va='center', fontsize=12, color='gray',
                     transform=ax_info.transAxes)

    # ── 2D map ────────────────────────────────────────────────────────────────
    ax_map.cla()
    ax_map.set_title("Tag position (XY)")
    ax_map.set_xlabel("X (m)")
    ax_map.set_ylabel("Y (m)")
    ax_map.set_aspect('equal')
    ax_map.grid(True, linestyle='--', alpha=0.4)

    all_x, all_y = [], []

    for i, (tag_id, info) in enumerate(sorted(tags.items())):
        color = COLORS[i % len(COLORS)]
        fresh = (now - info['ts']) < STALE_SEC

        trail = info['trail']
        if len(trail) > 1:
            tx, ty = zip(*trail)
            ax_map.plot(tx, ty, '-', color=color, alpha=0.35, linewidth=1)

        marker_color = color if fresh else '#aaaaaa'
        ax_map.plot(info['x'], info['y'], 'o',
                    color=marker_color, markersize=12,
                    markeredgecolor='white', markeredgewidth=1.5,
                    zorder=10)
        ax_map.text(info['x'] + 0.1, info['y'] + 0.1,
                    f"{tag_id}\nQF={info['qf']}",
                    fontsize=8, color=marker_color)

        all_x.append(info['x'])
        all_y.append(info['y'])

    if all_x and all_y:
        margin = 2.0
        try:
            ax_map.set_xlim(min(all_x) - margin, max(all_x) + margin)
            ax_map.set_ylim(min(all_y) - margin, max(all_y) + margin)
        except ValueError:
            pass
    else:
        ax_map.text(0.5, 0.5, 'Waiting for data...',
                    ha='center', va='center', fontsize=12, color='gray',
                    transform=ax_map.transAxes)


def main():
    t = threading.Thread(target=_serial_thread, daemon=True)
    t.start()

    fig, ax_info, ax_map = _build_figure()
    fig.ani = animation.FuncAnimation(
        fig, _update, fargs=(ax_info, ax_map),
        interval=200, cache_frame_data=False,
    )
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()
