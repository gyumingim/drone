"""
sitl_viz.py — SITL 실시간 XY 궤적 시각화 + 센서/목표 제어

LOCAL_POSITION_NED 수신 → 2D 궤적 실시간 플롯
슬라이더: FakeUWB North/East, 고도 Z
텍스트 입력: go_to 목표 좌표 (Enter로 적용)
UDP port 14560으로 sitl_flight.py에 전송.

포트: FC_VIZ_PORT 환경변수로 오버라이드 (기본 udpin:0.0.0.0:14552)
"""
import os
import socket
import threading
import collections

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Slider, TextBox
from pymavlink import mavutil

FC_VIZ_PORT = os.environ.get('FC_VIZ_PORT', 'udpin:0.0.0.0:14552')
UDP_CTRL_PORT = 14560
TRAIL = 300

_xs = collections.deque(maxlen=TRAIL)
_ys = collections.deque(maxlen=TRAIL)
_lock = threading.Lock()

_udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

_target_n = 0.0
_target_e = 0.0


def _send_ctrl(sensor_n, sensor_e, alt, target_n, target_e):
    msg = f'{sensor_n:.3f},{sensor_e:.3f},{alt:.3f},{target_n:.3f},{target_e:.3f}'.encode()
    _udp_sock.sendto(msg, ('127.0.0.1', UDP_CTRL_PORT))


def _recv_loop():
    c = mavutil.mavlink_connection(FC_VIZ_PORT)
    c.wait_heartbeat()
    while True:
        m = c.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
        if m:
            with _lock:
                _xs.append(m.x)
                _ys.append(m.y)


def main():
    global _target_n, _target_e

    threading.Thread(target=_recv_loop, daemon=True).start()

    fig = plt.figure(figsize=(8, 10))

    # ── trajectory plot ───────────────────────────────────────────────────────
    ax = fig.add_axes([0.10, 0.47, 0.82, 0.47])
    trail_line,  = ax.plot([], [], 'b-', alpha=0.4, linewidth=1.0, label='trail')
    cur_dot,     = ax.plot([], [], 'ro', markersize=8,  label='current pos', zorder=6)
    target_dot,  = ax.plot([0], [0], 'g*', markersize=14, label='go_to target', zorder=5)
    ax.set_xlabel('East (m)')
    ax.set_ylabel('North (m)')
    ax.set_title('Position Trajectory — SITL')
    ax.legend(loc='upper right')
    ax.grid(True)
    ax.set_aspect('equal')
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)

    # ── sensor input sliders ──────────────────────────────────────────────────
    fig.text(0.5, 0.455, '── Sensor Input (UWB / Alt) ──',
             ha='center', va='bottom', fontsize=9, color='#555')

    ax_sn  = fig.add_axes([0.18, 0.38, 0.64, 0.04])
    ax_se  = fig.add_axes([0.18, 0.31, 0.64, 0.04])
    ax_alt = fig.add_axes([0.18, 0.24, 0.64, 0.04])

    s_north = Slider(ax_sn,  'UWB North (m)', -5.0, 5.0, valinit=0.0, color='lightgreen')
    s_east  = Slider(ax_se,  'UWB East  (m)', -5.0, 5.0, valinit=0.0, color='skyblue')
    s_alt   = Slider(ax_alt, 'Alt       (m)',  0.2, 3.0, valinit=1.0, color='#ffffcc')

    def on_sensor_change(_):
        _send_ctrl(s_north.val, s_east.val, s_alt.val, _target_n, _target_e)

    s_north.on_changed(on_sensor_change)
    s_east.on_changed(on_sensor_change)
    s_alt.on_changed(on_sensor_change)

    # ── flight target textboxes ───────────────────────────────────────────────
    fig.text(0.5, 0.195, '── Flight Target (Enter to apply) ──',
             ha='center', va='bottom', fontsize=9, color='#555')

    ax_tn = fig.add_axes([0.45, 0.13, 0.35, 0.05])
    ax_te = fig.add_axes([0.45, 0.06, 0.35, 0.05])

    tb_north = TextBox(ax_tn, 'Target N (m)', initial='0.0')
    tb_east  = TextBox(ax_te, 'Target E (m)', initial='0.0')

    def on_target_submit(_):
        global _target_n, _target_e
        try:
            _target_n = float(tb_north.text)
            _target_e = float(tb_east.text)
        except ValueError:
            return
        _send_ctrl(s_north.val, s_east.val, s_alt.val, _target_n, _target_e)

    tb_north.on_submit(on_target_submit)
    tb_east.on_submit(on_target_submit)

    # ── animation ─────────────────────────────────────────────────────────────
    def update(_):
        with _lock:
            x = list(_xs)
            y = list(_ys)
        # update target marker (East=x-axis, North=y-axis)
        target_dot.set_data([_target_e], [_target_n])
        if not x:
            return trail_line, cur_dot, target_dot
        trail_line.set_data(y, x)
        cur_dot.set_data([y[-1]], [x[-1]])
        all_vals = x + y + [_target_n, _target_e]
        margin = max(max(abs(v) for v in all_vals), 0.5) * 1.4
        ax.set_xlim(-margin, margin)
        ax.set_ylim(-margin, margin)
        return trail_line, cur_dot, target_dot

    ani = animation.FuncAnimation(fig, update, interval=100, blit=False)  # noqa: F841
    plt.show()


if __name__ == '__main__':
    main()
