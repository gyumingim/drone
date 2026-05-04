"""
sitl_viz.py — SITL 실시간 XY 궤적 시각화 + 센서/목표 제어

LOCAL_POSITION_NED 수신 → 2D 궤적 실시간 플롯
슬라이더: FakeUWB North/East, 고도 Z
텍스트 입력: go_to 목표 좌표 (Enter로 적용)
UDP port 14560으로 sitl_flight.py에 전송.

포트: FC_VIZ_PORT 환경변수로 오버라이드 (기본 udpin:0.0.0.0:14552)
"""
import math
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

_WARN_M  = 0.5   # 이 이상이면 주황 경고
_CRASH_M = 1.0   # 이 이상이면 빨간 위험

_xs = collections.deque(maxlen=TRAIL)
_ys = collections.deque(maxlen=TRAIL)
_lock = threading.Lock()

_udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

_target_n = 0.0
_target_e = 0.0


def _send_ctrl(sensor_n, sensor_e, alt, target_n, target_e, noise):
    msg = f'{sensor_n:.3f},{sensor_e:.3f},{alt:.3f},{target_n:.3f},{target_e:.3f},{noise:.3f}'.encode()
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
    # 오차 벡터: 현재 위치 → 목표 (FC가 보정하려는 방향)
    error_line,  = ax.plot([], [], 'r--', alpha=0.7, linewidth=1.5, label='FC error vector')
    ax.set_xlabel('East (m)')
    ax.set_ylabel('North (m)')
    ax.set_title('Position Trajectory — SITL')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True)
    ax.set_aspect('equal')
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)

    # 오차 텍스트 (plot 내부 좌상단)
    err_text = ax.text(0.02, 0.97, '', transform=ax.transAxes,
                       va='top', ha='left', fontsize=10, fontweight='bold',
                       bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.8))

    # ── sensor input sliders ──────────────────────────────────────────────────
    fig.text(0.5, 0.455, '── Sensor Input (UWB / Alt / Noise) ──',
             ha='center', va='bottom', fontsize=9, color='#555')

    ax_sn    = fig.add_axes([0.18, 0.39, 0.64, 0.035])
    ax_se    = fig.add_axes([0.18, 0.33, 0.64, 0.035])
    ax_alt   = fig.add_axes([0.18, 0.27, 0.64, 0.035])
    ax_noise = fig.add_axes([0.18, 0.21, 0.64, 0.035])

    s_north = Slider(ax_sn,    'UWB North (m)', -5.0, 5.0, valinit=0.0, color='lightgreen')
    s_east  = Slider(ax_se,    'UWB East  (m)', -5.0, 5.0, valinit=0.0, color='skyblue')
    s_alt   = Slider(ax_alt,   'Alt       (m)',  0.2, 3.0, valinit=1.0, color='#ffffcc')
    s_noise = Slider(ax_noise, 'UWB Noise (m)',  0.0, 0.5, valinit=0.0, color='#ffcccc')

    def on_sensor_change(_):
        _send_ctrl(s_north.val, s_east.val, s_alt.val, _target_n, _target_e, s_noise.val)

    s_north.on_changed(on_sensor_change)
    s_east.on_changed(on_sensor_change)
    s_alt.on_changed(on_sensor_change)
    s_noise.on_changed(on_sensor_change)

    # ── flight target textboxes ───────────────────────────────────────────────
    fig.text(0.5, 0.165, '── Flight Target (Enter to apply) ──',
             ha='center', va='bottom', fontsize=9, color='#555')

    ax_tn = fig.add_axes([0.45, 0.10, 0.35, 0.05])
    ax_te = fig.add_axes([0.45, 0.03, 0.35, 0.05])

    tb_north = TextBox(ax_tn, 'Target N (m)', initial='0.0')
    tb_east  = TextBox(ax_te, 'Target E (m)', initial='0.0')

    def on_target_submit(_):
        global _target_n, _target_e
        try:
            _target_n = float(tb_north.text)
            _target_e = float(tb_east.text)
        except ValueError:
            return
        _send_ctrl(s_north.val, s_east.val, s_alt.val, _target_n, _target_e, s_noise.val)

    tb_north.on_submit(on_target_submit)
    tb_east.on_submit(on_target_submit)

    # ── animation ─────────────────────────────────────────────────────────────
    def update(_):
        with _lock:
            x = list(_xs)
            y = list(_ys)

        target_dot.set_data([_target_e], [_target_n])

        if not x:
            error_line.set_data([], [])
            err_text.set_text('no data')
            err_text.set_color('#888')
            return trail_line, cur_dot, target_dot, error_line, err_text

        cur_n, cur_e = x[-1], y[-1]
        trail_line.set_data(y, x)
        cur_dot.set_data([cur_e], [cur_n])

        # 오차 벡터: 현재 → 목표
        error_line.set_data([cur_e, _target_e], [cur_n, _target_n])

        # 오차 거리 + 위험도 색상
        err = math.hypot(cur_n - _target_n, cur_e - _target_e)
        if err >= _CRASH_M:
            color, label = '#cc0000', f'err {err:.2f}m  DANGER — may crash!'
        elif err >= _WARN_M:
            color, label = '#e07000', f'err {err:.2f}m  WARNING'
        else:
            color, label = '#008800', f'err {err:.2f}m  OK'
        err_text.set_text(label)
        err_text.set_color(color)
        err_text.get_bbox_patch().set_edgecolor(color)

        all_vals = x + y + [_target_n, _target_e]
        margin = max(max(abs(v) for v in all_vals), 0.5) * 1.4
        ax.set_xlim(-margin, margin)
        ax.set_ylim(-margin, margin)
        return trail_line, cur_dot, target_dot, error_line, err_text

    ani = animation.FuncAnimation(fig, update, interval=100, blit=False)  # noqa: F841
    plt.show()


if __name__ == '__main__':
    main()
