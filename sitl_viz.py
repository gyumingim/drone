"""
sitl_viz.py — SITL 실시간 XY 궤적 시각화 + 센서 입력 제어

LOCAL_POSITION_NED 수신 → 2D 궤적 실시간 플롯
슬라이더로 FakeUWB North/East, 고도 Z를 실시간 제어.
슬라이더 값은 UDP port 14560으로 sitl_flight.py에 전송.

실행: python3 sitl_viz.py  (sitl_flight.py와 별도 터미널)
포트: FC_VIZ_PORT 환경변수로 오버라이드 (기본 udpin:0.0.0.0:14552)
"""
import os
import socket
import threading
import collections

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Slider
from pymavlink import mavutil

FC_VIZ_PORT = os.environ.get('FC_VIZ_PORT', 'udpin:0.0.0.0:14552')
UDP_CTRL_PORT = 14560
TRAIL = 300  # 최근 N개 점만 표시

_xs = collections.deque(maxlen=TRAIL)
_ys = collections.deque(maxlen=TRAIL)
_lock = threading.Lock()

_udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def _send_ctrl(north, east, alt):
    msg = f'{north:.3f},{east:.3f},{alt:.3f}'.encode()
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
    threading.Thread(target=_recv_loop, daemon=True).start()

    fig = plt.figure(figsize=(7, 9))

    # trajectory plot (upper area)
    ax = fig.add_axes([0.10, 0.34, 0.82, 0.59])
    trail_line, = ax.plot([], [], 'b-', alpha=0.4, linewidth=1.0, label='trail')
    cur_dot,    = ax.plot([], [], 'ro', markersize=8, label='current pos')
    ax.scatter([0], [0], marker='+', s=300, c='green', zorder=5, label='target (0,0)')
    ax.set_xlabel('East (m)')
    ax.set_ylabel('North (m)')
    ax.set_title('Position Trajectory — SITL')
    ax.legend()
    ax.grid(True)
    ax.set_aspect('equal')

    fig.text(0.5, 0.315, 'Sensor Input Control', ha='center', va='bottom',
             fontsize=10, color='#444')

    # sliders (lower area)
    ax_north = fig.add_axes([0.18, 0.22, 0.64, 0.04])
    ax_east  = fig.add_axes([0.18, 0.15, 0.64, 0.04])
    ax_alt   = fig.add_axes([0.18, 0.08, 0.64, 0.04])

    s_north = Slider(ax_north, 'UWB North (m)', -5.0, 5.0, valinit=0.0, color='lightgreen')
    s_east  = Slider(ax_east,  'UWB East  (m)', -5.0, 5.0, valinit=0.0, color='skyblue')
    s_alt   = Slider(ax_alt,   'Alt       (m)',  0.2, 3.0, valinit=1.0, color='#ffffcc')

    def on_change(_):
        _send_ctrl(s_north.val, s_east.val, s_alt.val)

    s_north.on_changed(on_change)
    s_east.on_changed(on_change)
    s_alt.on_changed(on_change)

    def update(_):
        with _lock:
            x = list(_xs)
            y = list(_ys)
        if not x:
            return trail_line, cur_dot
        # x=North, y=East → plot: horizontal=East, vertical=North
        trail_line.set_data(y, x)
        cur_dot.set_data([y[-1]], [x[-1]])
        margin = max(max(abs(v) for v in x + y), 0.5) * 1.4
        ax.set_xlim(-margin, margin)
        ax.set_ylim(-margin, margin)
        return trail_line, cur_dot

    ani = animation.FuncAnimation(fig, update, interval=100, blit=False)  # noqa: F841
    plt.show()


if __name__ == '__main__':
    main()
