"""
sitl_viz.py — SITL 실시간 XY 궤적 시각화

LOCAL_POSITION_NED 수신 → 2D 궤적 실시간 플롯
원점(0,0) 수렴 여부와 진동 진폭을 시각적으로 확인.

실행: python3 sitl_viz.py  (sitl_flight.py와 별도 터미널)
포트: FC_VIZ_PORT 환경변수로 오버라이드 (기본 udpin:0.0.0.0:14552)
"""
import os
import threading
import collections

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from pymavlink import mavutil

FC_VIZ_PORT = os.environ.get('FC_VIZ_PORT', 'udpin:0.0.0.0:14552')
TRAIL = 300  # 최근 N개 점만 표시

_xs = collections.deque(maxlen=TRAIL)
_ys = collections.deque(maxlen=TRAIL)
_lock = threading.Lock()


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

    fig, ax = plt.subplots(figsize=(6, 6))
    trail_line, = ax.plot([], [], 'b-', alpha=0.4, linewidth=1.0, label='궤적')
    cur_dot,    = ax.plot([], [], 'ro', markersize=8, label='현재 위치')
    ax.scatter([0], [0], marker='+', s=300, c='green', zorder=5, label='목표 (0,0)')
    ax.set_xlabel('East (m)')
    ax.set_ylabel('North (m)')
    ax.set_title('Position Trajectory — SITL')
    ax.legend()
    ax.grid(True)
    ax.set_aspect('equal')

    def update(_):
        with _lock:
            x = list(_xs)
            y = list(_ys)
        if not x:
            return trail_line, cur_dot
        # 축: x축=East(y), y축=North(x)
        trail_line.set_data(y, x)
        cur_dot.set_data([y[-1]], [x[-1]])
        margin = max(max(abs(v) for v in x + y), 0.5) * 1.4
        ax.set_xlim(-margin, margin)
        ax.set_ylim(-margin, margin)
        return trail_line, cur_dot

    ani = animation.FuncAnimation(fig, update, interval=100, blit=True)  # noqa: F841
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()
