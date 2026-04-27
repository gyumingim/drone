"""
flight_cardinal.py — N -> E -> S -> W 30cm square, 2s hold per waypoint.
All shared primitives and CLI dashboard live in viz.py.
"""

import time
import viz

STEP_M = 0.30
STEP_HOLD = 2.0


def _flight(conn, uwb):
    try:
        viz.flog("Waiting for UWB origin...")
        deadline = time.time() + 20.0
        while time.time() < deadline:
            if uwb.get_drone_pos() is not None:
                viz.flog("UWB origin locked — ready to fly")
                break
            time.sleep(0.2)
        else:
            viz.flog("UWB origin timeout — proceeding anyway")

        viz.debug_status(conn, uwb, duration=15)
        viz.wait_ready(conn)
        viz.flog("Setting GUIDED mode...")
        viz.set_guided(conn)
        time.sleep(0.3)

        if not viz.arm(conn):
            viz.flog("[FLIGHT] ARM failed — aborting")
            return
        time.sleep(0.5)

        hover_z = viz.takeoff(conn, viz.TAKEOFF_ALT)
        if hover_z is None:
            viz.flog("[FLIGHT] Takeoff failed — aborting")
            return

        pos0 = conn.recv_match(
            type='LOCAL_POSITION_NED', blocking=True, timeout=2)
        sx = pos0.x if pos0 else 0.0
        sy = pos0.y if pos0 else 0.0
        viz.flog(f"Origin: ({sx:.2f}, {sy:.2f}, {hover_z:.2f})")

        waypoints = [
            ("North +30cm", sx + STEP_M, sy),
            ("East  +30cm", sx + STEP_M, sy + STEP_M),
            ("South -30cm", sx,          sy + STEP_M),
            ("West  -30cm", sx,          sy),
        ]

        viz._tset(phase='mission')
        for label, tx, ty in waypoints:
            viz.flog(f"-> {label}  target=({tx:.2f}, {ty:.2f})")
            if not viz.goto(conn, tx, ty, hover_z):
                viz.flog("[MISSION] goto timeout — landing")
                viz.land(conn)
                return
            viz.flog(f"   {label} reached — hold {STEP_HOLD}s")
            if not viz.hold(conn, tx, ty, hover_z, STEP_HOLD, uwb=uwb):
                viz.flog("[MISSION] UWB signal lost — abort")
                return

        viz.flog("Mission complete — landing")
        viz.land(conn)
        viz.flog("Done")
        viz._tset(phase='done', armed=False)
    except Exception as e:
        viz.flog(f"[FLIGHT] Exception: {e}")
        viz._emergency_disarm(conn)


if __name__ == '__main__':
    viz.run(_flight, title='FLIGHT CARDINAL  N->E->S->W  30cm')
