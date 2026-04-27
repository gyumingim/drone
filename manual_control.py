"""
manual_control.py — takeoff, hover 2s, land.
All shared primitives and CLI dashboard live in viz.py.
"""

import time
import viz


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

        viz.flog(f"Hovering 2s at z={hover_z:.2f}m...")
        viz._tset(phase='hover')
        held = viz.hold(conn, 0, 0, hover_z, 2.0, uwb=uwb)
        if held:
            viz.land(conn)
        viz.flog("Done")
        viz._tset(phase='done', armed=False)
    except Exception as e:
        viz.flog(f"[FLIGHT] Exception: {e}")
        viz._emergency_disarm(conn)


if __name__ == '__main__':
    viz.run(_flight, title='MANUAL CONTROL  --  hover test')