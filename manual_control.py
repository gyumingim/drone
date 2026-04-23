"""
ArduPilot manual control + GrowSpace UWB Listener integration
- Reads tag position from Listener (lep)
- Injects VISION_POSITION_ESTIMATE into ArduPilot EKF
- Live matplotlib visualization runs in main thread
- Flight control runs in background thread
"""

import time
import threading
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from pymavlink import mavutil
from uwb_tag import UWBTag

# ── config ───────────────────────────────────────────────────────────────────

FC_PORT = '/dev/ttyACM0'
FC_BAUD = 57600

TAKEOFF_ALT = 1.0
CLIMB_RATE  = 0.2   # m/s
TOLERANCE = 0.3

COLORS = ['#e74c3c', '#3498db', '#2ecc71', '#f39c12',
          '#9b59b6', '#1abc9c', '#e67e22', '#34495e']

_POS_ONLY = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)


# ── flight control ───────────────────────────────────────────────────────────

def connect(port=FC_PORT, baud=FC_BAUD):
    candidates = [port, '/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyACM2']
    for p in dict.fromkeys(candidates):  # deduplicate, preserve order
        try:
            print(f"Trying {p}...")
            conn = mavutil.mavlink_connection(p, baud=baud)
            conn.wait_heartbeat(timeout=3)
            print(f"Connected on {p} (sysid={conn.target_system})")
            return conn
        except Exception:
            pass
    raise RuntimeError("FC not found on any ACM port")


def send_global_origin(conn, lat=0.0, lon=0.0, alt=0.0):
    """EKF local frame origin — dummy value OK for indoor ExternalNav."""
    conn.mav.set_gps_global_origin_send(
        conn.target_system,
        int(lat * 1e7),
        int(lon * 1e7),
        int(alt * 1000),
    )
    print("Global origin sent")


def request_streams(conn):
    """Ask ArduPilot to send EKF, position, attitude, servo streams."""
    streams = [
        mavutil.mavlink.MAV_DATA_STREAM_EXTRA3,    # EKF_STATUS_REPORT
        mavutil.mavlink.MAV_DATA_STREAM_POSITION,   # LOCAL_POSITION_NED
        mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,     # ATTITUDE
        mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, # SERVO_OUTPUT_RAW
    ]
    for s in streams:
        conn.mav.request_data_stream_send(
            conn.target_system, conn.target_component,
            s, 10, 1
        )
    print("Streams requested")


_EKF_ATT      = 0x001   # attitude
_EKF_VEL_H    = 0x002   # horiz velocity
_EKF_POS_REL  = 0x008   # horiz pos (relative)
_EKF_POS_ABS  = 0x010   # horiz pos (absolute)
_EKF_CONST    = 0x080   # constant-position mode (no horiz pos)
_EKF_NEED     = _EKF_ATT | _EKF_VEL_H | _EKF_POS_REL | _EKF_POS_ABS


def wait_ready(conn, timeout=15):
    print("Waiting for EKF...")
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = conn.recv_match(
            type='EKF_STATUS_REPORT', blocking=True, timeout=2
        )
        if msg:
            bits = msg.flags
            print(
                f"  EKF={bits:#06x} "
                f"att={bool(bits&_EKF_ATT)} "
                f"vel={bool(bits&_EKF_VEL_H)} "
                f"pos_rel={bool(bits&_EKF_POS_REL)} "
                f"pos_abs={bool(bits&_EKF_POS_ABS)} "
                f"const_pos={bool(bits&_EKF_CONST)}"
            )
            if (bits & _EKF_NEED) == _EKF_NEED:
                print("  EKF Ready")
                return
    print("  EKF Timeout — proceeding anyway")


def set_mode(conn, mode_id):
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id, 0, 0, 0, 0, 0
    )


def set_guided(conn):
    set_mode(conn, 4)   # GUIDED


def set_althold(conn):
    set_mode(conn, 2)   # ALT_HOLD


def arm(conn, timeout=8):
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, 0, 0, 0, 0, 0, 0
    )
    print("Waiting for ARM...")
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = conn.recv_match(
            type=['HEARTBEAT', 'STATUSTEXT'],
            blocking=True, timeout=1.0
        )
        if msg is None:
            continue
        if msg.get_type() == 'STATUSTEXT':
            print(f"  [FC] {msg.text.strip()}")
        if msg.get_type() == 'HEARTBEAT':
            armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            if armed:
                print("Armed")
                return True
    print("ARM failed — pre-arm check messages above")
    return False


def send_position(conn, x, y, z):
    conn.mav.set_position_target_local_ned_send(
        0,
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        _POS_ONLY,
        x, y, z,
        0, 0, 0,
        0, 0, 0,
        0, 0
    )


def goto(conn, x, y, z, tolerance=TOLERANCE, timeout=30):
    print(f"goto ({x:.1f}, {y:.1f}, {-z:.1f}m)")
    deadline = time.time() + timeout
    last_servo = 0
    while time.time() < deadline:
        send_position(conn, x, y, z)
        msg = conn.recv_match(
            type=['LOCAL_POSITION_NED', 'SERVO_OUTPUT_RAW'],
            blocking=False
        )
        if msg:
            if msg.get_type() == 'LOCAL_POSITION_NED':
                dist = (
                    (msg.x - x) ** 2 +
                    (msg.y - y) ** 2 +
                    (msg.z - z) ** 2
                ) ** 0.5
                print(f"  pos=({msg.x:.2f},{msg.y:.2f},{msg.z:.2f})"
                      f"  dist={dist:.2f}m", end="\r")
                if dist < tolerance:
                    print("\n  Reached")
                    return True
            elif msg.get_type() == 'SERVO_OUTPUT_RAW':
                now2 = time.time()
                if now2 - last_servo > 3.0:
                    print(
                        f"\n  SERVO: {msg.servo1_raw} {msg.servo2_raw}"
                        f" {msg.servo3_raw} {msg.servo4_raw}"
                    )
                    last_servo = now2
        time.sleep(0.05)
    print("\n  Timeout")
    return False


def hold(conn, x, y, z, duration, uwb=None):
    """Hold position. If uwb given, abort and land if signal lost >2s."""
    deadline = time.time() + duration
    while time.time() < deadline:
        send_position(conn, x, y, z)
        if uwb is not None:
            tags = uwb.get_tags()
            stale = tags and all(
                time.time() - v['ts'] > 2.0 for v in tags.values()
            )
            if stale or not tags:
                print("\n[SAFETY] UWB signal lost during hold — landing")
                land(conn)
                return False
        time.sleep(0.1)
    return True


def takeoff(conn, alt=TAKEOFF_ALT):
    """Takeoff using MAV_CMD_NAV_TAKEOFF then wait for altitude."""
    # MAV_CMD_NAV_TAKEOFF: param7 = altitude above home (m)
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
        0, 0, 0, float('nan'),   # min_pitch, empty, empty, yaw
        0, 0, alt,               # lat, lon, altitude (m above home)
    )
    print(f"[TAKEOFF] MAV_CMD_NAV_TAKEOFF sent, target={alt}m")
    ack = conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    if ack:
        print(f"[TAKEOFF] ACK result={ack.result}"
              f" ({'OK' if ack.result==0 else 'FAIL'})")
    else:
        print("[TAKEOFF] No ACK received")

    deadline = time.time() + alt / CLIMB_RATE * 5
    last_servo = 0.0
    while time.time() < deadline:
        msg = conn.recv_match(
            type=['LOCAL_POSITION_NED', 'SERVO_OUTPUT_RAW', 'HEARTBEAT'],
            blocking=True, timeout=0.5,
        )
        if msg is None:
            continue
        t = msg.get_type()
        if t == 'HEARTBEAT':
            armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            if not armed:
                print("\n[TAKEOFF] Drone disarmed during takeoff!")
                return None
        elif t == 'SERVO_OUTPUT_RAW':
            now2 = time.time()
            if now2 - last_servo > 2.0:
                print(
                    f"\n  SERVO: {msg.servo1_raw} {msg.servo2_raw}"
                    f" {msg.servo3_raw} {msg.servo4_raw}"
                )
                last_servo = now2
        elif t == 'LOCAL_POSITION_NED':
            print(f"  z={msg.z:.2f}m", end="\r")
            # in NED: z becomes more negative as drone rises
            # hovering at alt means z ≈ -(alt) from home
            if msg.z < -(alt - TOLERANCE):
                print(f"\n  Takeoff complete (z={msg.z:.2f})")
                return msg.z

    print("\n  Takeoff timeout")
    return None


def land(conn):
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND, 0,
        0, 0, 0, 0, 0, 0, 0
    )
    print("Landing")


def debug_status(conn, uwb, duration=15):
    """Print live debug for EKF, UWB, and position for `duration` seconds."""
    print("\n" + "="*55)
    print(f"DEBUG MODE — watching for {duration}s")
    print("="*55)
    deadline = time.time() + duration
    while time.time() < deadline:
        ekf = conn.recv_match(
            type='EKF_STATUS_REPORT', blocking=False
        )
        pos = conn.recv_match(
            type='LOCAL_POSITION_NED', blocking=False
        )
        drone_pos = uwb.get_drone_pos()
        tags = uwb.get_tags()
        uwb_ok = drone_pos is not None
        vision_sent = getattr(
            uwb, '_dbg_cnt', 0
        )

        print(
            f"  UWB: {'OK' if uwb_ok else 'NO DATA':7s}"
            f"  tags={len(tags)}"
            f"  vision_sent={vision_sent:4d}"
        )
        if drone_pos:
            print(
                f"  UWB pos: x={drone_pos[0]:.2f}"
                f"  y={drone_pos[1]:.2f}"
                f"  z={drone_pos[2]:.2f}"
            )
        if ekf:
            b = ekf.flags
            print(
                f"  EKF flags={b:#06x}"
                f"  att={bool(b&_EKF_ATT)}"
                f"  vel={bool(b&_EKF_VEL_H)}"
                f"  pos_rel={bool(b&_EKF_POS_REL)}"
                f"  pos_abs={bool(b&_EKF_POS_ABS)}"
                f"  const={bool(b&_EKF_CONST)}"
            )
        if pos:
            print(
                f"  LOCAL_NED: x={pos.x:.2f}"
                f"  y={pos.y:.2f}"
                f"  z={pos.z:.2f}"
            )
        print("-"*55)
        time.sleep(1.0)


def _emergency_disarm(conn):
    """Best-effort force disarm — called after any flight exception."""
    print("\n[EMERGENCY] Sending force disarm...")
    try:
        conn.mav.command_long_send(
            conn.target_system, conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            0, 21196,  # param2=21196 = force disarm magic number
            0, 0, 0, 0, 0
        )
    except Exception as e:
        print(f"[EMERGENCY] Disarm failed: {e}")


def _flight(conn, uwb):
    try:
        send_global_origin(conn)
        request_streams(conn)
        print("Waiting for UWB origin...")
        deadline = time.time() + 20.0
        while time.time() < deadline:
            if uwb.get_drone_pos() is not None:
                print("UWB origin locked")
                break
            time.sleep(0.2)
        else:
            print("UWB origin timeout — proceeding anyway")

        debug_status(conn, uwb, duration=15)

        wait_ready(conn)
        print("Setting GUIDED mode...")
        set_guided(conn)
        time.sleep(0.3)
        if not arm(conn):
            print("[FLIGHT] ARM failed — aborting")
            return
        time.sleep(0.5)
        hover_z = takeoff(conn, TAKEOFF_ALT)
        if hover_z is None:
            print("[FLIGHT] Takeoff failed — aborting")
            return
        print("Hovering (2s)...")
        held = hold(conn, 0, 0, hover_z, 2.0, uwb=uwb)
        if held:
            land(conn)
        print("Done")
    except Exception as e:
        print(f"\n[FLIGHT] Exception: {e}")
        _emergency_disarm(conn)
    finally:
        uwb.stop()


# ── visualization (main thread) ──────────────────────────────────────────────

def _update(frame, ax_info, ax_map, uwb):
    now = time.time()
    tags = uwb.get_tags()
    stale = 1.0

    ax_info.cla()
    ax_info.set_title("Tag status")
    ax_info.axis('off')

    rows = []
    for tag_id, info in sorted(tags.items()):
        age = now - info['ts']
        rows.append([
            tag_id,
            f"{info['x']:.2f}", f"{info['y']:.2f}",
            f"{-info['z']:.2f}",
            str(info.get('qf', 0)), f"{age:.1f}s",
        ])

    if rows:
        table = ax_info.table(
            cellText=rows,
            colLabels=['Tag ID', 'X (m)', 'Y (m)', 'Height (m)', 'QF', 'Age'],
            loc='center', cellLoc='center',
        )
        table.auto_set_font_size(False)
        table.set_fontsize(10)
        table.scale(1.2, 2.0)
        for i, (tag_id, _) in enumerate(sorted(tags.items())):
            color = COLORS[i % len(COLORS)]
            for j in range(6):
                table[i + 1, j].set_facecolor(color + '33')
    else:
        ax_info.text(
            0.5, 0.5, 'Waiting for UWB data...',
            ha='center', va='center', fontsize=11, color='gray',
            transform=ax_info.transAxes,
        )

    ax_map.cla()
    ax_map.set_title("Tag position (XY)")
    ax_map.set_xlabel("X (m)")
    ax_map.set_ylabel("Y (m)")
    ax_map.set_aspect('equal')
    ax_map.grid(True, linestyle='--', alpha=0.4)

    all_x, all_y = [], []
    for i, (tag_id, info) in enumerate(sorted(tags.items())):
        color = COLORS[i % len(COLORS)]
        fresh = (now - info['ts']) < stale
        trail = info['trail']

        if len(trail) > 1:
            tx, ty = zip(*trail)
            ax_map.plot(tx, ty, '-', color=color, alpha=0.3, linewidth=1)

        mc = color if fresh else '#aaaaaa'
        ax_map.plot(
            info['x'], info['y'], 'o', color=mc,
            markersize=12, markeredgecolor='white',
            markeredgewidth=1.5, zorder=10,
        )
        ax_map.text(
            info['x'] + 0.1, info['y'] + 0.1,
            f"{tag_id}\nQF={info['qf']}", fontsize=8, color=mc,
        )
        all_x.append(info['x'])
        all_y.append(info['y'])

    if all_x:
        m = 2.0
        ax_map.set_xlim(min(all_x) - m, max(all_x) + m)
        ax_map.set_ylim(min(all_y) - m, max(all_y) + m)
    else:
        ax_map.text(
            0.5, 0.5, 'Waiting for UWB data...',
            ha='center', va='center', fontsize=11, color='gray',
            transform=ax_map.transAxes,
        )


def _heartbeat_loop(conn, stop_evt):
    """Send GCS heartbeat every 1s so ArduPilot knows we're alive."""
    while not stop_evt.is_set():
        try:
            conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0
            )
        except Exception:
            return
        time.sleep(1.0)


def main():
    conn = connect()
    uwb = UWBTag(conn)
    uwb.start()

    stop_hb = threading.Event()
    hb = threading.Thread(target=_heartbeat_loop, args=(conn, stop_hb), daemon=True)
    hb.start()

    ft = threading.Thread(target=_flight, args=(conn, uwb), daemon=True)
    ft.start()

    fig, (ax_info, ax_map) = plt.subplots(1, 2, figsize=(13, 5))
    fig.suptitle("ArduPilot + GrowSpace UWB", fontsize=13, fontweight='bold')
    ax_map.set_aspect('equal')
    fig.ani = animation.FuncAnimation(
        fig, _update, fargs=(ax_info, ax_map, uwb),
        interval=200, cache_frame_data=False,
    )
    plt.tight_layout()
    plt.show()
    ft.join()


if __name__ == '__main__':
    main()
