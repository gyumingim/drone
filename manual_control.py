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
CLIMB_RATE  = 0.05  # m/s
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

# ── flight log (shared with visualization) ───────────────────────────────────

_flight_log: list = []
_flight_log_lock = threading.Lock()
_MAX_LOG = 30


def flog(msg: str) -> None:
    ts = time.strftime('%H:%M:%S')
    line = f"[{ts}] {msg}"
    print(line)
    with _flight_log_lock:
        _flight_log.append(line)
        if len(_flight_log) > _MAX_LOG:
            del _flight_log[0]


# ── FC telemetry (updated by flight thread, read by viz thread) ───────────────

_telem: dict = {
    'phase': 'init',
    'armed': None,
    'mode': None,
    'ned_x': None, 'ned_y': None, 'ned_z': None,
    'servo': None,
    'ekf_flags': None,
    'takeoff_start_z': None,
    'takeoff_target_z': None,
    'ts': 0.0,
}
_telem_lock = threading.Lock()


def _tset(**kw) -> None:
    with _telem_lock:
        _telem.update(kw)
        _telem['ts'] = time.time()


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
    flog("Global origin sent")


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
    flog("Streams requested")


_EKF_ATT      = 0x001   # attitude
_EKF_VEL_H    = 0x002   # horiz velocity
_EKF_POS_REL  = 0x008   # horiz pos (relative)
_EKF_POS_ABS  = 0x010   # horiz pos (absolute)
_EKF_CONST    = 0x080   # constant-position mode (no horiz pos)
_EKF_NEED     = _EKF_ATT | _EKF_VEL_H | _EKF_POS_REL | _EKF_POS_ABS


def wait_ekf_z_stable(conn, uwb, threshold=0.5, ramp_rate=0.3, timeout=90):
    """VISION z offset을 EKF z에서 0으로 서서히 걸어서 baro drift 상쇄.

    EKF z = -16m이면 innovation gate가 UWB z=0을 거부함.
    offset을 -16m에서 시작해 0.3m/s로 0을 향해 ramp하면 EKF가 따라옴.
    """
    # 현재 EKF z 읽기
    ekf_z = None
    for _ in range(10):
        m = conn.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1.0)
        if m:
            ekf_z = m.z
            break
    if ekf_z is None:
        flog("EKF z ramp: no LOCAL_NED — skipping")
        return False

    if abs(ekf_z) < threshold:
        flog(f"EKF z already stable: z={ekf_z:.3f}m — skipping ramp")
        return True

    flog(f"EKF z ramp: start={ekf_z:.2f}m → 0  rate={ramp_rate}m/s")
    uwb.z_offset = ekf_z   # VISION z = UWB_real_z + offset ≈ EKF z (near)
    last_log = 0.0
    deadline = time.time() + timeout
    dt = 0.1  # 100ms step

    while time.time() < deadline:
        # ramp offset toward 0
        if uwb.z_offset > 0:
            uwb.z_offset = max(0.0, uwb.z_offset - ramp_rate * dt)
        else:
            uwb.z_offset = min(0.0, uwb.z_offset + ramp_rate * dt)

        m = conn.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=0.2)
        if m:
            _tset(ned_x=m.x, ned_y=m.y, ned_z=m.z)
            now = time.time()
            if now - last_log > 2.0:
                flog(f"  EKF z={m.z:.3f}m  offset={uwb.z_offset:.3f}m")
                last_log = now
            if abs(m.z) < threshold:
                uwb.z_offset = 0.0
                flog(f"EKF z stable: z={m.z:.3f}m  offset cancelled")
                return True
        time.sleep(dt)

    uwb.z_offset = 0.0
    flog(f"EKF z ramp timeout — proceeding anyway")
    return False


def wait_ready(conn, timeout=15):
    flog("Waiting for EKF...")
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = conn.recv_match(
            type='EKF_STATUS_REPORT', blocking=True, timeout=2
        )
        if msg:
            bits = msg.flags
            _tset(ekf_flags=bits)
            print(
                f"  EKF={bits:#06x} "
                f"att={bool(bits&_EKF_ATT)} "
                f"vel={bool(bits&_EKF_VEL_H)} "
                f"pos_rel={bool(bits&_EKF_POS_REL)} "
                f"pos_abs={bool(bits&_EKF_POS_ABS)} "
                f"const_pos={bool(bits&_EKF_CONST)}"
            )
            if (bits & _EKF_NEED) == _EKF_NEED:
                flog(f"EKF Ready: att vel pos_rel pos_abs all True")
                return
    flog("EKF Timeout — proceeding anyway (pos may be unreliable)")


def set_mode(conn, mode_id):
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id, 0, 0, 0, 0, 0
    )


def set_guided(conn, timeout=5):
    set_mode(conn, 4)   # GUIDED
    deadline = time.time() + timeout
    while time.time() < deadline:
        hb = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
        if hb and hb.custom_mode == 4:
            flog("GUIDED mode confirmed")
            _tset(mode='GUIDED')
            return True
    flog("GUIDED mode NOT confirmed — proceeding anyway")
    return False


def set_althold(conn):
    set_mode(conn, 2)   # ALT_HOLD


def arm(conn, timeout=8):
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, 0, 0, 0, 0, 0, 0
    )
    flog("ARM command sent — waiting for heartbeat...")
    _tset(phase='arming', armed=False)
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = conn.recv_match(
            type=['HEARTBEAT', 'STATUSTEXT'],
            blocking=True, timeout=1.0
        )
        if msg is None:
            continue
        if msg.get_type() == 'STATUSTEXT':
            flog(f"FC: {msg.text.strip()}")
        if msg.get_type() == 'HEARTBEAT':
            armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            if armed:
                flog("Armed OK")
                _tset(armed=True, phase='armed')
                return True
    flog("ARM FAILED — check pre-arm messages above")
    _tset(phase='arm_failed')
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
    flog(f"goto ({x:.1f}, {y:.1f}, alt={-z:.1f}m)")
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
                    flog(f"Reached target ({x:.1f},{y:.1f},{-z:.1f}m)")
                    return True
            elif msg.get_type() == 'SERVO_OUTPUT_RAW':
                now2 = time.time()
                if now2 - last_servo > 3.0:
                    flog(
                        f"SERVO: {msg.servo1_raw} {msg.servo2_raw}"
                        f" {msg.servo3_raw} {msg.servo4_raw}"
                    )
                    last_servo = now2
        time.sleep(0.05)
    flog("goto: Timeout")
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
                flog("[SAFETY] UWB signal lost during hold — landing")
                land(conn)
                return False
        time.sleep(0.1)
    return True


def takeoff(conn, alt=TAKEOFF_ALT):
    """Takeoff using MAV_CMD_NAV_TAKEOFF then wait for altitude."""
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
        0, 0, 0, float('nan'),   # min_pitch, empty, empty, yaw
        0, 0, alt,               # lat, lon, altitude (m above home)
    )
    flog(f"TAKEOFF cmd sent, target={alt}m")
    ack = conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    if ack:
        flog(f"TAKEOFF ACK: {'OK' if ack.result==0 else 'FAIL'} (result={ack.result})")
    else:
        flog("TAKEOFF: no ACK — FC may not be ready")

    _tset(phase='takeoff')
    start_z = None
    last_log = 0.0
    last_servo = 0.0
    deadline = time.time() + alt / CLIMB_RATE * 5
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
            _tset(armed=bool(armed))
            if not armed:
                flog("TAKEOFF ABORTED: drone disarmed mid-flight!")
                _tset(phase='disarmed')
                return None
        elif t == 'SERVO_OUTPUT_RAW':
            _tset(servo=(msg.servo1_raw, msg.servo2_raw,
                         msg.servo3_raw, msg.servo4_raw))
            now2 = time.time()
            if now2 - last_servo > 2.0:
                flog(
                    f"SERVO: {msg.servo1_raw} {msg.servo2_raw}"
                    f" {msg.servo3_raw} {msg.servo4_raw}"
                    f" {'(motors off!)' if msg.servo1_raw <= 1000 else ''}"
                )
                last_servo = now2
        elif t == 'LOCAL_POSITION_NED':
            _tset(ned_x=msg.x, ned_y=msg.y, ned_z=msg.z)
            if start_z is None:
                start_z = msg.z
                threshold = start_z - (alt - TOLERANCE)
                _tset(takeoff_start_z=start_z, takeoff_target_z=threshold)
                flog(
                    f"TAKEOFF: start_z={start_z:.2f}m  "
                    f"need z<{threshold:.2f}m  (Δ≥{alt-TOLERANCE:.1f}m)"
                )
                if start_z < -(alt - 0.2):
                    flog(
                        f"  WARNING: start_z={start_z:.2f}m already below "
                        f"target {threshold:.2f}m — baro drift detected!"
                    )
            threshold = start_z - (alt - TOLERANCE)
            now2 = time.time()
            if now2 - last_log > 1.0:
                delta = start_z - msg.z
                flog(f"  climbing: z={msg.z:.2f}m  Δ={delta:.2f}/{alt:.1f}m")
                last_log = now2
            print(f"  z={msg.z:.2f}m", end="\r")
            if msg.z < threshold:
                delta = start_z - msg.z
                flog(
                    f"Takeoff complete: z={msg.z:.2f}m  "
                    f"Δ={delta:.2f}m from start"
                )
                return (msg.x, msg.y, msg.z)

    flog("TAKEOFF TIMEOUT — drone may not have moved")
    return None


def land(conn):
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND, 0,
        0, 0, 0, 0, 0, 0, 0
    )
    flog("Landing command sent")
    _tset(phase='landing')


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
        vision_sent = getattr(uwb, '_dbg_cnt', 0)

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
        if pos and drone_pos:
            dx = drone_pos[0] - pos.x
            dy = drone_pos[1] - pos.y
            dz = drone_pos[2] - pos.z
            print(
                f"  DRIFT  Δx={dx:+.3f}  Δy={dy:+.3f}  Δz={dz:+.3f}"
                f"  (UWB - EKF, 작을수록 정렬됨)"
            )
        print("-"*55)
        time.sleep(1.0)


def _emergency_disarm(conn):
    """Best-effort force disarm — called after any flight exception."""
    flog("[EMERGENCY] Sending force disarm...")
    try:
        conn.mav.command_long_send(
            conn.target_system, conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            0, 21196,  # param2=21196 = force disarm magic number
            0, 0, 0, 0, 0
        )
    except Exception as e:
        flog(f"[EMERGENCY] Disarm failed: {e}")


def _flight(conn, uwb):
    try:
        send_global_origin(conn)
        request_streams(conn)
        flog("Waiting for UWB origin...")
        deadline = time.time() + 20.0
        while time.time() < deadline:
            if uwb.get_drone_pos() is not None:
                flog("UWB origin locked — ready to fly")
                break
            time.sleep(0.2)
        else:
            flog("UWB origin timeout — proceeding anyway")

        debug_status(conn, uwb, duration=15)

        wait_ready(conn)
        flog("Setting GUIDED mode...")
        set_guided(conn)
        time.sleep(0.3)
        wait_ekf_z_stable(conn, uwb)
        if not arm(conn):
            flog("[FLIGHT] ARM failed — aborting")
            return
        time.sleep(0.5)
        result = takeoff(conn, TAKEOFF_ALT)
        if result is None:
            flog("[FLIGHT] Takeoff failed — aborting")
            return
        hover_x, hover_y, hover_z = result
        flog(f"Hovering 2s at ({hover_x:.2f},{hover_y:.2f},{hover_z:.2f})...")
        _tset(phase='hover')
        held = hold(conn, hover_x, hover_y, hover_z, 2.0, uwb=uwb)
        if held:
            land(conn)
        flog("Done")
        _tset(phase='done', armed=False)
    except Exception as e:
        flog(f"[FLIGHT] Exception: {e}")
        _emergency_disarm(conn)


# ── visualization (main thread) ──────────────────────────────────────────────

def _draw_fc_panel(ax, uwb):
    ax.cla()
    ax.set_title("FC Telemetry")
    ax.axis('off')

    with _telem_lock:
        t = dict(_telem)

    lines = []

    # phase / armed / mode
    phase = t.get('phase', 'init')
    armed = t.get('armed')
    mode  = t.get('mode') or '?'
    arm_str = 'ARMED' if armed else ('DISARMED' if armed is not None else '?')
    lines.append(f"Phase : {phase.upper():<12}  Mode: {mode}")
    lines.append(f"Armed : {arm_str}")
    lines.append("")

    # LOCAL_NED z — highlight baro drift
    ned_z = t.get('ned_z')
    ned_x = t.get('ned_x') or 0.0
    ned_y = t.get('ned_y') or 0.0
    if ned_z is not None:
        drift = ned_z < -0.5
        lines.append(
            f"LOCAL_NED  x={ned_x:+.2f}  y={ned_y:+.2f}  z={ned_z:+.2f}m"
            + ("  ← BARO DRIFT" if drift else "")
        )
    else:
        lines.append("LOCAL_NED: waiting...")
    lines.append("")

    # takeoff progress
    start_z  = t.get('takeoff_start_z')
    target_z = t.get('takeoff_target_z')
    if start_z is not None:
        lines.append(f"Takeoff start_z  = {start_z:+.2f}m")
        lines.append(f"        target_z = {target_z:+.2f}m"
                     f"  (need Δ≥{TAKEOFF_ALT-TOLERANCE:.1f}m)")
        if ned_z is not None:
            delta = start_z - ned_z
            pct   = max(0.0, min(delta / TAKEOFF_ALT, 1.0))
            bar   = '█' * int(pct * 16) + '░' * (16 - int(pct * 16))
            lines.append(f"        climbed  = {delta:.2f}/{TAKEOFF_ALT:.1f}m"
                         f"  [{bar}]")
        if start_z < -(TAKEOFF_ALT - 0.2):
            lines.append("  !! start_z already below target — baro drift!")
    lines.append("")

    # SERVO
    servo = t.get('servo')
    if servo is not None:
        motors_on = any(v > 1050 for v in servo)
        lines.append(
            f"SERVO: {servo[0]:4d} {servo[1]:4d} {servo[2]:4d} {servo[3]:4d}"
            + ("" if motors_on else "  ← MOTORS OFF")
        )
    else:
        lines.append("SERVO: waiting...")
    lines.append("")

    # EKF
    ekf = t.get('ekf_flags')
    if ekf is not None:
        ok = (ekf & _EKF_NEED) == _EKF_NEED
        lines.append(f"EKF: {'READY' if ok else 'NOT READY'}  ({ekf:#06x})")
        lines.append(
            f"  att={bool(ekf&_EKF_ATT)}"
            f"  vel={bool(ekf&_EKF_VEL_H)}"
            f"  pos_rel={bool(ekf&_EKF_POS_REL)}"
            f"  pos_abs={bool(ekf&_EKF_POS_ABS)}"
        )
    lines.append("")

    # UWB-EKF z drift
    tags = uwb.get_tags()
    if tags and ned_z is not None:
        info = next(iter(tags.values()))
        uwb_z = info['z']
        dz    = uwb_z - ned_z
        lines.append(f"UWB z={uwb_z:+.2f}m  EKF z={ned_z:+.2f}m  Δz={dz:+.2f}m")

    # color
    if start_z is not None and start_z < -(TAKEOFF_ALT - 0.2):
        col = '#c0392b'
    elif ned_z is not None and ned_z < -0.5:
        col = '#e67e22'
    elif armed:
        col = '#27ae60'
    else:
        col = '#2c3e50'

    ax.text(0.03, 0.97, '\n'.join(lines),
            transform=ax.transAxes,
            va='top', fontsize=8, family='monospace', color=col)


def _update(frame, ax_info, ax_map, ax_fc, ax_log, uwb):
    now = time.time()
    tags = uwb.get_tags()
    stale = 1.0

    # ── UWB table ────────────────────────────────────────────────────────────
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

    # ── XY map ───────────────────────────────────────────────────────────────
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

    # ── FC telemetry ─────────────────────────────────────────────────────────
    _draw_fc_panel(ax_fc, uwb)

    # ── flight log ───────────────────────────────────────────────────────────
    ax_log.cla()
    ax_log.set_title("Flight log")
    ax_log.axis('off')

    with _flight_log_lock:
        lines = list(_flight_log)

    if lines:
        # color-code by keyword
        display = lines[-20:]
        txt = '\n'.join(display)
        last = display[-1].lower() if display else ''
        if 'fail' in last or 'abort' in last or 'emergency' in last or 'disarm' in last:
            color = '#e74c3c'
        elif 'safety' in last or 'lost' in last or 'timeout' in last:
            color = '#e67e22'
        elif 'done' in last or 'complete' in last or 'ok' in last or 'ready' in last:
            color = '#27ae60'
        else:
            color = '#2c3e50'
        ax_log.text(
            0.03, 0.97, txt,
            transform=ax_log.transAxes,
            va='top', fontsize=7.5, family='monospace', color=color,
            wrap=True,
        )
    else:
        ax_log.text(
            0.5, 0.5, 'Waiting for flight events...',
            ha='center', va='center', fontsize=10, color='gray',
            transform=ax_log.transAxes,
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

    fig = plt.figure(figsize=(18, 9))
    fig.suptitle("ArduPilot + GrowSpace UWB", fontsize=13, fontweight='bold')
    ax_info = fig.add_subplot(2, 2, 1)
    ax_map  = fig.add_subplot(2, 2, 2)
    ax_fc   = fig.add_subplot(2, 2, 3)
    ax_log  = fig.add_subplot(2, 2, 4)
    ax_map.set_aspect('equal')
    fig.ani = animation.FuncAnimation(
        fig, _update, fargs=(ax_info, ax_map, ax_fc, ax_log, uwb),
        interval=200, cache_frame_data=False,
    )
    plt.tight_layout()
    plt.show()
    ft.join()
    uwb.stop()
    stop_hb.set()


if __name__ == '__main__':
    main()