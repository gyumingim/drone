"""
ArduPilot manual control + GrowSpace UWB Listener integration
- Reads tag position from Listener (lep)
- Injects VISION_POSITION_ESTIMATE into ArduPilot EKF
- Live matplotlib visualization runs in main thread
- Flight control runs in background thread
"""

import time
import threading
from collections import deque
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

STEP_M    = 0.30   # 한 방향 이동 거리 (m)
STEP_HOLD = 2.0    # 각 웨이포인트 정지 시간 (s)

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
    'baro_alt': None,
    'yaw_rad': None,
    'roll_rad': None,
    'pitch_rad': None,
    'ts': 0.0,
}
_telem_lock = threading.Lock()

_alt_history: deque = deque(maxlen=300)  # (timestamp, baro_alt, ekf_alt)


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
    """Ask ArduPilot to send EKF, position, attitude, servo, vfr streams."""
    streams = [
        mavutil.mavlink.MAV_DATA_STREAM_EXTRA3,    # EKF_STATUS_REPORT
        mavutil.mavlink.MAV_DATA_STREAM_POSITION,   # LOCAL_POSITION_NED
        mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,     # ATTITUDE
        mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, # SERVO_OUTPUT_RAW
        mavutil.mavlink.MAV_DATA_STREAM_EXTRA2,     # VFR_HUD (baro alt)
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
        att = conn.recv_match(type='ATTITUDE', blocking=False)
        if att and uwb is not None:
            uwb.set_yaw(att.yaw)
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
                return msg.z

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
    # 북(+x) → 동(+y) → 남(-x) → 서(-y) 정사각형 미션 후 착륙
    try:
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
        if not arm(conn):
            flog("[FLIGHT] ARM failed — aborting")
            return
        time.sleep(0.5)
        hover_z = takeoff(conn, TAKEOFF_ALT)
        if hover_z is None:
            flog("[FLIGHT] Takeoff failed — aborting")
            return

        # 이륙 직후 EKF x,y 기준점 캡처
        pos0 = conn.recv_match(type='LOCAL_POSITION_NED',
                               blocking=True, timeout=2)
        sx = pos0.x if pos0 else 0.0
        sy = pos0.y if pos0 else 0.0
        flog(f"기준점 잠금: x={sx:.2f} y={sy:.2f} z={hover_z:.2f}")

        # NED 기준: 북=+x, 동=+y
        waypoints = [
            ("북 +30cm", sx + STEP_M, sy),
            ("동 +30cm", sx + STEP_M, sy + STEP_M),
            ("남 -30cm", sx,          sy + STEP_M),
            ("서 -30cm", sx,          sy),
        ]

        _tset(phase='mission')
        for label, tx, ty in waypoints:
            flog(f"→ {label}  target=({tx:.2f},{ty:.2f})")
            if not goto(conn, tx, ty, hover_z):
                flog("[MISSION] goto 타임아웃 — 착륙")
                land(conn)
                return
            flog(f"  {label} 도착 — {STEP_HOLD}s 정지")
            if not hold(conn, tx, ty, hover_z, STEP_HOLD, uwb=uwb):
                flog("[MISSION] UWB 신호 손실 — 중단")
                return

        flog("미션 완료 — 착륙")
        land(conn)
        flog("Done")
        _tset(phase='done', armed=False)
    except Exception as e:
        flog(f"[FLIGHT] Exception: {e}")
        _emergency_disarm(conn)


# ── visualization (main thread) ──────────────────────────────────────────────

import math as _math


def _draw_status(ax):
    """Panel: ARM 상태, Phase, EKF 플래그 — 가장 중요한 패널."""
    ax.cla()
    ax.axis('off')

    with _telem_lock:
        t = dict(_telem)

    armed   = t.get('armed')
    phase   = t.get('phase', 'init').upper()
    mode    = t.get('mode') or '?'
    ekf     = t.get('ekf_flags')
    ned_z   = t.get('ned_z')
    ned_x   = t.get('ned_x') or 0.0
    ned_y   = t.get('ned_y') or 0.0
    baro    = t.get('baro_alt')
    yaw     = t.get('yaw_rad')
    roll    = t.get('roll_rad')
    pitch   = t.get('pitch_rad')

    # ARM 배너
    if armed:
        bg_col = '#27ae60'; arm_txt = '● ARMED'
    elif armed is False:
        bg_col = '#c0392b'; arm_txt = '○ DISARMED'
    else:
        bg_col = '#7f8c8d'; arm_txt = '? UNKNOWN'

    ax.set_facecolor(bg_col + '22')
    ax.text(0.5, 0.97, arm_txt, transform=ax.transAxes,
            ha='center', va='top', fontsize=22, fontweight='bold',
            color=bg_col, family='monospace')

    # Phase
    _phase_color = {
        'INIT': '#7f8c8d', 'ARMING': '#f39c12', 'ARMED': '#f39c12',
        'TAKEOFF': '#3498db', 'HOVER': '#2ecc71', 'MISSION': '#9b59b6',
        'LANDING': '#e67e22', 'DONE': '#27ae60',
        'ARM_FAILED': '#e74c3c', 'DISARMED': '#e74c3c',
    }
    pc = _phase_color.get(phase, '#2c3e50')
    ax.text(0.5, 0.80, f'PHASE: {phase}', transform=ax.transAxes,
            ha='center', va='top', fontsize=13, color=pc, fontweight='bold')
    ax.text(0.5, 0.71, f'MODE: {mode}', transform=ax.transAxes,
            ha='center', va='top', fontsize=10, color='#2c3e50')

    # EKF 상태
    if ekf is not None:
        ok = (ekf & _EKF_NEED) == _EKF_NEED
        ec = '#27ae60' if ok else '#e74c3c'
        ax.text(0.5, 0.63, '✓ EKF READY' if ok else '✗ EKF NOT READY',
                transform=ax.transAxes, ha='center', va='top',
                fontsize=11, color=ec, fontweight='bold')
        flags = (f"  att={'✓' if ekf & _EKF_ATT else '✗'}"
                 f"  vel={'✓' if ekf & _EKF_VEL_H else '✗'}"
                 f"  pos_rel={'✓' if ekf & _EKF_POS_REL else '✗'}"
                 f"  pos_abs={'✓' if ekf & _EKF_POS_ABS else '✗'}")
        ax.text(0.5, 0.56, flags, transform=ax.transAxes,
                ha='center', va='top', fontsize=8.5, family='monospace',
                color=ec)

    # 수치 정보
    lines = ['']
    if ned_z is not None:
        lines.append(f"EKF 고도   : {-ned_z:+.3f} m")
        lines.append(f"EKF x,y    : ({ned_x:+.2f}, {ned_y:+.2f}) m")
    if baro is not None:
        lines.append(f"Baro 고도  : {baro:+.3f} m")
    if roll is not None:
        lines.append(f"Roll/Pitch : {_math.degrees(roll):+.1f}° / "
                     f"{_math.degrees(pitch):+.1f}°")
    if yaw is not None:
        lines.append(f"Yaw        : {_math.degrees(yaw):+.1f}°")

    # 이륙 진행 바
    start_z  = t.get('takeoff_start_z')
    target_z = t.get('takeoff_target_z')
    if start_z is not None and ned_z is not None:
        delta = start_z - ned_z
        pct   = max(0.0, min(delta / TAKEOFF_ALT, 1.0))
        bar   = '█' * int(pct * 14) + '░' * (14 - int(pct * 14))
        lines.append('')
        lines.append(f"이륙 [{bar}]")
        lines.append(f"     {delta:.2f} / {TAKEOFF_ALT:.1f} m")

    ax.text(0.05, 0.49, '\n'.join(lines), transform=ax.transAxes,
            va='top', fontsize=9, family='monospace', color='#2c3e50')


def _draw_map(ax, uwb):
    """Panel: XY 맵 — UWB 위치, 앵커, yaw 화살표, EKF 위치."""
    ax.cla()
    ax.set_aspect('equal')
    ax.grid(True, linestyle='--', alpha=0.4)

    with _telem_lock:
        yaw   = _telem.get('yaw_rad')
        ned_x = _telem.get('ned_x')
        ned_y = _telem.get('ned_y')

    now   = time.time()
    tags  = uwb.get_tags()
    all_x, all_y = [], []

    for i, (tag_id, info) in enumerate(sorted(tags.items())):
        color = COLORS[i % len(COLORS)]
        fresh = (now - info['ts']) < 1.0
        trail = info['trail']

        # 이동 궤적
        if len(trail) > 1:
            tx, ty = zip(*trail)
            ax.plot(tx, ty, '-', color=color, alpha=0.3, linewidth=1)

        # 드론 점
        mc = color if fresh else '#aaaaaa'
        ax.plot(info['x'], info['y'], 'o', color=mc,
                markersize=14, markeredgecolor='white',
                markeredgewidth=2, zorder=10)

        # yaw 화살표 (나침반 방향)
        if yaw is not None:
            L = 0.5
            dx = L * _math.cos(yaw)
            dy = L * _math.sin(yaw)
            ax.annotate('',
                        xy=(info['x'] + dx, info['y'] + dy),
                        xytext=(info['x'], info['y']),
                        arrowprops=dict(arrowstyle='->', color='#f39c12',
                                        lw=2.5),
                        zorder=12)
            ax.text(info['x'] + dx * 1.3, info['y'] + dy * 1.3,
                    f"{_math.degrees(yaw):.0f}°",
                    fontsize=9, color='#f39c12', fontweight='bold',
                    ha='center', va='center')

        ax.text(info['x'] + 0.12, info['y'] - 0.28,
                f"QF={info['qf']}", fontsize=8, color=mc)
        all_x.append(info['x'])
        all_y.append(info['y'])

        # 앵커 위치 및 거리선
        for aid, av in sorted(info.get('anchors', {}).items()):
            apx, apy, apz = av['pos']
            ax.plot(apx, apy, 's', color='#555555', markersize=10, zorder=5)
            ax.text(apx + 0.1, apy + 0.1,
                    f"{aid}\n({apx:.1f},{apy:.1f},z={apz:.1f})",
                    fontsize=7, color='#555555')
            ax.plot([info['x'], apx], [info['y'], apy],
                    ':', color='#bbbbbb', linewidth=0.8, alpha=0.7)
            mid_x = (info['x'] + apx) / 2
            mid_y = (info['y'] + apy) / 2
            ax.text(mid_x, mid_y, f"{av['dist_m']:.2f}m",
                    fontsize=7, color='#888888', ha='center')
            all_x.append(apx)
            all_y.append(apy)

    # EKF 위치 (파란 삼각형)
    if ned_x is not None and ned_y is not None:
        ax.plot(ned_x, ned_y, '^', color='#3498db', markersize=11,
                markeredgecolor='white', markeredgewidth=1.5,
                zorder=9, label='EKF')
        all_x.append(ned_x)
        all_y.append(ned_y)

    if all_x:
        m = 2.0
        xl = min(all_x) - m;  xr = max(all_x) + m
        yl = min(all_y) - m;  yr = max(all_y) + m
        ax.set_xlim(xl, xr)
        ax.set_ylim(yl, yr)
        # 북쪽(+x) 기준 화살표
        nx = xl + 0.3;  ny = yr - 0.5
        ax.annotate('', xy=(nx + 0.5, ny), xytext=(nx, ny),
                    arrowprops=dict(arrowstyle='->', color='#2c3e50', lw=1.5))
        ax.text(nx + 0.55, ny, 'N(+x)', fontsize=8, color='#2c3e50',
                va='center')
    else:
        ax.text(0.5, 0.5, 'UWB 데이터 없음', ha='center', va='center',
                fontsize=11, color='gray', transform=ax.transAxes)

    if ned_x is not None:
        ax.legend(fontsize=8, loc='lower right')

    yaw_str = f"{_math.degrees(yaw):.1f}°" if yaw is not None else "N/A"
    ax.set_xlabel("X (m)  ← 나침반 북쪽")
    ax.set_ylabel("Y (m)  ← 나침반 동쪽")
    ax.set_title(f"UWB 위치 맵   yaw={yaw_str}")


def _draw_altitude(ax):
    """Panel: Baro / EKF / UWB 고도 시계열."""
    ax.cla()
    ax.grid(True, linestyle='--', alpha=0.4)

    with _telem_lock:
        hist     = list(_alt_history)
        cur_baro = _telem.get('baro_alt')
        ned_z    = _telem.get('ned_z')

    if not hist:
        ax.set_title("고도 (m)")
        ax.text(0.5, 0.5, 'Baro 대기 중...',
                ha='center', va='center', transform=ax.transAxes,
                color='gray', fontsize=10)
        return

    now  = time.time()
    ts   = [h[0] - now for h in hist]
    baro = [h[1] for h in hist]
    ekfa = [h[2] for h in hist]
    uwbh = [h[3] if len(h) > 3 else None for h in hist]

    ax.plot(ts, baro, color='#3498db', linewidth=2, label='Baro')

    vekf = [(t, e) for t, e in zip(ts, ekfa) if e is not None]
    if vekf:
        te, ve = zip(*vekf)
        ax.plot(te, ve, color='#e74c3c', linewidth=1.5,
                linestyle='--', label='EKF')

    vuwb = [(t, u) for t, u in zip(ts, uwbh) if u is not None]
    if vuwb:
        tu, vu = zip(*vuwb)
        ax.plot(tu, vu, color='#2ecc71', linewidth=1.5,
                linestyle=':', label='UWB')

    ax.legend(fontsize=8)
    ax.set_xlim(max(ts[0], -60), 2)
    ax.set_xlabel("time (s ago)")
    ax.set_ylabel("alt (m)")

    parts = []
    if cur_baro is not None:
        parts.append(f"Baro={cur_baro:+.2f}m")
    if ned_z is not None:
        parts.append(f"EKF={-ned_z:+.2f}m")
    ax.set_title("고도  " + "  ".join(parts))


def _draw_sensors(ax, uwb):
    """Panel: 모터 PWM, 자세, UWB 앵커 거리 수치."""
    ax.cla()
    ax.axis('off')

    with _telem_lock:
        t = dict(_telem)

    lines = ['─── 액추에이터 / 센서 ───', '']

    # 모터 PWM 바
    servo = t.get('servo')
    if servo is not None:
        lines.append('모터 PWM:')
        for i, v in enumerate(servo):
            pct = max(0.0, min((v - 1000) / 1000, 1.0))
            bar = '█' * int(pct * 10) + '░' * (10 - int(pct * 10))
            on  = v > 1050
            lines.append(f"  M{i+1}: {v:4d} [{bar}] {'▲ON' if on else 'off'}")
    else:
        lines.append('모터: 대기 중')
    lines.append('')

    # 자세 (Roll / Pitch / Yaw)
    roll  = t.get('roll_rad')
    pitch = t.get('pitch_rad')
    yaw   = t.get('yaw_rad')
    if roll is not None:
        lines.append('자세:')
        for name, rad in [('Roll ', roll), ('Pitch', pitch), ('Yaw  ', yaw)]:
            deg = _math.degrees(rad)
            pct = (deg + 180) / 360
            bar = '█' * int(pct * 10) + '░' * (10 - int(pct * 10))
            lines.append(f"  {name}: {deg:+6.1f}° [{bar}]")
    lines.append('')

    # UWB 수치
    drone_pos = uwb.get_drone_pos()
    vision_n  = getattr(uwb, '_dbg_cnt', 0)
    lines.append(f'VISION 전송: {vision_n}회')
    if drone_pos:
        rx, ry, rz = drone_pos
        lines.append(f'UWB 높이  : {-rz:+.3f} m')
        lines.append(f'UWB rel   : ({rx:+.2f}, {ry:+.2f})')
    tags = uwb.get_tags()
    if tags:
        info = next(iter(tags.values()))
        fw = info.get('fw_pos')
        if fw:
            lines.append(f'fw_pos z  : {fw[2]:+.3f}m  qf={fw[3]}')
        lines.append('앵커 거리:')
        for aid, av in sorted(info.get('anchors', {}).items()):
            lines.append(f"  {aid}: {av['dist_m']:.3f} m")
    lines.append('')

    # EKF-UWB 드리프트
    ned_x = t.get('ned_x') or 0.0
    ned_y = t.get('ned_y') or 0.0
    ned_z = t.get('ned_z')
    if drone_pos and ned_z is not None:
        dx = drone_pos[0] - ned_x
        dy = drone_pos[1] - ned_y
        dz = drone_pos[2] - ned_z
        lines.append(f'EKF-UWB Δ:')
        lines.append(f'  Δx={dx:+.3f} Δy={dy:+.3f} Δz={dz:+.3f}')

    ax.text(0.03, 0.97, '\n'.join(lines),
            transform=ax.transAxes, va='top',
            fontsize=8.5, family='monospace', color='#2c3e50')


def _draw_log(ax):
    """Panel: 비행 로그 전체 — 줄별 색상 코딩."""
    ax.cla()
    ax.set_title("Flight Log")
    ax.axis('off')

    with _flight_log_lock:
        lines = list(_flight_log)

    if not lines:
        ax.text(0.5, 0.5, '로그 대기 중...',
                ha='center', va='center', fontsize=10, color='gray',
                transform=ax.transAxes)
        return

    display = lines[-30:]
    for i, line in enumerate(reversed(display)):
        y = 0.97 - i * 0.032
        ll = line.lower()
        if any(k in ll for k in ('fail', 'abort', 'emergency', 'error')):
            c = '#e74c3c'
        elif any(k in ll for k in ('safety', 'lost', 'timeout', 'warn')):
            c = '#e67e22'
        elif any(k in ll for k in ('armed ok', 'takeoff complete', 'done',
                                    'ready', 'mission')):
            c = '#27ae60'
        elif any(k in ll for k in ('arm', 'takeoff', 'climbing',
                                    'landing', 'goto', '→')):
            c = '#3498db'
        else:
            c = '#2c3e50'
        ax.text(0.02, y, line, transform=ax.transAxes,
                va='top', fontsize=7.5, family='monospace', color=c,
                clip_on=True)


def _update(frame, ax_status, ax_map, ax_alt, ax_sensor, ax_log, uwb):
    _draw_status(ax_status)
    _draw_map(ax_map, uwb)
    _draw_altitude(ax_alt)
    _draw_sensors(ax_sensor, uwb)
    _draw_log(ax_log)


def _heartbeat_loop(conn, stop_evt, uwb):
    """Send GCS heartbeat every 1s and collect all telemetry for display."""
    while not stop_evt.is_set():
        try:
            conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0
            )
        except Exception:
            return
        msg = conn.recv_match(type='VFR_HUD', blocking=False)
        if msg:
            with _telem_lock:
                _telem['baro_alt'] = msg.alt
                ned_z = _telem.get('ned_z')
            ekf_alt = -ned_z if ned_z is not None else None
            drone_pos = uwb.get_drone_pos()
            uwb_h = -drone_pos[2] if drone_pos else None
            _alt_history.append((time.time(), msg.alt, ekf_alt, uwb_h))
        att = conn.recv_match(type='ATTITUDE', blocking=False)
        if att:
            with _telem_lock:
                _telem['yaw_rad']   = att.yaw
                _telem['roll_rad']  = att.roll
                _telem['pitch_rad'] = att.pitch
        time.sleep(1.0)


def monitor_mode(conn, uwb):
    """
    Read-only monitor: UWB / Baro 섹션 분리 출력.
    No arming, no flight commands.  Ctrl-C to exit.
    """
    import math
    print("=" * 60)
    print("  MONITOR MODE  (Ctrl-C to exit, no arming)")
    print("=" * 60)

    _ekf = _pos = _baro = _att = None

    while True:
        for _ in range(50):
            msg = conn.recv_match(
                type=['EKF_STATUS_REPORT', 'LOCAL_POSITION_NED',
                      'VFR_HUD', 'ATTITUDE'],
                blocking=False,
            )
            if msg is None:
                break
            t = msg.get_type()
            if t == 'EKF_STATUS_REPORT':  _ekf  = msg
            elif t == 'LOCAL_POSITION_NED': _pos  = msg
            elif t == 'VFR_HUD':           _baro = msg
            elif t == 'ATTITUDE':          _att  = msg

        drone_pos = uwb.get_drone_pos()
        tags      = uwb.get_tags()
        vision_n  = getattr(uwb, '_dbg_cnt', 0)

        ts = time.strftime('%H:%M:%S')
        print(f"\n[{ts}]")

        # ══ UWB ══════════════════════════════════════════════════
        print("  [ UWB ]")
        if drone_pos:
            rx, ry, rz = drone_pos
            uwb_h = -rz          # rel NED z → 물리 고도 (양수=위)
            print(f"    고도(trilat) : {uwb_h:+.3f} m")
            print(f"    rel x,y      : ({rx:+.3f}, {ry:+.3f}) m")
            print(f"    vision 전송  : {vision_n} 회")
        else:
            print(f"    NO DATA  (vision 전송: {vision_n} 회)")

        if tags:
            info = next(iter(tags.values()))
            fw = info.get('fw_pos')
            if fw:
                # fw_pos z 는 DWM 펌웨어 추정값 (앵커 배치 낮으면 음수로 틀림)
                print(f"    fw_pos z     : {fw[2]:+.3f} m  (qf={fw[3]})")
            anchors = info.get('anchors', {})
            for aid, v in sorted(anchors.items()):
                ax, ay, az = v['pos']
                print(f"    앵커 {aid}: 거리={v['dist_m']:.3f} m"
                      f"  위치=({ax:.2f},{ay:.2f},{az:.2f})")

        # ══ BARO / EKF ═══════════════════════════════════════════
        print("  [ BARO / EKF ]")
        if _baro:
            print(f"    기압계 고도  : {_baro.alt:+.3f} m  "
                  f"(climb={_baro.climb:+.2f} m/s)")
        else:
            print("    기압계       : 데이터 없음")

        if _pos:
            ekf_h = -_pos.z      # NED z → 물리 고도
            print(f"    EKF 고도     : {ekf_h:+.3f} m  (NED z={_pos.z:+.3f})")
            print(f"    EKF x,y      : ({_pos.x:+.3f}, {_pos.y:+.3f}) m")
        else:
            print("    EKF 위치     : 데이터 없음")

        if _ekf:
            b = _ekf.flags
            ok = (b & _EKF_NEED) == _EKF_NEED
            print(f"    EKF 상태     : {'READY' if ok else 'NOT READY'}"
                  f"  ({b:#06x})")

        if _att:
            print(f"    자세         : roll={math.degrees(_att.roll):+.1f}°"
                  f"  pitch={math.degrees(_att.pitch):+.1f}°"
                  f"  yaw={math.degrees(_att.yaw):+.1f}°")

        # ══ 비교 ═════════════════════════════════════════════════
        if drone_pos and _pos:
            uwb_h  = -drone_pos[2]
            ekf_h  = -_pos.z
            baro_h = _baro.alt if _baro else None
            print("  [ 비교 ]")
            print(f"    UWB고도={uwb_h:+.3f}  EKF고도={ekf_h:+.3f}"
                  + (f"  Baro={baro_h:+.3f}" if baro_h is not None else ""))
            print(f"    UWB-EKF Δz={uwb_h - ekf_h:+.4f} m")

        time.sleep(1.0)


def main():
    import sys
    from matplotlib.gridspec import GridSpec

    monitor = '--monitor' in sys.argv

    conn = connect()
    uwb = UWBTag(conn)
    uwb.start()

    request_streams(conn)

    stop_hb = threading.Event()
    hb = threading.Thread(target=_heartbeat_loop,
                          args=(conn, stop_hb, uwb), daemon=True)
    hb.start()

    if monitor:
        try:
            monitor_mode(conn, uwb)
        except KeyboardInterrupt:
            print("\nMonitor stopped.")
        finally:
            uwb.stop()
            stop_hb.set()
        return

    ft = threading.Thread(target=_flight, args=(conn, uwb), daemon=True)
    ft.start()

    # ── 레이아웃: 2행 3열, 맵이 가운데 열 전체 차지 ──────────────────────────
    fig = plt.figure(figsize=(26, 10))
    fig.suptitle("ArduPilot + GrowSpace UWB  |  실내 자율비행",
                 fontsize=13, fontweight='bold')
    gs = GridSpec(2, 3, figure=fig,
                  left=0.04, right=0.97, top=0.93, bottom=0.06,
                  hspace=0.35, wspace=0.32)

    ax_status = fig.add_subplot(gs[0, 0])   # 상태/ARM
    ax_map    = fig.add_subplot(gs[:, 1])   # XY 맵 (전체 높이)
    ax_alt    = fig.add_subplot(gs[0, 2])   # 고도 그래프
    ax_sensor = fig.add_subplot(gs[1, 0])   # 센서/모터
    ax_log    = fig.add_subplot(gs[1, 2])   # 비행 로그

    fig.ani = animation.FuncAnimation(
        fig, _update,
        fargs=(ax_status, ax_map, ax_alt, ax_sensor, ax_log, uwb),
        interval=200, cache_frame_data=False,
    )
    plt.show()
    ft.join()
    uwb.stop()
    stop_hb.set()


if __name__ == '__main__':
    main()