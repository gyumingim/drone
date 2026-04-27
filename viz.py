"""
viz.py — shared telemetry, flight primitives, and CLI dashboard.
Imported by manual_control.py and flight_cardinal.py.
"""

import re
import sys
import time
import math
import threading
from collections import deque
from pymavlink import mavutil
from uwb_tag import UWBTag

# ── ANSI ──────────────────────────────────────────────────────────────────────
_RST = '\033[0m'
_BLD = '\033[1m'
_GRN = '\033[32m'
_RED = '\033[31m'
_YEL = '\033[33m'
_BLU = '\033[34m'
_CYN = '\033[36m'
_MGN = '\033[35m'
_GRY = '\033[90m'
_WHT = '\033[97m'

_ANSI_RE = re.compile(r'\x1b\[[0-9;]*m')

def _vlen(s: str) -> int:
    """Visible (printable) length, ignoring ANSI escape codes."""
    return len(_ANSI_RE.sub('', s))

def _pad(s: str, w: int) -> str:
    s = str(s)
    v = _vlen(s)
    if v >= w:
        # truncate on visible chars
        raw = _ANSI_RE.sub('', s)[:w]
        return raw
    return s + ' ' * (w - v)

def _c(text, *codes):
    return ''.join(codes) + text + _RST

# ── config ────────────────────────────────────────────────────────────────────
FC_PORT     = '/dev/ttyACM0'
FC_BAUD     = 57600
TAKEOFF_ALT = 1.0
CLIMB_RATE  = 0.05
TOLERANCE   = 0.3

# ── shared state ──────────────────────────────────────────────────────────────
_telem: dict = {
    'phase': 'init', 'armed': None, 'mode': None,
    'ned_x': None, 'ned_y': None, 'ned_z': None,
    'servo': None, 'ekf_flags': None,
    'takeoff_start_z': None, 'takeoff_target_z': None,
    'baro_alt': None, 'yaw_rad': None,
    'roll_rad': None, 'pitch_rad': None,
    'ts': 0.0,
}
_telem_lock   = threading.Lock()
_alt_history  = deque(maxlen=300)   # (ts, baro, ekf, uwb)
_flight_log:  list = []
_flight_log_lock   = threading.Lock()
_MAX_LOG      = 200
_display_active    = False   # suppresses print() in flog when dashboard is running


def flog(msg: str) -> None:
    ts   = time.strftime('%H:%M:%S')
    line = f"[{ts}] {msg}"
    if not _display_active:
        print(line, flush=True)
    with _flight_log_lock:
        _flight_log.append(line)
        if len(_flight_log) > _MAX_LOG:
            del _flight_log[0]


def _tset(**kw) -> None:
    with _telem_lock:
        _telem.update(kw)
        _telem['ts'] = time.time()


def _live(msg: str) -> None:
    """Single-line live status (overwritten). Suppressed when dashboard is active."""
    if not _display_active:
        print(msg, end='\r', flush=True)


# ── EKF bits ──────────────────────────────────────────────────────────────────
_EKF_ATT      = 0x001
_EKF_VEL_H    = 0x002
_EKF_VEL_V    = 0x004
_EKF_POS_REL  = 0x008
_EKF_POS_ABS  = 0x010
_EKF_VERT_ABS = 0x020
_EKF_AGL      = 0x040
_EKF_CONST    = 0x080   # const-position fallback: no horizontal position source
_EKF_NEED     = _EKF_ATT | _EKF_VEL_H | _EKF_POS_REL | _EKF_POS_ABS

_ekf_prev_flags = None   # change-detection for EKF log


def _ekf_diagnose(flags: int) -> str:
    """One-line reason string for an EKF state that is not fully ready."""
    reasons = []
    if flags & _EKF_CONST:
        reasons.append('const_pos_mode(수평위치 없음)')
    if not (flags & _EKF_ATT):
        reasons.append('no_att')
    if not (flags & _EKF_VEL_H):
        reasons.append('no_vel_h')
    if not (flags & _EKF_POS_REL):
        reasons.append('no_pos_rel')
    if not (flags & _EKF_POS_ABS):
        reasons.append('no_pos_abs')
    return ', '.join(reasons)

_POS_ONLY = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE  |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE  |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE  |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE  |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE  |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE  |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)

# ── flight primitives ─────────────────────────────────────────────────────────

def connect(port=FC_PORT, baud=FC_BAUD):
    for p in dict.fromkeys([port, '/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyACM2']):
        try:
            flog(f"Trying {p}...")
            conn = mavutil.mavlink_connection(p, baud=baud)
            conn.wait_heartbeat(timeout=3)
            flog(f"Connected on {p} (sysid={conn.target_system})")
            return conn
        except Exception:
            pass
    raise RuntimeError("FC not found on any ACM port")


def request_streams(conn):
    for s in [
        mavutil.mavlink.MAV_DATA_STREAM_EXTRA3,
        mavutil.mavlink.MAV_DATA_STREAM_POSITION,
        mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
        mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,
        mavutil.mavlink.MAV_DATA_STREAM_EXTRA2,
    ]:
        conn.mav.request_data_stream_send(
            conn.target_system, conn.target_component, s, 10, 1
        )
    flog("Streams requested")


def wait_ready(conn, timeout=15):
    global _ekf_prev_flags
    flog("Waiting for EKF...")
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = conn.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=2)
        if msg:
            bits = msg.flags
            _tset(ekf_flags=bits)
            if bits != _ekf_prev_flags:
                _ekf_prev_flags = bits
                ok = (bits & _EKF_NEED) == _EKF_NEED
                if ok:
                    flog(f"[EKF] READY ({bits:#06x})")
                else:
                    diag = _ekf_diagnose(bits)
                    flog(f"[EKF] NOT READY ({bits:#06x}) — {diag}")
            if (bits & _EKF_NEED) == _EKF_NEED:
                flog("EKF Ready")
                return
    flog("EKF Timeout — proceeding anyway")


def set_guided(conn, timeout=5):
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        4, 0, 0, 0, 0, 0
    )
    deadline = time.time() + timeout
    while time.time() < deadline:
        hb = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
        if hb and hb.custom_mode == 4:
            flog("GUIDED mode confirmed")
            _tset(mode='GUIDED')
            return True
    flog("GUIDED mode NOT confirmed — proceeding anyway")
    return False


def arm(conn, timeout=8):
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, 0, 0, 0, 0, 0, 0
    )
    flog("ARM command sent...")
    _tset(phase='arming', armed=False)
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = conn.recv_match(type=['HEARTBEAT', 'STATUSTEXT'], blocking=True, timeout=1.0)
        if msg is None:
            continue
        if msg.get_type() == 'STATUSTEXT':
            flog(f"FC: {msg.text.strip()}")
        if msg.get_type() == 'HEARTBEAT':
            if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                flog("Armed OK")
                _tset(armed=True, phase='armed')
                return True
    flog("ARM FAILED")
    _tset(phase='arm_failed')
    return False


def send_position(conn, x, y, z):
    conn.mav.set_position_target_local_ned_send(
        0, conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, _POS_ONLY,
        x, y, z, 0, 0, 0, 0, 0, 0, 0, 0
    )


def goto(conn, x, y, z, tolerance=TOLERANCE, timeout=30):
    flog(f"goto ({x:.1f}, {y:.1f}, alt={-z:.1f}m)")
    deadline = time.time() + timeout
    last_servo = 0.0
    while time.time() < deadline:
        send_position(conn, x, y, z)
        msg = conn.recv_match(
            type=['LOCAL_POSITION_NED', 'SERVO_OUTPUT_RAW'], blocking=False
        )
        if msg:
            if msg.get_type() == 'LOCAL_POSITION_NED':
                dist = ((msg.x-x)**2 + (msg.y-y)**2 + (msg.z-z)**2) ** 0.5
                _tset(ned_x=msg.x, ned_y=msg.y, ned_z=msg.z)
                _live(f"  pos=({msg.x:.2f},{msg.y:.2f},{msg.z:.2f}) dist={dist:.2f}m")
                if dist < tolerance:
                    flog(f"Reached ({x:.1f},{y:.1f},{-z:.1f}m)")
                    return True
            elif msg.get_type() == 'SERVO_OUTPUT_RAW':
                now2 = time.time()
                if now2 - last_servo > 3.0:
                    flog(f"SERVO: {msg.servo1_raw} {msg.servo2_raw}"
                         f" {msg.servo3_raw} {msg.servo4_raw}")
                    last_servo = now2
        time.sleep(0.05)
    flog("goto: Timeout")
    return False


def hold(conn, x, y, z, duration, uwb=None):
    deadline = time.time() + duration
    while time.time() < deadline:
        send_position(conn, x, y, z)
        att = conn.recv_match(type='ATTITUDE', blocking=False)
        if att and uwb is not None:
            uwb.set_yaw(att.yaw)
        if uwb is not None:
            tags = uwb.get_tags()
            stale = tags and all(time.time() - v['ts'] > 2.0 for v in tags.values())
            if stale or not tags:
                flog("[SAFETY] UWB signal lost during hold — landing")
                land(conn)
                return False
        time.sleep(0.1)
    return True


def takeoff(conn, alt=TAKEOFF_ALT):
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
        0, 0, 0, float('nan'), 0, 0, alt,
    )
    flog(f"TAKEOFF cmd sent, target={alt}m")
    ack = conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    if ack:
        flog(f"TAKEOFF ACK: {'OK' if ack.result==0 else 'FAIL'} (result={ack.result})")
        if ack.result != 0:
            flog("TAKEOFF rejected by FC — aborting")
            return None
    else:
        flog("TAKEOFF: no ACK")

    _tset(phase='takeoff')
    start_z = None
    last_log = last_servo = 0.0
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
                flog("TAKEOFF ABORTED: disarmed mid-flight!")
                _tset(phase='disarmed')
                return None
        elif t == 'SERVO_OUTPUT_RAW':
            _tset(servo=(msg.servo1_raw, msg.servo2_raw,
                         msg.servo3_raw, msg.servo4_raw))
            now2 = time.time()
            if now2 - last_servo > 2.0:
                flog(f"SERVO: {msg.servo1_raw} {msg.servo2_raw}"
                     f" {msg.servo3_raw} {msg.servo4_raw}"
                     f" {'(motors off!)' if msg.servo1_raw <= 1000 else ''}")
                last_servo = now2
        elif t == 'LOCAL_POSITION_NED':
            _tset(ned_x=msg.x, ned_y=msg.y, ned_z=msg.z)
            if start_z is None:
                start_z = msg.z
                threshold = start_z - (alt - TOLERANCE)
                _tset(takeoff_start_z=start_z, takeoff_target_z=threshold)
                flog(f"TAKEOFF: start_z={start_z:.2f}m  need z<{threshold:.2f}m")
                if start_z < -(alt - 0.2):
                    flog(f"  WARNING: start_z={start_z:.2f}m already near target — baro drift!")
            threshold = start_z - (alt - TOLERANCE)
            now2 = time.time()
            if now2 - last_log > 1.0:
                delta = start_z - msg.z
                flog(f"  climbing: z={msg.z:.2f}m  delta={delta:.2f}/{alt:.1f}m")
                last_log = now2
            _live(f"  z={msg.z:.2f}m")
            if msg.z < threshold:
                delta = start_z - msg.z
                flog(f"Takeoff complete: z={msg.z:.2f}m  delta={delta:.2f}m")
                return msg.z
    flog("TAKEOFF TIMEOUT")
    return None


def land(conn):
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND, 0,
        0, 0, 0, 0, 0, 0, 0
    )
    flog("Landing command sent")
    _tset(phase='landing')


def _emergency_disarm(conn):
    flog("[EMERGENCY] Sending force disarm...")
    try:
        conn.mav.command_long_send(
            conn.target_system, conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            0, 21196, 0, 0, 0, 0, 0
        )
    except Exception as e:
        flog(f"[EMERGENCY] Disarm failed: {e}")


def debug_status(conn, uwb, duration=15):
    flog(f"DEBUG MODE — watching for {duration}s")
    deadline = time.time() + duration
    while time.time() < deadline:
        ekf       = conn.recv_match(type='EKF_STATUS_REPORT', blocking=False)
        pos       = conn.recv_match(type='LOCAL_POSITION_NED', blocking=False)
        drone_pos = uwb.get_drone_pos()
        tags      = uwb.get_tags()
        vision_n  = getattr(uwb, '_dbg_cnt', 0)
        uwb_ok    = drone_pos is not None

        flog(f"  UWB:{'OK' if uwb_ok else 'NO DATA':7s}  tags={len(tags)}"
             f"  vision_sent={vision_n:4d}")
        if drone_pos:
            flog(f"  UWB pos: ({drone_pos[0]:.2f}, {drone_pos[1]:.2f}, {drone_pos[2]:.2f})")
        if ekf:
            b = ekf.flags
            flog(f"  EKF={b:#06x} att={bool(b&_EKF_ATT)} vel={bool(b&_EKF_VEL_H)}"
                 f" pos_rel={bool(b&_EKF_POS_REL)} pos_abs={bool(b&_EKF_POS_ABS)}")
        if pos:
            flog(f"  NED: x={pos.x:.2f} y={pos.y:.2f} z={pos.z:.2f}")
        if pos and drone_pos:
            dx = drone_pos[0] - pos.x
            dy = drone_pos[1] - pos.y
            dz = drone_pos[2] - pos.z
            flog(f"  DRIFT dx={dx:+.3f} dy={dy:+.3f} dz={dz:+.3f}")
        time.sleep(1.0)


# ── heartbeat + telem collection ──────────────────────────────────────────────

_vision_log_ts = 0.0   # last time we logged VISION content


def _heartbeat_loop(conn, stop_evt, uwb):
    global _vision_log_ts
    while not stop_evt.is_set():
        try:
            conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0
            )
        except Exception:
            return
        for _ in range(20):
            msg = conn.recv_match(
                type=['VFR_HUD', 'ATTITUDE', 'LOCAL_POSITION_NED',
                      'EKF_STATUS_REPORT', 'SERVO_OUTPUT_RAW', 'STATUSTEXT'],
                blocking=False,
            )
            if msg is None:
                break
            t = msg.get_type()
            if t == 'VFR_HUD':
                with _telem_lock:
                    _telem['baro_alt'] = msg.alt
                    ned_z = _telem.get('ned_z')
                ekf_alt   = -ned_z if ned_z is not None else None
                drone_pos = uwb.get_drone_pos()
                uwb_h     = -drone_pos[2] if drone_pos else None
                _alt_history.append((time.time(), msg.alt, ekf_alt, uwb_h))
            elif t == 'ATTITUDE':
                with _telem_lock:
                    _telem['yaw_rad']   = msg.yaw
                    _telem['roll_rad']  = msg.roll
                    _telem['pitch_rad'] = msg.pitch
                uwb.set_yaw(msg.yaw)
            elif t == 'LOCAL_POSITION_NED':
                _tset(ned_x=msg.x, ned_y=msg.y, ned_z=msg.z)
            elif t == 'EKF_STATUS_REPORT':
                global _ekf_prev_flags
                new_flags = msg.flags
                _tset(ekf_flags=new_flags)
                if new_flags != _ekf_prev_flags:
                    old_flags = _ekf_prev_flags
                    _ekf_prev_flags = new_flags
                    pr_new = bool(new_flags & _EKF_POS_REL)
                    pa_new = bool(new_flags & _EKF_POS_ABS)
                    ok_new = (new_flags & _EKF_NEED) == _EKF_NEED
                    ok_old = (old_flags & _EKF_NEED) == _EKF_NEED if old_flags is not None else None
                    if not ok_new:
                        diag = _ekf_diagnose(new_flags)
                        flog(f"[EKF!!] {new_flags:#06x} pos_rel={'Y' if pr_new else 'N'}"
                             f" pos_abs={'Y' if pa_new else 'N'} — {diag}")
                    elif ok_new and ok_old is False:
                        flog(f"[EKF] 복구됨 — pos_rel=Y pos_abs=Y ({new_flags:#06x})")
            elif t == 'SERVO_OUTPUT_RAW':
                _tset(servo=(msg.servo1_raw, msg.servo2_raw,
                             msg.servo3_raw, msg.servo4_raw))
            elif t == 'STATUSTEXT':
                text = msg.text.strip()
                # EKF / navigation 관련 메시지만 필터링
                tl = text.lower()
                if any(k in tl for k in ('ekf', 'nav', 'vision', 'extern', 'gps',
                                          'yaw', 'compass', 'pos', 'aiding',
                                          'origin', 'field elev')):
                    sev = getattr(msg, 'severity', 6)
                    tag = '[FC!!]' if sev <= 3 else '[FC]'
                    flog(f"{tag} {text}")

        # VISION 전송 내용 5초마다 로그
        now = time.time()
        if now - _vision_log_ts >= 5.0:
            _vision_log_ts = now
            drone_pos = uwb.get_drone_pos()
            with _telem_lock:
                yaw_r = _telem.get('yaw_rad')
            vis_n = getattr(uwb, '_dbg_cnt', 0)
            if drone_pos:
                x, y, _ = drone_pos
                yaw_d = math.degrees(yaw_r) if yaw_r is not None else float('nan')
                flog(f"[VISION] x={x:+.3f} y={y:+.3f} yaw={yaw_d:+.1f}° vis#{vis_n}")
            else:
                flog(f"[VISION] NO UWB DATA  vis#{vis_n}")

        time.sleep(1.0)


# ── CLI dashboard ─────────────────────────────────────────────────────────────

_TW = 92   # total terminal width
_IW = _TW - 2   # inner width (between outer │ chars)

# 3 columns: C1 | C2 | C3  (widths sum to _IW - 2 separators)
_C1, _C2, _C3 = 26, 29, _IW - 26 - 29 - 2


def _row(c1, c2, c3):
    return '│' + _pad(c1, _C1) + '│' + _pad(c2, _C2) + '│' + _pad(c3, _C3) + '│'


def _hbar(pct, w=10):
    n = int(pct * w)
    return '#' * n + '-' * (w - n)


_PHASE_CLR = {
    'INIT': _GRY, 'ARMING': _YEL, 'ARMED': _YEL,
    'TAKEOFF': _BLU, 'HOVER': _GRN, 'MISSION': _MGN,
    'LANDING': _YEL, 'DONE': _GRN,
    'ARM_FAILED': _RED, 'DISARMED': _RED,
}

_LOG_CLR = {
    ('fail', 'abort', 'emergency', 'error', '[ekf!!]'):     _RED,
    ('safety', 'lost', 'timeout', 'warn', 'not ready'):     _YEL,
    ('armed ok', 'takeoff complete', 'done', '[ekf] ready', '[ekf] 복구'): _GRN,
    ('arm', 'takeoff', 'climbing', 'landing', 'goto', 'reached'): _BLU,
}


def _log_color(line):
    ll = line.lower()
    for keys, col in _LOG_CLR.items():
        if any(k in ll for k in keys):
            return col
    return ''


def _render(uwb, title):
    with _telem_lock:
        t = dict(_telem)
    armed   = t.get('armed')
    phase   = t.get('phase', 'init').upper()
    mode    = t.get('mode') or '?'
    ekf     = t.get('ekf_flags')
    ned_x   = t.get('ned_x')
    ned_y   = t.get('ned_y')
    ned_z   = t.get('ned_z')
    baro    = t.get('baro_alt')
    yaw     = t.get('yaw_rad')
    roll    = t.get('roll_rad')
    pitch   = t.get('pitch_rad')
    servo   = t.get('servo')
    start_z = t.get('takeoff_start_z')

    drone_pos = uwb.get_drone_pos()
    tags      = uwb.get_tags()
    vision_n  = getattr(uwb, '_dbg_cnt', 0)

    with _flight_log_lock:
        logs = list(_flight_log)

    now_str = time.strftime('%H:%M:%S')
    out = []

    # ── header ────────────────────────────────────────────────────────────────
    out.append('┌' + '─' * _IW + '┐')
    hdr_right = f'[{now_str}]  '
    hdr_left  = f'  {title}'
    hdr_mid   = ' ' * (_IW - len(hdr_left) - len(hdr_right))
    out.append('│' + _c(hdr_left, _BLD) + hdr_mid + _c(hdr_right, _CYN) + '│')

    # ── column header ─────────────────────────────────────────────────────────
    out.append('├' + '─'*_C1 + '┬' + '─'*_C2 + '┬' + '─'*_C3 + '┤')
    out.append(_row(
        _c(' STATUS', _BLD),
        _c(' MOTORS / ATTITUDE', _BLD),
        _c(' UWB', _BLD),
    ))
    out.append('├' + '─'*_C1 + '┼' + '─'*_C2 + '┼' + '─'*_C3 + '┤')

    # ── STATUS column ─────────────────────────────────────────────────────────
    if armed:
        arm_s = _c(' [ ARMED ]  ', _GRN + _BLD)
    elif armed is False:
        arm_s = _c(' [ DISARMED ]', _RED + _BLD)
    else:
        arm_s = _c(' [ UNKNOWN ]', _GRY)

    pc = _PHASE_CLR.get(phase, '')
    ekf_ok = ekf is not None and (ekf & _EKF_NEED) == _EKF_NEED
    ekf_s  = _c(' EKF: [OK] ', _GRN) if ekf_ok else _c(' EKF: [!!] ', _RED + _BLD)

    c1 = [
        arm_s,
        _c(f' PHASE : {phase}', pc),
        f' MODE  : {mode}',
        ekf_s,
    ]
    if ekf is not None:
        Y, N = _c('Y', _GRN), _c('N', _RED)
        c1.append(f'  att={Y if ekf&_EKF_ATT else N}'
                  f' vel={Y if ekf&_EKF_VEL_H else N}'
                  f' pos_rel={Y if ekf&_EKF_POS_REL else N}')
        c1.append(f'  pos_abs={Y if ekf&_EKF_POS_ABS else N}')
        if ekf & _EKF_CONST:
            c1.append(_c('  [const_pos: 위치입력 없음]', _RED))
        else:
            c1.append('')
    else:
        c1 += [' (no EKF data)', '', '']
    c1.append('')
    c1.append(f" EKF alt : {f'{-ned_z:+.3f}m' if ned_z is not None else 'N/A'}")
    c1.append(f" Baro    : {f'{baro:+.3f}m' if baro is not None else 'N/A'}")
    if drone_pos:
        c1.append(f" UWB h   : {-drone_pos[2]:+.3f}m")
    else:
        c1.append(' UWB h   : N/A')
    c1.append('')
    if start_z is not None and ned_z is not None:
        delta = start_z - ned_z
        pct   = max(0.0, min(delta / TAKEOFF_ALT, 1.0))
        c1.append(f" Climb [{_hbar(pct, 12)}]")
        c1.append(f"       {delta:.2f}/{TAKEOFF_ALT:.1f}m")
    else:
        c1 += ['', '']

    # ── MOTORS / ATTITUDE column ───────────────────────────────────────────────
    c2 = [' Motors:']
    if servo is not None:
        for i, v in enumerate(servo):
            pct = max(0.0, min((v - 1000) / 1000, 1.0))
            on  = v > 1050
            bar = _hbar(pct, 8)
            on_s = _c('ON', _GRN) if on else _c('off', _GRY)
            c2.append(f"  M{i+1}: {v:4d} [{bar}] {on_s}")
    else:
        c2.append('  waiting...')
        c2 += [''] * 3
    c2.append('')
    if roll is not None:
        c2.append(' Attitude:')
        c2.append(f"  Roll : {math.degrees(roll):+6.1f} deg")
        c2.append(f"  Pitch: {math.degrees(pitch):+6.1f} deg")
        c2.append(f"  Yaw  : {math.degrees(yaw):+6.1f} deg")
    else:
        c2.append(' Attitude: waiting...')
        c2 += [''] * 3
    c2.append('')
    if drone_pos and ned_z is not None:
        dx = drone_pos[0] - (ned_x or 0.0)
        dy = drone_pos[1] - (ned_y or 0.0)
        dz = drone_pos[2] - ned_z
        c2.append(' EKF-UWB delta:')
        c2.append(f"  dx={dx:+.3f}  dy={dy:+.3f}")
        c2.append(f"  dz={dz:+.3f}")
    else:
        c2.append(' EKF-UWB delta: N/A')
        c2 += ['', '']

    # ── UWB column ────────────────────────────────────────────────────────────
    c3 = []
    if drone_pos:
        rx, ry, rz = drone_pos
        c3.append(f" pos: ({rx:+.2f}, {ry:+.2f})")
        c3.append(f" h  : {_c(f'{-rz:+.3f}m', _CYN)}")
    else:
        c3 += [_c(' pos: NO DATA', _YEL), '']
    if tags:
        info = next(iter(tags.values()))
        c3.append(f" QF : {info.get('qf', '?')}")
    else:
        c3.append(' QF : --')
    c3.append(f" vis: {vision_n}")
    c3.append('')
    if tags:
        info = next(iter(tags.values()))
        fw = info.get('fw_pos')
        if fw:
            c3.append(f" fw_z: {fw[2]:+.3f}m")
        else:
            c3.append('')
        c3.append(' Anchors:')
        for aid, av in sorted(info.get('anchors', {}).items()):
            ax, ay, az = av['pos']
            c3.append(f"  {aid}: {av['dist_m']:.3f}m"
                      f" ({ax:.2f},{ay:.2f},{az:.2f})")
    else:
        c3.append(' NO TAGS')
        c3 += [''] * 6

    # ── pad columns to equal height ───────────────────────────────────────────
    n = max(len(c1), len(c2), len(c3))
    c1 += [''] * (n - len(c1))
    c2 += [''] * (n - len(c2))
    c3 += [''] * (n - len(c3))
    for r1, r2, r3 in zip(c1, c2, c3):
        out.append(_row(r1, r2, r3))

    # ── flight log ────────────────────────────────────────────────────────────
    LOG_ROWS = 8
    out.append('├' + '─'*_C1 + '┴' + '─'*_C2 + '┴' + '─'*_C3 + '┤')
    out.append('│' + _pad(_c(' FLIGHT LOG', _BLD), _IW) + '│')
    out.append('│' + '─' * _IW + '│')

    display = logs[-LOG_ROWS:]
    for entry in display:
        col = _log_color(entry)
        entry_disp = entry[:_IW - 2]
        out.append('│ ' + _pad(_c(entry_disp, col) if col else entry_disp, _IW - 2) + '│')
    for _ in range(LOG_ROWS - len(display)):
        out.append('│' + ' ' * _IW + '│')

    out.append('└' + '─' * _IW + '┘')
    return '\n'.join(out)


def _display_loop(uwb, title, stop_evt):
    global _display_active
    _display_active = True
    sys.stdout.write('\033[?25l')   # hide cursor
    sys.stdout.flush()
    try:
        while not stop_evt.is_set():
            frame = _render(uwb, title)
            sys.stdout.write('\033[2J\033[H')
            sys.stdout.write(frame + '\n')
            sys.stdout.flush()
            time.sleep(0.3)
    finally:
        _display_active = False
        sys.stdout.write('\033[?25h')   # restore cursor
        sys.stdout.flush()


# ── monitor mode (--monitor flag, no arming) ──────────────────────────────────

def monitor_mode(conn, uwb):
    print("=" * 60)
    print("  MONITOR MODE  (Ctrl-C to exit, no arming)")
    print("=" * 60)

    _ekf = _pos = _baro = _att = None
    while True:
        for _ in range(50):
            msg = conn.recv_match(
                type=['EKF_STATUS_REPORT', 'LOCAL_POSITION_NED', 'VFR_HUD', 'ATTITUDE'],
                blocking=False,
            )
            if msg is None:
                break
            t = msg.get_type()
            if   t == 'EKF_STATUS_REPORT':  _ekf  = msg
            elif t == 'LOCAL_POSITION_NED':  _pos  = msg
            elif t == 'VFR_HUD':            _baro = msg
            elif t == 'ATTITUDE':           _att  = msg

        drone_pos = uwb.get_drone_pos()
        tags      = uwb.get_tags()
        vision_n  = getattr(uwb, '_dbg_cnt', 0)
        print(f"\n[{time.strftime('%H:%M:%S')}]")
        print("  [ UWB ]")
        if drone_pos:
            rx, ry, rz = drone_pos
            print(f"    trilat h : {-rz:+.3f} m")
            print(f"    rel x,y  : ({rx:+.3f}, {ry:+.3f}) m")
            print(f"    vis sent : {vision_n}")
        else:
            print(f"    NO DATA  (vis sent: {vision_n})")
        if tags:
            info = next(iter(tags.values()))
            fw = info.get('fw_pos')
            if fw:
                print(f"    fw_pos z : {fw[2]:+.3f} m  (qf={fw[3]})")
            for aid, v in sorted(info.get('anchors', {}).items()):
                ax, ay, az = v['pos']
                print(f"    {aid}: dist={v['dist_m']:.3f}m  pos=({ax:.2f},{ay:.2f},{az:.2f})")
        print("  [ BARO / EKF ]")
        if _baro:
            print(f"    baro alt : {_baro.alt:+.3f} m  (climb={_baro.climb:+.2f} m/s)")
        else:
            print("    baro     : no data")
        if _pos:
            print(f"    EKF alt  : {-_pos.z:+.3f} m  (NED z={_pos.z:+.3f})")
            print(f"    EKF x,y  : ({_pos.x:+.3f}, {_pos.y:+.3f}) m")
        else:
            print("    EKF pos  : no data")
        if _ekf:
            b  = _ekf.flags
            ok = (b & _EKF_NEED) == _EKF_NEED
            print(f"    EKF status: {'READY' if ok else 'NOT READY'}  ({b:#06x})")
        if _att:
            print(f"    attitude : roll={math.degrees(_att.roll):+.1f}  "
                  f"pitch={math.degrees(_att.pitch):+.1f}  "
                  f"yaw={math.degrees(_att.yaw):+.1f}")
        if drone_pos and _pos:
            uwb_h  = -drone_pos[2]
            ekf_h  = -_pos.z
            baro_h = _baro.alt if _baro else None
            print("  [ COMPARE ]")
            print(f"    UWB={uwb_h:+.3f}  EKF={ekf_h:+.3f}"
                  + (f"  Baro={baro_h:+.3f}" if baro_h is not None else ""))
            print(f"    UWB-EKF dz={uwb_h - ekf_h:+.4f} m")
        time.sleep(1.0)


# ── main entry point ──────────────────────────────────────────────────────────

def run(flight_fn, title='FLIGHT'):
    """Connect, start UWB+heartbeat+dashboard, run flight_fn, clean up."""
    monitor = '--monitor' in sys.argv

    conn = connect()
    uwb  = UWBTag(conn)
    uwb.start()
    request_streams(conn)

    stop_hb   = threading.Event()
    stop_disp = threading.Event()

    hb = threading.Thread(target=_heartbeat_loop, args=(conn, stop_hb, uwb), daemon=True)
    hb.start()

    monitor_title = title + '  [MONITOR]' if monitor else title
    disp = threading.Thread(
        target=_display_loop, args=(uwb, monitor_title, stop_disp), daemon=True)
    disp.start()

    if monitor:
        flog("Monitor mode — no arming. Ctrl-C to exit.")
        try:
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            flog("Monitor stopped.")
        finally:
            stop_disp.set()
            time.sleep(0.4)
            stop_hb.set()
            uwb.stop()
            sys.stdout.write('\033[?25h\n')
            sys.stdout.flush()
        return

    ft = threading.Thread(target=flight_fn, args=(conn, uwb), daemon=True)
    ft.start()
    try:
        ft.join()
    except KeyboardInterrupt:
        flog("[USER] Ctrl-C — emergency disarm")
        _emergency_disarm(conn)
    finally:
        stop_disp.set()
        time.sleep(0.4)
        stop_hb.set()
        uwb.stop()
        sys.stdout.write('\033[?25h\n')
        sys.stdout.flush()
        # print final log to stdout
        print("\n--- Session log ---")
        with _flight_log_lock:
            for line in _flight_log:
                print(line)
