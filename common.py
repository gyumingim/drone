"""common.py — 공유 상수, 헬퍼, 쓰레드, 비행 시퀀스"""
import time, threading
from pymavlink import mavutil

FC_PORT   = '/dev/ttyACM0'
FC_BAUD   = 57600
TAKEOFF_M = 1.0
HOVER_S   = 5.0

_EKF_NEED = 0x001 | 0x002 | 0x008 | 0x010
_ARMED    = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED

_MAV_RESULT = {
    0: 'ACCEPTED', 1: 'TEMP_REJECTED', 2: 'DENIED',
    3: 'UNSUPPORTED', 4: 'FAILED', 5: 'IN_PROGRESS', 6: 'CANCELLED',
}
_SEV = ['EMERG', 'ALERT', 'CRIT', 'ERR', 'WARN', 'NOTICE', 'INFO', 'DEBUG']
_EKF_BITS = {
    0x001: 'att',        0x002: 'vel_h',      0x004: 'vel_v',
    0x008: 'pos_rel',    0x010: 'pos_abs',    0x020: 'const_pos',
    0x040: 'pred_h',     0x080: 'pred_v',     0x100: 'pred_rel',
    0x200: 'gps_glitch', 0x400: 'accel_err',
}

# SET_POSITION_TARGET_LOCAL_NED: 위치만 사용 (속도·가속도·yaw 무시)
_POS_MASK = 0b0000111111111000


def ts():
    return time.strftime('%H:%M:%S')


def ekf_str(flags):
    return '|'.join(n for b, n in _EKF_BITS.items() if flags & b) or 'none'


def cmd(c, command, *p):
    c.mav.command_long_send(c.target_system, c.target_component, command, 0, *p)


def go_to(c, x_north, y_east, z_ned):
    """NED 절대 좌표 위치 목표 설정 (z_ned 음수=위쪽)."""
    c.mav.set_position_target_local_ned_send(
        0, c.target_system, c.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        _POS_MASK,
        x_north, y_east, z_ned,
        0, 0, 0, 0, 0, 0, 0, 0)


def wait_pos(c, x, y, tol=0.3, timeout=20):
    """XY 도달 대기. 도달 시 True, 타임아웃 시 False."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        m = c.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
        if m is None:
            continue
        dist = ((m.x - x)**2 + (m.y - y)**2) ** 0.5
        print(f'[NAV] {ts()} pos=({m.x:.2f},{m.y:.2f}) '
              f'target=({x:.2f},{y:.2f}) dist={dist:.2f}m')
        if dist < tol:
            return True
    return False


# ── 백그라운드 쓰레드 ─────────────────────────────────────────────────────────

def _vision_loop(c, uwb, stop):
    cov = [0.0] * 21
    cov[0] = cov[6] = 0.01
    cov[11] = 9999.0
    yaw, sent = 0.0, 0
    while not stop.is_set():
        att = c.recv_match(type='ATTITUDE', blocking=False)
        if att:
            yaw = att.yaw
        xy = uwb.get_xy()
        if xy:
            c.mav.vision_position_estimate_send(
                int(time.time() * 1e6), xy[0], xy[1], 0.0, 0, 0, yaw, cov, 0)
            sent += 1
            if sent % 20 == 0:
                print(f'[VIS] {ts()} xy=({xy[0]:.3f},{xy[1]:.3f}) '
                      f'yaw={yaw:.3f}rad total={sent}')
        time.sleep(0.05)


def _hb_loop(c, stop):
    while not stop.is_set():
        c.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        time.sleep(1)


def _rc_override_loop(c, stop):
    """5Hz RC override — ch3만 1000으로 throttle failsafe 방지.
    나머지 채널은 0(release) → 물리적 RC 조종기 입력 통과.
    ref: mavlink.io RC_CHANNELS_OVERRIDE, value=0 means release to RC radio."""
    while not stop.is_set():
        try:
            c.mav.rc_channels_override_send(
                c.target_system, c.target_component,
                0, 0, 1000, 0,   # ch1/2/4: release, ch3: 1000 (failsafe 방지)
                0, 0, 0, 0)      # ch5-8: release → RC 스위치 사용 가능
        except Exception as e:
            print(f'[RC-OVR] {ts()} 오류: {e}')
        time.sleep(0.2)


def _status_loop(c, stop):
    while not stop.is_set():
        try:
            m = c.recv_match(type='STATUSTEXT', blocking=False)
            if m:
                sev = _SEV[m.severity] if m.severity < len(_SEV) else str(m.severity)
                print(f'[FC-MSG] {ts()} [{sev}] {m.text}')
        except Exception as e:
            print(f'[FC-MSG] {ts()} 오류: {e}')
        time.sleep(0.05)


# ── 공통 비행 시퀀스 ──────────────────────────────────────────────────────────

def connect(uwb):
    """FC 연결, EKF 준비, GUIDED 설정, ARM, 쓰레드 시작.
    Returns (mavlink_connection, stop_event)."""
    print(f'[FC] {ts()} {FC_PORT}@{FC_BAUD} 연결 중...')
    c = mavutil.mavlink_connection(FC_PORT, baud=FC_BAUD)
    hb0 = c.wait_heartbeat()
    print(f'[FC] {ts()} sysid={c.target_system} '
          f'base_mode={hb0.base_mode:#010b} custom_mode={hb0.custom_mode}')

    c.mav.request_data_stream_send(
        c.target_system, c.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
    c.mav.set_gps_global_origin_send(c.target_system, 0, 0, 0)

    stop = threading.Event()
    threading.Thread(target=_vision_loop,      args=(c, uwb, stop), daemon=True).start()
    threading.Thread(target=_hb_loop,          args=(c, stop),      daemon=True).start()
    threading.Thread(target=_rc_override_loop, args=(c, stop),      daemon=True).start()
    threading.Thread(target=_status_loop,      args=(c, stop),      daemon=True).start()
    print(f'[THREAD] {ts()} 쓰레드 시작')
    time.sleep(1)

    print(f'[EKF] {ts()} 준비 대기...')
    deadline = time.time() + 20
    last_flags = None
    while time.time() < deadline:
        m = c.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=1)
        if m is None:
            continue
        if m.flags != last_flags:
            print(f'[EKF] {ts()} flags={m.flags:#06x} ({ekf_str(m.flags)})')
            last_flags = m.flags
        if (m.flags & _EKF_NEED) == _EKF_NEED:
            print(f'[EKF] {ts()} 준비 완료!')
            break
    else:
        print(f'[EKF] {ts()} 타임아웃 — 강행')

    cmd(c, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4, 0, 0, 0, 0, 0)
    ack = c.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f'[MODE] {ts()} GUIDED ACK: {_MAV_RESULT.get(getattr(ack,"result",-1),"?")}')

    cmd(c, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1, 0, 0, 0, 0, 0, 0)
    ack = c.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f'[ARM] {ts()} ARM ACK: {_MAV_RESULT.get(getattr(ack,"result",-1),"?")}')

    t0 = time.time()
    while True:
        hb = c.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if hb and hb.base_mode & _ARMED:
            print(f'[ARM] {ts()} ARMED! ({time.time()-t0:.1f}s)')
            break
    time.sleep(0.5)
    return c, stop


def do_takeoff(c, stop, takeoff_m=TAKEOFF_M):
    """TAKEOFF 명령 + 목표 고도 도달 + 속도 안정 대기. 성공 True, 실패 False."""
    cmd(c, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, float('nan'), 0, 0, takeoff_m)
    ack = c.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    result = getattr(ack, 'result', -1)
    print(f'[TKOF] {ts()} TAKEOFF ACK: {_MAV_RESULT.get(result,"?")}')
    if result != 0:
        return False

    # 목표 고도 95% 도달 대기 (DroneKit 표준 패턴)
    # ref: github.com/dronekit/dronekit-python docs/guide/taking_off.rst
    target_z = -(takeoff_m * 0.95)
    deadline = time.time() + 20
    while time.time() < deadline:
        m = c.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
        if m is None:
            continue
        print(f'[TKOF] {ts()} z={m.z:.3f} vz={m.vz:.3f} (목표 z<{target_z:.2f})')
        if m.z < target_z:
            print(f'[TKOF] {ts()} 고도 도달!')
            return True
    print(f'[TKOF] {ts()} 타임아웃')
    return False


def do_land(c, stop):
    """착륙 명령 전송 + 쓰레드 정지."""
    cmd(c, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0)
    ack = c.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f'[LAND] {ts()} LAND ACK: {_MAV_RESULT.get(getattr(ack,"result",-1),"?")}')
    stop.set()
    print(f'[DONE] {ts()} 완료')
