"""
flight.py — UWB XY + Barometer Z indoor hover  [DEBUG]
XY: VISION_POSITION_ESTIMATE (Z 분산=9999 → EKF가 바로미터로 Z 처리)
Z:  EKF3 Barometer (EK3_SRC1_POSZ=1)
"""
import time, threading
from pymavlink import mavutil
from uwb_reader import UWBReader

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
    0x001: 'att',       0x002: 'vel_h',     0x004: 'vel_v',
    0x008: 'pos_rel',   0x010: 'pos_abs',   0x020: 'const_pos',
    0x040: 'pred_h',    0x080: 'pred_v',    0x100: 'pred_rel',
    0x200: 'gps_glitch',0x400: 'accel_err',
}


def _ts():
    return time.strftime('%H:%M:%S')


def _ekf_str(flags):
    return '|'.join(n for b, n in _EKF_BITS.items() if flags & b) or 'none'


def _cmd(c, cmd, *p):
    c.mav.command_long_send(c.target_system, c.target_component, cmd, 0, *p)


def _vision_loop(c, uwb, stop):
    cov = [0.0] * 21
    cov[0] = cov[6] = 0.01
    cov[11] = 9999.0
    sent = 0
    while not stop.is_set():
        xy = uwb.get_xy()
        if xy:
            c.mav.vision_position_estimate_send(
                int(time.time() * 1e6), xy[0], xy[1], 0.0, 0, 0, 0.0, cov, 0)
            sent += 1
            if sent % 20 == 0:
                print(f'[VIS] {_ts()} xy=({xy[0]:.3f},{xy[1]:.3f}) yaw=0.0 total={sent}')
        time.sleep(0.05)


def _hb_loop(c, stop):
    while not stop.is_set():
        c.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        time.sleep(1)


def _rc_override_loop(c, stop):
    """5Hz RC override — RC 수신기 없을 때 failsafe 방지.
    ch3(스로틀)=1000 (FS_THR_VALUE=975 초과), ch8=1500 (LAND 모드 미발동)."""
    while not stop.is_set():
        try:
            c.mav.rc_channels_override_send(
                c.target_system, c.target_component,
                1500, 1500, 1000, 1500,   # ch1-4: roll/pitch/thr/yaw
                1500, 1500, 1500, 1500)   # ch5-8: aux 전부 mid
        except Exception as e:
            print(f'[RC-OVR] {_ts()} 오류: {e}')
        time.sleep(0.2)


def _status_loop(c, stop):
    while not stop.is_set():
        try:
            m = c.recv_match(type='STATUSTEXT', blocking=False)
            if m:
                sev = _SEV[m.severity] if m.severity < len(_SEV) else str(m.severity)
                print(f'[FC-MSG] {_ts()} [{sev}] {m.text}')
        except Exception as e:
            print(f'[FC-MSG] {_ts()} 오류: {e}')
        time.sleep(0.05)


def main():
    print(f'[BOOT] {_ts()} flight.py 시작')

    # ── UWB ──────────────────────────────────────────────────────────────
    uwb = UWBReader()
    uwb.start()
    print(f'[UWB] {_ts()} origin 대기...')
    while uwb.get_xy() is None:
        time.sleep(0.2)
    print(f'[UWB] {_ts()} origin 확정: {uwb.get_xy()}')

    # ── FC 연결 ───────────────────────────────────────────────────────────
    print(f'[FC] {_ts()} {FC_PORT}@{FC_BAUD} 연결 중...')
    c = mavutil.mavlink_connection(FC_PORT, baud=FC_BAUD)
    hb0 = c.wait_heartbeat()
    print(f'[FC] {_ts()} 연결 sysid={c.target_system} compid={c.target_component}')
    print(f'[FC] {_ts()} heartbeat: autopilot={hb0.autopilot} type={hb0.type} '
          f'base_mode={hb0.base_mode:#010b} custom_mode={hb0.custom_mode} '
          f'system_status={hb0.system_status}')

    print(f'[FC] {_ts()} 데이터 스트림 요청 (10Hz ALL)')
    c.mav.request_data_stream_send(
        c.target_system, c.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)

    print(f'[FC] {_ts()} GPS 원점 설정 (lat=0 lon=0 alt=0)')
    c.mav.set_gps_global_origin_send(c.target_system, 0, 0, 0)

    # ── 백그라운드 쓰레드 ────────────────────────────────────────────────
    stop = threading.Event()
    threading.Thread(target=_vision_loop,     args=(c, uwb, stop), daemon=True).start()
    threading.Thread(target=_hb_loop,         args=(c, stop),      daemon=True).start()
    threading.Thread(target=_rc_override_loop, args=(c, stop),     daemon=True).start()
    threading.Thread(target=_status_loop,     args=(c, stop),      daemon=True).start()
    print(f'[THREAD] {_ts()} vision / heartbeat / rc_override / statustext 쓰레드 시작')
    time.sleep(1)

    # ── EKF 준비 대기 ────────────────────────────────────────────────────
    print(f'[EKF] {_ts()} 준비 대기 (필요={_ekf_str(_EKF_NEED)})')
    deadline = time.time() + 20
    last_flags = None
    while time.time() < deadline:
        m = c.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=1)
        if m is None:
            print(f'[EKF] {_ts()} 수신 없음')
            continue
        if m.flags != last_flags:
            print(f'[EKF] {_ts()} flags={m.flags:#06x} ({_ekf_str(m.flags)}) '
                  f'vel_var={m.velocity_variance:.3f} '
                  f'pos_h_var={m.pos_horiz_variance:.3f} '
                  f'pos_v_var={m.pos_vert_variance:.3f} '
                  f'compass_var={m.compass_variance:.3f}')
            last_flags = m.flags
        if (m.flags & _EKF_NEED) == _EKF_NEED:
            print(f'[EKF] {_ts()} 준비 완료!')
            break
    else:
        print(f'[EKF] {_ts()} 타임아웃 flags={last_flags:#06x if last_flags else 0} — 강행')

    # ── RC 채널 스냅샷 ───────────────────────────────────────────────────
    rc = c.recv_match(type='RC_CHANNELS', blocking=True, timeout=2)
    if rc:
        print(f'[RC] {_ts()} '
              f'ch1={rc.chan1_raw} ch2={rc.chan2_raw} ch3={rc.chan3_raw} '
              f'ch4={rc.chan4_raw} ch5={rc.chan5_raw} ch6={rc.chan6_raw} '
              f'ch7={rc.chan7_raw} ch8={rc.chan8_raw} rssi={rc.rssi}')
    else:
        print(f'[RC] {_ts()} RC_CHANNELS 수신 실패')

    # ── SYS_STATUS ───────────────────────────────────────────────────────
    ss = c.recv_match(type='SYS_STATUS', blocking=True, timeout=2)
    if ss:
        print(f'[SYS] {_ts()} '
              f'sensors_present={ss.onboard_control_sensors_present:#010x} '
              f'enabled={ss.onboard_control_sensors_enabled:#010x} '
              f'health={ss.onboard_control_sensors_health:#010x} '
              f'battery={ss.voltage_battery/1000:.2f}V '
              f'load={ss.load/10:.1f}%')
    else:
        print(f'[SYS] {_ts()} SYS_STATUS 수신 실패')

    # ── GUIDED 모드 ──────────────────────────────────────────────────────
    print(f'[MODE] {_ts()} GUIDED 모드(4) 설정 명령 전송...')
    _cmd(c, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
         mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4, 0, 0, 0, 0, 0)
    ack = c.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    if ack:
        print(f'[MODE] {_ts()} SET_MODE ACK: '
              f'result={ack.result} ({_MAV_RESULT.get(ack.result,"?")})')
    else:
        print(f'[MODE] {_ts()} SET_MODE ACK 없음 (타임아웃)')

    hb1 = c.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
    if hb1:
        print(f'[MODE] {_ts()} HEARTBEAT 확인: custom_mode={hb1.custom_mode} '
              f'base_mode={hb1.base_mode:#010b} system_status={hb1.system_status}')

    # ── ARM ──────────────────────────────────────────────────────────────
    print(f'[ARM] {_ts()} ARM 명령 전송...')
    _cmd(c, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1, 0, 0, 0, 0, 0, 0)
    ack = c.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    if ack:
        print(f'[ARM] {_ts()} ARM ACK: result={ack.result} ({_MAV_RESULT.get(ack.result,"?")})')
    else:
        print(f'[ARM] {_ts()} ARM ACK 없음 (타임아웃)')

    print(f'[ARM] {_ts()} ARMED 상태 대기...')
    t_arm = time.time()
    while True:
        hb = c.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if hb is None:
            print(f'[ARM] {_ts()} HEARTBEAT 수신 없음')
            continue
        print(f'[ARM] {_ts()} base_mode={hb.base_mode:#010b} custom_mode={hb.custom_mode} '
              f'system_status={hb.system_status}')
        if hb.base_mode & _ARMED:
            print(f'[ARM] {_ts()} ARMED! ({time.time()-t_arm:.1f}s 경과)')
            break

    # ARM 후 RC 재확인
    rc = c.recv_match(type='RC_CHANNELS', blocking=True, timeout=2)
    if rc:
        print(f'[RC-ARM] {_ts()} '
              f'ch1={rc.chan1_raw} ch2={rc.chan2_raw} ch3={rc.chan3_raw} '
              f'ch4={rc.chan4_raw} ch5={rc.chan5_raw} ch6={rc.chan6_raw} '
              f'ch7={rc.chan7_raw} ch8={rc.chan8_raw} rssi={rc.rssi}')

    time.sleep(0.5)

    # ── TAKEOFF ──────────────────────────────────────────────────────────
    print(f'[TKOF] {_ts()} MAV_CMD_NAV_TAKEOFF 전송 (alt={TAKEOFF_M}m)...')
    _cmd(c, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, float('nan'), 0, 0, TAKEOFF_M)
    ack = c.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    if ack:
        print(f'[TKOF] {_ts()} TAKEOFF ACK: result={ack.result} ({_MAV_RESULT.get(ack.result,"?")})')
        if ack.result != 0:
            print(f'[TKOF] {_ts()} TAKEOFF 거부 — 착륙 후 종료')
            _cmd(c, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0)
            stop.set()
            return
    else:
        print(f'[TKOF] {_ts()} TAKEOFF ACK 없음 (타임아웃) — 계속 진행')

    # 이륙 모니터링 (vz 기반)
    print(f'[TKOF] {_ts()} 이륙 모니터링 (vz<-0.2 m/s 기준, 최대 15s)...')
    deadline = time.time() + 15
    while time.time() < deadline:
        m = c.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
        if m is None:
            print(f'[TKOF] {_ts()} LOCAL_POSITION_NED 없음')
            continue
        print(f'[TKOF] {_ts()} '
              f'x={m.x:.3f} y={m.y:.3f} z={m.z:.3f} '
              f'vx={m.vx:.3f} vy={m.vy:.3f} vz={m.vz:.3f}')
        if m.vz < -0.2:
            print(f'[TKOF] {_ts()} 이륙 감지! vz={m.vz:.3f}')
            break
    else:
        print(f'[TKOF] {_ts()} 이륙 타임아웃 — 착륙 후 종료')
        _cmd(c, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0)
        stop.set()
        return

    # ── HOVER ────────────────────────────────────────────────────────────
    print(f'[HOVR] {_ts()} 호버 {HOVER_S}s 시작')
    deadline = time.time() + HOVER_S
    while time.time() < deadline:
        xy = uwb.get_xy()
        if xy is None:
            print(f'[SAFE] {_ts()} UWB 신호 끊김 — 착륙')
            break
        m = c.recv_match(type='LOCAL_POSITION_NED', blocking=False)
        z_str = f'z={m.z:.3f} vz={m.vz:.3f}' if m else 'z=N/A'
        print(f'[HOVR] {_ts()} xy=({xy[0]:.3f},{xy[1]:.3f}) {z_str} '
              f'남은={deadline-time.time():.1f}s')
        time.sleep(0.5)

    # ── LAND ─────────────────────────────────────────────────────────────
    print(f'[LAND] {_ts()} MAV_CMD_NAV_LAND 전송...')
    _cmd(c, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0)
    ack = c.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    if ack:
        print(f'[LAND] {_ts()} LAND ACK: result={ack.result} ({_MAV_RESULT.get(ack.result,"?")})')
    else:
        print(f'[LAND] {_ts()} LAND ACK 없음')
    stop.set()
    print(f'[DONE] {_ts()} 완료')


if __name__ == '__main__':
    main()
