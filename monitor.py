"""monitor.py — 드론/UWB 값 모니터링 (비행 없음)

ARM/이륙/모드변경 없이 텔레메트리만 수신해 Rich 대시보드로 표시.
단일 _reader_loop로 모든 MAVLink 메시지를 읽어 멀티스레드 시리얼 충돌 방지.
"""
import math
import time
import threading
import sys

from pymavlink import mavutil
from rich.live import Live

from uwb_reader import UWBReader
from common import FC_PORT, FC_BAUD
from dashboard import DroneData, render

# ── 상수 ────────────────────────────────────────────────────────────────────

REFRESH_HZ   = 10         # 대시보드 갱신 Hz
_SEV = ['EMERG', 'ALERT', 'CRIT', 'ERR', 'WARN', 'NOTICE', 'INFO', 'DEBUG']

_GUIDED = 4               # ArduPilot custom_mode for GUIDED
_ARMED_FLAG = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED


# ── MAVLink 메시지 핸들러 ─────────────────────────────────────────────────────

def _handle_heartbeat(data: DroneData, m):
    armed = bool(m.base_mode & _ARMED_FLAG)
    mode  = 'GUIDED' if m.custom_mode == _GUIDED else str(m.custom_mode)
    data.update(base_mode=m.base_mode, custom_mode=m.custom_mode,
                armed=armed, mode_str=mode)


def _handle_local_position_ned(data: DroneData, m):
    data.update(pos_x=m.x, pos_y=m.y, pos_z=m.z,
                vel_x=m.vx, vel_y=m.vy, vel_z=m.vz)


def _handle_attitude(data: DroneData, m):
    data.update(roll=m.roll, pitch=m.pitch, yaw=m.yaw,
                rollspeed=m.rollspeed, pitchspeed=m.pitchspeed, yawspeed=m.yawspeed)


def _handle_raw_imu(data: DroneData, m):
    data.update(xacc=m.xacc, yacc=m.yacc, zacc=m.zacc,
                xgyro=m.xgyro, ygyro=m.ygyro, zgyro=m.zgyro)


def _handle_vfr_hud(data: DroneData, m):
    data.update(airspeed=m.airspeed, groundspeed=m.groundspeed,
                alt_vfr=m.alt, climb=m.climb)


def _handle_scaled_pressure(data: DroneData, m):
    data.update(press_abs=m.press_abs, temperature=m.temperature)


def _handle_ekf_status_report(data: DroneData, m):
    data.update(
        ekf_flags=m.flags,
        ekf_vel_var=m.velocity_variance,
        ekf_pos_horiz_var=m.pos_horiz_variance,
        ekf_pos_vert_var=m.pos_vert_variance,
        ekf_compass_var=m.compass_variance,
        ekf_terrain_var=m.terrain_alt_variance,
    )


def _handle_rc_channels(data: DroneData, m):
    rc = {
        1: m.chan1_raw, 2: m.chan2_raw, 3: m.chan3_raw, 4: m.chan4_raw,
        5: m.chan5_raw, 6: m.chan6_raw, 7: m.chan7_raw, 8: m.chan8_raw,
    }
    data.update(rc=rc)


def _handle_sys_status(data: DroneData, m):
    data.update(
        bat_voltage=m.voltage_battery / 1000.0,
        bat_current=m.current_battery / 100.0 if m.current_battery >= 0 else None,
        bat_remain=m.battery_remaining if m.battery_remaining >= 0 else None,
        cpu_load=m.load,
    )


def _handle_statustext(data: DroneData, m):
    sev = _SEV[m.severity] if m.severity < len(_SEV) else str(m.severity)
    data.add_log(m.text.rstrip('\x00'), level=sev)


_HANDLERS = {
    'HEARTBEAT':           _handle_heartbeat,
    'LOCAL_POSITION_NED':  _handle_local_position_ned,
    'ATTITUDE':            _handle_attitude,
    'RAW_IMU':             _handle_raw_imu,
    'VFR_HUD':             _handle_vfr_hud,
    'SCALED_PRESSURE':     _handle_scaled_pressure,
    'EKF_STATUS_REPORT':   _handle_ekf_status_report,
    'RC_CHANNELS':         _handle_rc_channels,
    'SYS_STATUS':          _handle_sys_status,
    'STATUSTEXT':          _handle_statustext,
}


# ── 백그라운드 루프 ───────────────────────────────────────────────────────────

def _reader_loop(c, data: DroneData, stop: threading.Event):
    """단일 스레드로 모든 MAVLink 메시지를 읽고 핸들러로 분기."""
    while not stop.is_set():
        try:
            m = c.recv_match(blocking=True, timeout=0.1)
            if m is None:
                continue
            with data._lock:
                data.msg_count += 1
            handler = _HANDLERS.get(m.get_type())
            if handler:
                handler(data, m)
        except Exception as e:
            data.add_log(f'reader 오류: {e}', level='ERR')


def _hb_loop(c, stop: threading.Event):
    """1Hz GCS heartbeat — FC가 GCS 끊김 failsafe를 트리거하지 않도록."""
    while not stop.is_set():
        try:
            c.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        except Exception:
            pass
        time.sleep(1)


def _uwb_loop(c, uwb: UWBReader, data: DroneData, stop: threading.Event):
    """20Hz UWB → VISION_POSITION_ESTIMATE 전송 + DroneData 갱신."""
    cov = [0.0] * 21
    cov[0] = cov[6] = 0.01
    cov[11] = 9999.0
    last_uwb_time = 0.0

    while not stop.is_set():
        xy = uwb.get_xy()
        now = time.time()
        if xy:
            last_uwb_time = now
            # 현재 yaw echo-back (common._vision_loop과 동일 방식)
            yaw = data.yaw if data.yaw is not None else 0.0
            try:
                c.mav.vision_position_estimate_send(
                    int(now * 1e6), xy[0], xy[1], 0.0, 0, 0, yaw, cov)
            except Exception as e:
                data.add_log(f'VPE 전송 오류: {e}', level='ERR')
        age = now - last_uwb_time if last_uwb_time else 999.0
        stats = uwb.get_stats()
        data.update(
            uwb_xy=xy, uwb_age_s=age,
            uwb_speed_ms=stats['speed_ms'],
            uwb_rejects=stats['reject_count'],
            uwb_total=stats['total_count'],
        )
        time.sleep(0.05)


# ── 메인 ────────────────────────────────────────────────────────────────────

def main():
    # UWB 시작
    uwb = UWBReader()
    uwb.start()
    print('[UWB] origin 대기...')
    t0 = time.time()
    while uwb.get_xy() is None:
        if time.time() - t0 > 10:
            print('[UWB] 10s 타임아웃 — UWB 없이 계속')
            break
        time.sleep(0.2)
    print(f'[UWB] 초기 XY: {uwb.get_xy()}')

    # FC 연결 (heartbeat만 기다림, ARM/모드변경 없음)
    print(f'[FC] {FC_PORT}@{FC_BAUD} 연결 중...')
    c = mavutil.mavlink_connection(FC_PORT, baud=FC_BAUD)
    hb = c.wait_heartbeat(timeout=10)
    if hb is None:
        print('[FC] Heartbeat 타임아웃 — FC 연결 확인 필요')
        sys.exit(1)
    print(f'[FC] sysid={c.target_system} 연결 완료')

    # 데이터 스트림 요청
    c.mav.request_data_stream_send(
        c.target_system, c.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)

    data = DroneData()
    stop = threading.Event()

    threading.Thread(target=_reader_loop, args=(c, data, stop), daemon=True).start()
    threading.Thread(target=_hb_loop,     args=(c, stop),        daemon=True).start()
    threading.Thread(target=_uwb_loop,    args=(c, uwb, data, stop), daemon=True).start()

    print('[MON] 대시보드 시작 (Ctrl+C 종료)')
    time.sleep(0.5)   # 첫 데이터 수신 대기

    try:
        with Live(render(data), refresh_per_second=REFRESH_HZ, screen=True) as live:
            while True:
                live.update(render(data))
                time.sleep(1 / REFRESH_HZ)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        print('\n[MON] 종료')


if __name__ == '__main__':
    main()
