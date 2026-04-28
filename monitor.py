"""monitor.py — 텔레메트리 + UWB 로그 출력 (대시보드 없음)"""
import time
import threading
import sys

from pymavlink import mavutil
from uwb_reader import UWBReader
from common import FC_PORT, FC_BAUD

_SEV = ['EMERG', 'ALERT', 'CRIT', 'ERR', 'WARN', 'NOTICE', 'INFO', 'DEBUG']
_GUIDED = 4
_ARMED_FLAG = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED

_cache = {
    'pos': None, 'att': None, 'ekf': None,
    'armed': False, 'mode': '?',
}
_lock = threading.Lock()


def _reader_loop(c, stop):
    while not stop.is_set():
        try:
            m = c.recv_match(blocking=True, timeout=0.1)
            if m is None:
                continue
            t = m.get_type()
            with _lock:
                if t == 'LOCAL_POSITION_NED':
                    _cache['pos'] = m
                elif t == 'ATTITUDE':
                    _cache['att'] = m
                elif t == 'EKF_STATUS_REPORT':
                    _cache['ekf'] = m
                elif t == 'HEARTBEAT':
                    _cache['armed'] = bool(m.base_mode & _ARMED_FLAG)
                    _cache['mode'] = 'GUIDED' if m.custom_mode == _GUIDED else str(m.custom_mode)
                elif t == 'STATUSTEXT':
                    sev = _SEV[m.severity] if m.severity < len(_SEV) else str(m.severity)
                    print(f'[FC] [{sev}] {m.text.rstrip(chr(0))}')
        except Exception as e:
            print(f'[READER] 오류: {e}')


def _hb_loop(c, stop):
    while not stop.is_set():
        try:
            c.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        except Exception:
            pass
        time.sleep(1)


def _uwb_loop(c, uwb, stop):
    cov = [0.0] * 21
    cov[0] = cov[6] = 0.01
    cov[11] = 9999.0
    while not stop.is_set():
        xyz = uwb.get_xyz()
        xy = (xyz[0], xyz[1]) if xyz else None
        if xy:
            with _lock:
                att = _cache['att']
            yaw = att.yaw if att else 0.0
            try:
                c.mav.vision_position_estimate_send(
                    int(time.time() * 1e6), xy[0], xy[1], 0.0, 0, 0, yaw, cov)
            except Exception as e:
                print(f'[VPE] 오류: {e}')
        time.sleep(0.05)


def main():
    uwb = UWBReader()
    uwb.start()
    print('[UWB] 대기...')
    t0 = time.time()
    while uwb.get_xy() is None:
        err = uwb.get_error()
        if err:
            print(f'[UWB] 에러: {err}')
            sys.exit(1)
        if time.time() - t0 > 10:
            print('[UWB] 10s 타임아웃 — 계속')
            break
        time.sleep(0.2)
    print(f'[UWB] 초기 XY: {uwb.get_xy()}')

    print(f'[FC] {FC_PORT}@{FC_BAUD} 연결...')
    c = mavutil.mavlink_connection(FC_PORT, baud=FC_BAUD)
    if c.wait_heartbeat(timeout=10) is None:
        print('[FC] Heartbeat 타임아웃')
        sys.exit(1)
    print(f'[FC] sysid={c.target_system} 연결 완료')

    c.mav.request_data_stream_send(
        c.target_system, c.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)

    stop = threading.Event()
    threading.Thread(target=_reader_loop, args=(c, stop), daemon=True).start()
    threading.Thread(target=_hb_loop,     args=(c, stop), daemon=True).start()
    threading.Thread(target=_uwb_loop,    args=(c, uwb, stop), daemon=True).start()

    print('[MON] 시작 (Ctrl+C 종료) — 2초마다 출력\n')
    try:
        while True:
            time.sleep(2)
            with _lock:
                pos  = _cache['pos']
                att  = _cache['att']
                ekf  = _cache['ekf']
                armed = _cache['armed']
                mode  = _cache['mode']
            xyz = uwb.get_xyz()

            ts = time.strftime('%H:%M:%S')
            arm_str = 'ARMED' if armed else 'DISARMED'

            pos_str = f'N={pos.x:.2f} E={pos.y:.2f} D={pos.z:.2f}' if pos else '없음'
            att_str = (f'roll={att.roll:.2f} pitch={att.pitch:.2f} yaw={att.yaw:.2f}'
                       if att else '없음')
            ekf_str = f'flags={ekf.flags:#06x}' if ekf else '없음'
            uwb_str = (f'x={xyz[0]:.3f} y={xyz[1]:.3f} z={xyz[2]:.3f}'
                       if xyz else f'없음 (cnt={uwb.get_stats()["total_count"]} err={uwb.get_error()})')

            print(f'[{ts}] {arm_str}[{mode}] | NAV: {pos_str} | ATT: {att_str} | EKF: {ekf_str} | UWB: {uwb_str}')
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        print('\n[MON] 종료')


if __name__ == '__main__':
    main()
