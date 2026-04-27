"""
flight.py — UWB XY + Barometer Z indoor hover
XY: VISION_POSITION_ESTIMATE (Z 분산=9999 → EKF가 바로미터로 Z 처리)
Z:  EKF3 Barometer (EK3_SRC1_POSZ=1)
"""
import time, threading
from pymavlink import mavutil
from uwb_reader import UWBReader

FC_PORT   = '/dev/ttyACM0'
FC_BAUD   = 57600
TAKEOFF_M = 1.0    # 목표 고도 (m)
HOVER_S   = 5.0    # 호버 시간 (s)
TOLERANCE = 0.3    # 고도 도달 판정 마진 (m)

# EKF 준비 완료 플래그: att | vel_h | pos_rel | pos_abs
_EKF_NEED = 0x001 | 0x002 | 0x008 | 0x010
_ARMED    = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED


def _cmd(c, cmd, *p):
    c.mav.command_long_send(c.target_system, c.target_component, cmd, 0, *p)


def _vision_loop(c, uwb, stop):
    """20Hz로 UWB XY를 VISION_POSITION_ESTIMATE 전송. Z 분산=9999 → 바로미터 전담."""
    cov = [0.0] * 21
    cov[0] = cov[6] = 0.01   # XY 분산 (0.1m σ)
    cov[11] = 9999.0          # Z 분산 무한대 → EKF Z 무시
    while not stop.is_set():
        xy = uwb.get_xy()
        if xy:
            c.mav.vision_position_estimate_send(
                int(time.time() * 1e6), xy[0], xy[1], 0.0, 0, 0, 0, cov, 0)
        time.sleep(0.05)


def _hb_loop(c, stop):
    """GCS heartbeat 1Hz — 없으면 ArduPilot이 GCS 끊김으로 판단할 수 있음."""
    while not stop.is_set():
        c.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        time.sleep(1)


def main():
    # UWB 시작 + origin 대기
    uwb = UWBReader()
    uwb.start()
    print('[UWB] origin 대기...')
    while uwb.get_xy() is None:
        time.sleep(0.2)
    print(f'[UWB] origin 확정: {uwb.get_xy()}')

    # FC 연결
    c = mavutil.mavlink_connection(FC_PORT, baud=FC_BAUD)
    c.wait_heartbeat()
    print(f'[FC] 연결 sysid={c.target_system}')

    c.mav.request_data_stream_send(
        c.target_system, c.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)

    # EKF 로컬 프레임 원점 설정 (실내 = 위경도 0)
    c.mav.set_gps_global_origin_send(c.target_system, 0, 0, 0)

    # 백그라운드 쓰레드
    stop = threading.Event()
    threading.Thread(target=_vision_loop, args=(c, uwb, stop), daemon=True).start()
    threading.Thread(target=_hb_loop,    args=(c, stop),       daemon=True).start()
    time.sleep(1)

    # EKF 준비 대기
    print('[EKF] 준비 대기...')
    deadline = time.time() + 20
    while time.time() < deadline:
        m = c.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=1)
        if m and (m.flags & _EKF_NEED) == _EKF_NEED:
            print(f'[EKF] 준비 완료 flags={m.flags:#06x}')
            break
    else:
        print('[EKF] 타임아웃 — 강행')

    # GUIDED 모드
    _cmd(c, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
         mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4, 0, 0, 0, 0, 0)
    time.sleep(0.5)

    # ARM
    _cmd(c, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1, 0, 0, 0, 0, 0, 0)
    print('[FC] arm 대기...')
    while True:
        hb = c.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if hb and hb.base_mode & _ARMED:
            print('[FC] armed')
            break
    time.sleep(0.5)

    # TAKEOFF
    _cmd(c, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, float('nan'), 0, 0, TAKEOFF_M)
    print(f'[FC] takeoff → {TAKEOFF_M}m')
    start_z = None
    while True:
        m = c.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
        if m is None:
            continue
        if start_z is None:
            start_z = m.z
        print(f'  z={m.z:.2f}', end='\r')
        # NED: z 음수가 위쪽 → start_z보다 (TAKEOFF_M - TOLERANCE) 이상 감소하면 도달
        if m.z < start_z - (TAKEOFF_M - TOLERANCE):
            print(f'\n[FC] 고도 도달 z={m.z:.2f}')
            break

    # HOVER + UWB 안전 감시
    print(f'[FC] 호버 {HOVER_S}s')
    deadline = time.time() + HOVER_S
    while time.time() < deadline:
        if uwb.get_xy() is None:
            print('[SAFETY] UWB 신호 끊김 — 착륙')
            break
        time.sleep(0.1)

    # LAND
    _cmd(c, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0)
    print('[FC] 착륙')
    stop.set()


if __name__ == '__main__':
    main()
