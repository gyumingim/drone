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
    """20Hz로 UWB XY를 VISION_POSITION_ESTIMATE 전송.
    EK3_SRC1_YAW=6(ExternalNav)이므로 현재 EKF yaw를 에코백해 드리프트 방지.
    Z 분산=9999 → 바로미터 전담."""
    cov = [0.0] * 21
    cov[0] = cov[6] = 0.01   # XY 분산 (0.1m σ)
    cov[11] = 9999.0          # Z 분산 무한대 → EKF Z 무시
    yaw = 0.0
    while not stop.is_set():
        att = c.recv_match(type='ATTITUDE', blocking=False)
        if att:
            yaw = att.yaw
        xy = uwb.get_xy()
        if xy:
            c.mav.vision_position_estimate_send(
                int(time.time() * 1e6), xy[0], xy[1], 0.0, 0, 0, yaw, cov, 0)
        time.sleep(0.05)


def _hb_loop(c, stop):
    """GCS heartbeat 1Hz — 없으면 ArduPilot이 GCS 끊김으로 판단할 수 있음."""
    while not stop.is_set():
        c.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        time.sleep(1)


def _status_loop(c, stop):
    """STATUSTEXT 수신 → 실시간 출력 (non-blocking, 메인 쓰레드와 경쟁 방지)."""
    while not stop.is_set():
        m = c.recv_match(type='STATUSTEXT', blocking=False)
        if m:
            print(f'[FC] {m.severity} {m.text}')
        time.sleep(0.05)


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
    threading.Thread(target=_status_loop, args=(c, stop),      daemon=True).start()
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
    ack = c.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    if not ack or ack.result != 0:
        print(f'[FC] TAKEOFF 거부 result={getattr(ack, "result", "timeout")} — 중단')
        stop.set()
        return
    print(f'[FC] takeoff 수락 → {TAKEOFF_M}m')

    # 이륙 완료 판정: vz(수직속도) 기반 — 바로미터 드리프트에 속지 않음
    deadline = time.time() + 15
    while time.time() < deadline:
        m = c.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
        if m is None:
            continue
        print(f'  z={m.z:.2f} vz={m.vz:.2f}', end='\r')
        # NED vz 음수=상승, 0.2m/s 이상 상승 중이면 이륙으로 판정
        if m.vz < -0.2:
            print(f'\n[FC] 이륙 감지 z={m.z:.2f} vz={m.vz:.2f}')
            break
    else:
        print('\n[FC] 이륙 타임아웃 — 착륙')
        _cmd(c, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0)
        stop.set()
        return

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
