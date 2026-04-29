"""
lib_common.py — MAVLink 공유 상수, 헬퍼 함수, 백그라운드 스레드, 비행 시퀀스

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
단일 리더 스레드 아키텍처
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  recv_match()는 _reader_loop 단 하나만 호출.
  다른 스레드가 동시에 recv_match()를 호출하면 메시지가 서로 빼앗겨 유실됨.
  모든 메시지는 cache dict에 저장 → 다른 스레드는 cache에서만 읽음.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
백그라운드 스레드 구성
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  _reader_loop      : MAVLink 수신 → cache 저장
  _vision_loop      : UWB → VISION_POSITION_ESTIMATE 20Hz 전송
  _hb_loop          : GCS heartbeat 1Hz 전송 (FC watchdog 방지)
  _rc_override_loop : throttle failsafe 방지용 RC override 5Hz
"""
import os
# MAVLINK20=1: pymavlink이 MAVLink 2 프로토콜 사용
# MAVLink 2가 있어야 VISION_POSITION_ESTIMATE의 covariance, reset_counter 필드가 전송됨
# import 전에 설정해야 적용됨
os.environ['MAVLINK20'] = '1'

import sys
import math
import time
import threading
from queue import Queue, Empty

from loguru import logger
from pymavlink import mavutil

# ── loguru 설정 ───────────────────────────────────────────────────────────────
# 기본 핸들러 제거 후 재설정
logger.remove()
# 터미널: 시각 + 레벨 + 메시지 (간결하게)
logger.add(sys.stderr, format="{time:HH:mm:ss.SSS} | {level:<7} | {message}")
# 파일: 날짜별 로그, 10MB 초과 시 rotation, 7일 보관
logger.add(
    "logs/drone_{time:YYYY-MM-DD}.log",
    rotation="10 MB",
    retention="7 days",
    encoding="utf-8",
    format="{time:YYYY-MM-DD HH:mm:ss.SSS} | {level:<7} | {message}",
)

# ── 하드웨어 설정 ─────────────────────────────────────────────────────────────
FC_PORT   = '/dev/ttyACM0'   # FC USB-UART 포트 (Pixhawk/ArduPilot)
FC_BAUD   = 57600            # ArduPilot 기본 텔레메트리 보레이트
TAKEOFF_M = 1.0              # 이륙 목표 고도 (m)
HOVER_S   = 5.0              # flight.py 호버 시간 (s)

# ── EKF 준비 확인 플래그 ──────────────────────────────────────────────────────
# EKF_STATUS_REPORT.flags에서 아래 비트가 모두 set돼야 비행 가능
# 0x001=att(자세), 0x002=vel_h(수평속도), 0x008=pos_rel(상대위치), 0x010=pos_abs(절대위치)
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

# SET_POSITION_TARGET_LOCAL_NED type_mask
# 0x0DF8 = 0b110111111000 = 3576
# bit0~2: pos x,y,z → 0 (사용)
# bit3~8: vel, acc  → 1 (무시)
# bit9:   force     → 0 (반드시 0, 1이면 force 모드로 오동작)
# bit10~11: yaw, yaw_rate → 1 (무시)
# 참조: ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
_POS_MASK = 0b0000110111111000   # 0x0DF8 / 3576


# ── 헬퍼 함수 ─────────────────────────────────────────────────────────────────

def ekf_str(flags):
    """EKF flags 정수를 비트 이름 문자열로 변환 (로그 출력용)."""
    return '|'.join(n for b, n in _EKF_BITS.items() if flags & b) or 'none'


def cmd(c, command, *p):
    """COMMAND_LONG 전송 헬퍼. confirmation=0 고정."""
    c.mav.command_long_send(c.target_system, c.target_component, command, 0, *p)


def go_to(c, x_north, y_east, z_ned):
    """SET_POSITION_TARGET_LOCAL_NED로 NED 절대 위치 목표 전송.

    z_ned: NED 기준 (음수=위쪽). 예: z_ned=-1.0 → 지면 위 1m
    위치만 사용(_POS_MASK), 속도/가속도/yaw는 FC 내부 제어기가 결정.
    """
    c.mav.set_position_target_local_ned_send(
        0,
        c.target_system, c.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        _POS_MASK,
        x_north, y_east, z_ned,
        0, 0, 0,
        0, 0, 0,
        0, 0)


def wait_pos(c, cache, lock, x, y, tol=0.3, timeout=20):
    """LOCAL_POSITION_NED 기반 XY 도달 대기.

    tol: 허용 오차 (m), timeout: 최대 대기 시간 (s)
    도달 시 True, 타임아웃 시 False.
    z(고도)는 체크하지 않음 — 고도는 FC 내부에서 CRUISE_Z로 유지.
    """
    deadline = time.time() + timeout
    while time.time() < deadline:
        with lock:
            m = cache['local_pos']
        if m is None:
            time.sleep(0.05)
            continue
        dist = ((m.x - x)**2 + (m.y - y)**2) ** 0.5
        logger.debug('[NAV] pos=({:.2f},{:.2f}) target=({:.2f},{:.2f}) dist={:.2f}m',
                     m.x, m.y, x, y, dist)
        if dist < tol:
            return True
        time.sleep(0.1)
    return False


# ── 백그라운드 스레드 ─────────────────────────────────────────────────────────

def _reader_loop(c, cache, lock, stop):
    """유일하게 recv_match()를 호출하는 MAVLink 수신 스레드.

    COMMAND_ACK는 Queue에 넣어 _wait_ack()에서 꺼내 씀.
    STATUSTEXT는 FC가 보내는 텍스트 알림 (arming 실패 이유 등) → 즉시 출력.
    """
    while not stop.is_set():
        try:
            m = c.recv_match(blocking=True, timeout=0.1)
            if m is None:
                continue
            t = m.get_type()
            with lock:
                if t == 'ATTITUDE':
                    cache['attitude'] = m
                    cache['yaw'] = m.yaw
                elif t == 'LOCAL_POSITION_NED':
                    cache['local_pos'] = m
                elif t == 'EKF_STATUS_REPORT':
                    cache['ekf'] = m
                elif t == 'HEARTBEAT':
                    cache['heartbeat'] = m
                elif t == 'COMMAND_ACK':
                    cache['ack_queue'].put(m)
                elif t == 'STATUSTEXT':
                    sev = _SEV[m.severity] if m.severity < len(_SEV) else str(m.severity)
                    logger.info('[FC] [{}] {}', sev, m.text)
        except Exception as e:
            logger.error('[READER] 오류: {}', e)


def _vision_loop(c, uwb, cache, lock, stop):
    """20Hz UWB → VISION_POSITION_ESTIMATE 전송 스레드.

    connect(start_vision=True)일 때만 시작됨.
    flight_tag.py는 start_vision=False로 직접 VPE를 관리.
    yaw는 cache에서 읽어 VPE yaw 필드에 넣음 (EKF yaw 추정 보조).
    20개마다 1회 로그 출력 (1Hz).
    """
    cov = [0.0] * 21
    cov[0] = cov[6] = 0.01   # xy ±10cm
    cov[11] = 9999.0          # z 무시
    sent = 0
    while not stop.is_set():
        with lock:
            yaw = cache['yaw']
        xy = uwb.get_xy()
        if xy:
            c.mav.vision_position_estimate_send(
                int(time.time() * 1e6), xy[0], xy[1], 0.0, 0, 0, yaw, cov)
            sent += 1
            if sent % 20 == 0:
                logger.debug('[VIS] xy=({:.3f},{:.3f}) yaw={:.3f}rad total={}',
                             xy[0], xy[1], yaw, sent)
        time.sleep(0.05)


def _hb_loop(c, stop):
    """GCS heartbeat 1Hz 전송. FC watchdog 방지."""
    while not stop.is_set():
        c.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        time.sleep(1)


def _rc_override_loop(c, stop):
    """RC override 5Hz — ch3(throttle)=1000 고정으로 throttle failsafe 방지.

    나머지 채널 0(release) → 물리적 RC 조종기 입력 통과.
    """
    while not stop.is_set():
        try:
            c.mav.rc_channels_override_send(
                c.target_system, c.target_component,
                0, 0, 1000, 0,
                0, 0, 0, 0)
        except Exception as e:
            logger.warning('[RC-OVR] 오류: {}', e)
        time.sleep(0.2)


# ── 공통 비행 시퀀스 ──────────────────────────────────────────────────────────

def _wait_ack(cache, timeout=3):
    """ack_queue에서 COMMAND_ACK 꺼냄. 타임아웃 시 None."""
    try:
        return cache['ack_queue'].get(timeout=timeout)
    except Empty:
        return None


def connect(uwb, start_vision=True):
    """FC 연결, 스레드 시작, EKF 준비 대기, GUIDED 모드 설정, ARM.

    Args:
        uwb: UWBReader 인스턴스 (_vision_loop에서 사용)
        start_vision: True면 UWB VPE 스레드 자동 시작.
                      flight_tag.py처럼 직접 VPE를 관리할 때는 False.

    Returns:
        성공: (c, stop, cache, lock)
        실패(ARM 불가): (None, stop, None, None)
    """
    logger.info('[FC] {}@{} 연결 중...', FC_PORT, FC_BAUD)
    c = mavutil.mavlink_connection(FC_PORT, baud=FC_BAUD)
    hb0 = c.wait_heartbeat()
    logger.info('[FC] sysid={} base_mode={:#010b} custom_mode={}',
                c.target_system, hb0.base_mode, hb0.custom_mode)

    c.mav.request_data_stream_send(
        c.target_system, c.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)

    # alt=0이면 EKF가 1Hz마다 고도를 재계산해 LOCAL_POSITION_NED.z가 튀는 버그 있음
    c.mav.set_gps_global_origin_send(c.target_system, 0, 0, 100000)

    cache = {
        'attitude':  None,
        'local_pos': None,
        'ekf':       None,
        'heartbeat': None,
        'yaw':       0.0,
        'ack_queue': Queue(),
    }
    lock = threading.Lock()
    stop = threading.Event()

    threading.Thread(
        target=_reader_loop, args=(c, cache, lock, stop), daemon=True).start()
    if start_vision:
        threading.Thread(
            target=_vision_loop, args=(c, uwb, cache, lock, stop), daemon=True).start()
    threading.Thread(target=_hb_loop, args=(c, stop), daemon=True).start()
    threading.Thread(target=_rc_override_loop, args=(c, stop), daemon=True).start()
    logger.info('[THREAD] 스레드 시작')
    time.sleep(1)

    # EKF 준비 대기 (최대 20초)
    logger.info('[EKF] 준비 대기...')
    deadline = time.time() + 20
    last_flags = None
    while time.time() < deadline:
        with lock:
            m = cache['ekf']
        if m is None:
            time.sleep(0.05)
            continue
        if m.flags != last_flags:
            logger.info('[EKF] flags={:#06x} ({})', m.flags, ekf_str(m.flags))
            last_flags = m.flags
        if (m.flags & _EKF_NEED) == _EKF_NEED:
            logger.info('[EKF] 준비 완료!')
            break
    else:
        logger.warning('[EKF] 타임아웃 — 강행')

    # GUIDED 모드 (custom_mode=4)
    cmd(c, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4, 0, 0, 0, 0, 0)
    ack = _wait_ack(cache)
    logger.info('[MODE] GUIDED ACK: {}',
                _MAV_RESULT.get(getattr(ack, 'result', -1), '?'))

    # ARM
    cmd(c, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1, 0, 0, 0, 0, 0, 0)
    ack = _wait_ack(cache)
    result = getattr(ack, 'result', -1)
    logger.info('[ARM] ARM ACK: {}', _MAV_RESULT.get(result, '?'))
    if result != 0:
        logger.error('[ARM] ARM 실패 — pre-arm check 확인 필요')
        stop.set()
        return None, stop, None, None

    t0 = time.time()
    while True:
        with lock:
            hb = cache['heartbeat']
        if hb and hb.base_mode & _ARMED:
            logger.info('[ARM] ARMED! ({:.1f}s)', time.time() - t0)
            break
        time.sleep(0.1)
    time.sleep(0.5)
    return c, stop, cache, lock


def do_takeoff(c, stop, cache, lock, takeoff_m=TAKEOFF_M):
    """NAV_TAKEOFF 명령 + 목표 고도 도달 대기. 성공 True, 실패 False."""
    cmd(c, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, float('nan'), 0, 0, takeoff_m)
    ack = _wait_ack(cache)
    result = getattr(ack, 'result', -1)
    logger.info('[TKOF] TAKEOFF ACK: {}', _MAV_RESULT.get(result, '?'))
    if result != 0:
        return False

    target_z  = -(takeoff_m * 0.95)   # 목표의 95% 고도 도달 시 성공 판정
    deadline  = time.time() + 20
    last_print = 0.0
    while time.time() < deadline:
        with lock:
            m   = cache['local_pos']
            att = cache['attitude']
        if m is None:
            time.sleep(0.05)
            continue

        if m.z < target_z:
            logger.info('[TKOF] 고도 도달!')
            return True

        now = time.time()
        if now - last_print >= 0.1:
            roll_deg  = math.degrees(att.roll)  if att else float('nan')
            pitch_deg = math.degrees(att.pitch) if att else float('nan')
            logger.debug('[TKOF] z={:.3f} vz={:.3f} | x={:.3f} y={:.3f} | '
                         'roll={:.1f}° pitch={:.1f}°',
                         m.z, m.vz, m.x, m.y, roll_deg, pitch_deg)
            last_print = now
        time.sleep(0.02)

    logger.warning('[TKOF] 타임아웃')
    return False


def do_land(c, stop, cache):
    """NAV_LAND 명령 + 모든 백그라운드 스레드 정지."""
    cmd(c, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0)
    ack = _wait_ack(cache)
    logger.info('[LAND] LAND ACK: {}',
                _MAV_RESULT.get(getattr(ack, 'result', -1), '?'))
    stop.set()
    logger.info('[DONE] 완료')
