"""dashboard.py — Rich 기반 CLI 대시보드 렌더러

DroneData: 스레드 안전한 텔레메트리 저장소
render():  Rich Layout 반환 (monitor.py에서 Live로 출력)
"""
import math
import time
import threading
from collections import deque
from dataclasses import dataclass, field
from typing import Optional, Tuple

from rich.console import Console
from rich.layout import Layout
from rich.panel import Panel
from rich.table import Table
from rich.text import Text
from rich import box

# ── 상수 ────────────────────────────────────────────────────────────────────

MAX_LOG = 30          # 로그 최대 줄 수
WARN_KEYWORDS = ('warn', 'fail', 'error', 'bad', 'ekf', 'timeout',
                 'crash', 'critical', 'disarm', 'overflow', 'lost')

_EKF_BITS = {
    0x001: 'att',      0x002: 'vel_h',   0x004: 'vel_v',
    0x008: 'pos_rel',  0x010: 'pos_abs', 0x020: 'const_pos',
    0x040: 'pred_h',   0x080: 'pred_v',  0x100: 'pred_rel',
    0x200: 'gps_gltch',0x400: 'accel_err',
}


# ── 데이터 클래스 ────────────────────────────────────────────────────────────

@dataclass
class DroneData:
    """스레드 안전한 텔레메트리 저장소."""

    _lock: threading.Lock = field(default_factory=threading.Lock, repr=False)

    # UWB
    uwb_xy:    Optional[Tuple[float, float]] = None
    uwb_age_s: float = 999.0
    uwb_total: int   = 0

    # LOCAL_POSITION_NED
    pos_x:  Optional[float] = None      # North (m)
    pos_y:  Optional[float] = None      # East  (m)
    pos_z:  Optional[float] = None      # Down  (m, 음수=위)
    vel_x:  Optional[float] = None
    vel_y:  Optional[float] = None
    vel_z:  Optional[float] = None

    # ATTITUDE
    roll:   Optional[float] = None      # rad
    pitch:  Optional[float] = None      # rad
    yaw:    Optional[float] = None      # rad
    rollspeed:  Optional[float] = None
    pitchspeed: Optional[float] = None
    yawspeed:   Optional[float] = None

    # RAW_IMU (counts → 내부 단위)
    xacc:  Optional[int] = None
    yacc:  Optional[int] = None
    zacc:  Optional[int] = None
    xgyro: Optional[int] = None
    ygyro: Optional[int] = None
    zgyro: Optional[int] = None

    # VFR_HUD
    airspeed:  Optional[float] = None
    groundspeed: Optional[float] = None
    alt_vfr:   Optional[float] = None   # m (AGL from baro)
    climb:     Optional[float] = None

    # SCALED_PRESSURE
    press_abs:  Optional[float] = None  # hPa
    temperature: Optional[float] = None  # 0.01 degC

    # EKF_STATUS_REPORT
    ekf_flags:    Optional[int]   = None
    ekf_vel_var:  Optional[float] = None
    ekf_pos_horiz_var: Optional[float] = None
    ekf_pos_vert_var:  Optional[float] = None
    ekf_compass_var:   Optional[float] = None
    ekf_terrain_var:   Optional[float] = None

    # RC_CHANNELS
    rc: dict = field(default_factory=dict)  # {ch: pwm}

    # SYS_STATUS
    bat_voltage:  Optional[float] = None   # V
    bat_current:  Optional[float] = None   # A
    bat_remain:   Optional[int]   = None   # %
    cpu_load:     Optional[int]   = None   # per-mille

    # HEARTBEAT
    base_mode:    Optional[int]   = None
    custom_mode:  Optional[int]   = None
    armed:        bool            = False
    mode_str:     str             = '?'

    # 로그
    logs: deque = field(default_factory=lambda: deque(maxlen=MAX_LOG))

    # 통계
    msg_count:    int   = 0
    start_time:   float = field(default_factory=time.time)

    def update(self, **kwargs):
        with self._lock:
            for k, v in kwargs.items():
                setattr(self, k, v)

    def snapshot(self):
        with self._lock:
            import copy
            d = copy.copy(self)
            d.logs = list(self.logs)
            d.rc   = dict(self.rc)
            return d

    def add_log(self, text: str, level: str = 'INFO'):
        ts = time.strftime('%H:%M:%S')
        with self._lock:
            self.logs.append((ts, level, text))


# ── 렌더 헬퍼 ───────────────────────────────────────────────────────────────

def _val(v, fmt='.2f', unit='', na='—') -> Text:
    if v is None:
        return Text(na, style='dim')
    s = f'{v:{fmt}}{unit}'
    return Text(s)


def _rad2deg(r) -> Optional[float]:
    return None if r is None else math.degrees(r)


def _ekf_flags_str(flags: Optional[int]) -> Tuple[str, str]:
    if flags is None:
        return '—', 'dim'
    active = [n for b, n in _EKF_BITS.items() if flags & b]
    bad    = flags & 0x200 or flags & 0x400
    style  = 'red bold' if bad else ('green' if active else 'yellow')
    return ' '.join(active) or 'none', style


def _armed_text(armed: bool, mode: str) -> Text:
    if armed:
        return Text(f'ARMED  [{mode}]', style='red bold')
    return Text(f'DISARMED [{mode}]', style='green')


# ── 각 패널 렌더 ─────────────────────────────────────────────────────────────

def _panel_uwb(d) -> Panel:
    t = Table.grid(padding=(0, 1))
    t.add_column(width=14); t.add_column()

    if d.uwb_xy:
        xy_text  = Text(f'({d.uwb_xy[0]:.3f}, {d.uwb_xy[1]:.3f}) m')
        age_style = 'red' if d.uwb_age_s > 1.0 else 'green'
        age_text  = Text(f'{d.uwb_age_s:.2f}s', style=age_style)
    else:
        xy_text  = Text('NO SIGNAL', style='red bold blink')
        age_text = Text('—', style='dim')

    t.add_row('[bold]XY[/bold]',    xy_text)
    t.add_row('[bold]Age[/bold]',   age_text)
    t.add_row('[bold]Count[/bold]', Text(str(d.uwb_total)))
    return Panel(t, title='[cyan]UWB[/cyan]', box=box.ROUNDED, padding=(0,1))


def _panel_position(d) -> Panel:
    t = Table.grid(padding=(0, 1))
    t.add_column(width=6); t.add_column(width=9); t.add_column(width=9)
    t.add_row('[bold]Axis[/bold]', '[bold]Pos (m)[/bold]', '[bold]Vel (m/s)[/bold]')

    for axis, pos, vel in [('N(x)', d.pos_x, d.vel_x),
                            ('E(y)', d.pos_y, d.vel_y),
                            ('D(z)', d.pos_z, d.vel_z)]:
        p = _val(pos); v = _val(vel)
        if vel is not None and abs(vel) > 1.5:
            v.stylize('yellow')
        t.add_row(axis, p, v)

    alt_m = None if d.pos_z is None else -d.pos_z
    t.add_row('[dim]Alt[/dim]', _val(alt_m, unit='m'), Text(''))

    return Panel(t, title='[cyan]Position NED[/cyan]', box=box.ROUNDED, padding=(0,1))


def _panel_attitude(d) -> Panel:
    t = Table.grid(padding=(0, 1))
    t.add_column(width=9); t.add_column(width=10); t.add_column(width=10)
    t.add_row('[bold]Angle[/bold]', '[bold]deg[/bold]', '[bold]rate °/s[/bold]')

    for label, ang, rate in [('Roll',  d.roll,  d.rollspeed),
                               ('Pitch', d.pitch, d.pitchspeed),
                               ('Yaw',   d.yaw,   d.yawspeed)]:
        deg  = _rad2deg(ang)
        rdeg = None if rate is None else math.degrees(rate)
        av = _val(deg);  rv = _val(rdeg)
        if deg is not None and label in ('Roll', 'Pitch') and abs(deg) > 20:
            av.stylize('red bold')
        t.add_row(label, av, rv)

    return Panel(t, title='[cyan]Attitude[/cyan]', box=box.ROUNDED, padding=(0,1))


def _panel_imu(d) -> Panel:
    t = Table.grid(padding=(0, 1))
    t.add_column(width=7); t.add_column(width=8); t.add_column(width=8); t.add_column(width=8)
    t.add_row('[bold][/bold]', '[bold]X[/bold]', '[bold]Y[/bold]', '[bold]Z[/bold]')
    t.add_row('Accel',
              _val(d.xacc, 'd'), _val(d.yacc, 'd'), _val(d.zacc, 'd'))
    t.add_row('Gyro',
              _val(d.xgyro, 'd'), _val(d.ygyro, 'd'), _val(d.zgyro, 'd'))
    return Panel(t, title='[cyan]RAW IMU[/cyan]', box=box.ROUNDED, padding=(0,1))


def _panel_baro(d) -> Panel:
    t = Table.grid(padding=(0, 1))
    t.add_column(width=14); t.add_column()
    temp_c = None if d.temperature is None else d.temperature / 100.0

    t.add_row('[bold]Alt(VFR)[/bold]',  _val(d.alt_vfr,   unit='m'))
    t.add_row('[bold]Climb[/bold]',     _val(d.climb,      unit='m/s'))
    t.add_row('[bold]Press[/bold]',     _val(d.press_abs,  unit='hPa'))
    t.add_row('[bold]Temp[/bold]',      _val(temp_c,       unit='°C'))
    t.add_row('[bold]GndSpd[/bold]',    _val(d.groundspeed,unit='m/s'))
    return Panel(t, title='[cyan]Baro / VFR_HUD[/cyan]', box=box.ROUNDED, padding=(0,1))


def _panel_ekf(d) -> Panel:
    t = Table.grid(padding=(0, 1))
    t.add_column(width=18); t.add_column()

    flags_str, flags_style = _ekf_flags_str(d.ekf_flags)
    t.add_row('[bold]Flags[/bold]',
              Text(f'{d.ekf_flags:#06x} ' if d.ekf_flags is not None else '—') +
              Text(flags_str, style=flags_style))

    for label, val in [('Vel var',   d.ekf_vel_var),
                        ('PosH var', d.ekf_pos_horiz_var),
                        ('PosV var', d.ekf_pos_vert_var),
                        ('Compass',  d.ekf_compass_var),
                        ('Terrain',  d.ekf_terrain_var)]:
        v = _val(val, '.4f')
        if val is not None and val > 1.0:
            v.stylize('yellow')
        if val is not None and val > 5.0:
            v.stylize('red bold')
        t.add_row(f'[bold]{label}[/bold]', v)

    return Panel(t, title='[cyan]EKF Status[/cyan]', box=box.ROUNDED, padding=(0,1))


def _panel_rc(d) -> Panel:
    t = Table.grid(padding=(0, 1))
    t.add_column(width=5); t.add_column(width=6)

    if not d.rc:
        t.add_row(Text('No data', style='dim'), Text(''))
    else:
        items = sorted(d.rc.items())
        for ch, pwm in items:
            style = ''
            if ch == 3:
                style = 'green' if pwm > 1100 else 'red'
            t.add_row(f'CH{ch}', Text(str(pwm), style=style))

    return Panel(t, title='[cyan]RC Channels[/cyan]', box=box.ROUNDED, padding=(0,1))


def _panel_sys(d) -> Panel:
    t = Table.grid(padding=(0, 1))
    t.add_column(width=14); t.add_column()

    bat_v = _val(d.bat_voltage, '.2f', 'V')
    if d.bat_voltage is not None and d.bat_voltage < 10.5:
        bat_v.stylize('red bold')

    bat_r = _val(d.bat_remain, 'd', '%')
    if d.bat_remain is not None and d.bat_remain < 20:
        bat_r.stylize('red bold')

    load = _val(d.cpu_load, 'd')
    if d.cpu_load is not None and d.cpu_load > 800:
        load.stylize('yellow')

    uptime = int(time.time() - d.start_time)

    t.add_row('[bold]Voltage[/bold]',  bat_v)
    t.add_row('[bold]Current[/bold]',  _val(d.bat_current, '.1f', 'A'))
    t.add_row('[bold]Battery[/bold]',  bat_r)
    t.add_row('[bold]CPU load[/bold]', load)
    t.add_row('[bold]Msg count[/bold]',Text(str(d.msg_count)))
    t.add_row('[bold]Uptime[/bold]',   Text(f'{uptime}s'))

    return Panel(t, title='[cyan]System[/cyan]', box=box.ROUNDED, padding=(0,1))


def _panel_log(logs) -> Panel:
    text = Text()
    for ts_str, level, msg in logs[-MAX_LOG:]:
        low = msg.lower()
        if level in ('ERROR', 'CRIT', 'EMERG', 'ALERT') or \
                any(w in low for w in WARN_KEYWORDS):
            style = 'red bold'
        elif level == 'WARN':
            style = 'yellow'
        elif level == 'DEBUG':
            style = 'dim'
        else:
            style = ''
        text.append(f'[{ts_str}] ', style='dim')
        text.append(f'[{level}] ', style=style)
        text.append(msg + '\n', style=style)
    return Panel(text, title='[cyan]FC Messages / Log[/cyan]', box=box.ROUNDED, padding=(0,1))


# ── 메인 렌더 ────────────────────────────────────────────────────────────────

def render(data: DroneData) -> Layout:
    """DroneData 스냅샷을 받아 Rich Layout을 반환."""
    d = data.snapshot()

    layout = Layout()
    layout.split_column(
        Layout(name='header', size=3),
        Layout(name='body'),
        Layout(name='log', size=MAX_LOG // 2 + 2),
    )

    # Header
    armed_text = _armed_text(d.armed, d.mode_str)
    header_table = Table.grid(expand=True)
    header_table.add_column(ratio=1)
    header_table.add_column(ratio=1)
    header_table.add_row(
        Text('DRONE MONITOR', style='bold magenta'),
        armed_text,
    )
    layout['header'].update(Panel(header_table, box=box.SIMPLE))

    # Body — 2단 분할
    layout['body'].split_row(
        Layout(name='left', ratio=2),
        Layout(name='right', ratio=3),
    )

    # Left: UWB + Baro + RC + Sys
    layout['left'].split_column(
        Layout(_panel_uwb(d),      name='uwb',  ratio=1),
        Layout(_panel_baro(d),     name='baro', ratio=2),
        Layout(_panel_rc(d),       name='rc',   ratio=3),
        Layout(_panel_sys(d),      name='sys',  ratio=2),
    )

    # Right: Pos + Att + IMU + EKF
    layout['right'].split_column(
        Layout(_panel_position(d), name='pos',  ratio=2),
        Layout(_panel_attitude(d), name='att',  ratio=2),
        Layout(_panel_imu(d),      name='imu',  ratio=1),
        Layout(_panel_ekf(d),      name='ekf',  ratio=3),
    )

    # Log
    layout['log'].update(_panel_log(d.logs))

    return layout
