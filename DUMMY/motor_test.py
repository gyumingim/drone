"""
Motor spin test via RC override in STABILIZE mode.
Confirms MAVLink motor control path works.
"""

import time
from pymavlink import mavutil

PORT = '/dev/ttyACM0'
BAUD = 57600


def connect():
    print("Connecting...")
    conn = mavutil.mavlink_connection(PORT, baud=BAUD)
    conn.wait_heartbeat()
    print(f"Connected (sysid={conn.target_system})")
    return conn


def set_mode(conn, mode_id):
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id, 0, 0, 0, 0, 0
    )
    time.sleep(0.5)


def arm(conn, timeout=10):
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, 0, 0, 0, 0, 0, 0
    )
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = conn.recv_match(
            type=['HEARTBEAT', 'STATUSTEXT'],
            blocking=True, timeout=1.0
        )
        if not msg:
            continue
        if msg.get_type() == 'STATUSTEXT':
            print(f"  [FC] {msg.text.strip()}")
        if msg.get_type() == 'HEARTBEAT':
            if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                print("  Armed OK")
                return True
    print("  Arm failed")
    return False


def disarm(conn):
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        0, 0, 0, 0, 0, 0, 0
    )
    # clear RC override
    conn.mav.rc_channels_override_send(
        conn.target_system, conn.target_component,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    print("Disarmed")


def send_throttle(conn, throttle_pwm):
    """RC override — ch3 = throttle (1000=min, 2000=max)."""
    conn.mav.rc_channels_override_send(
        conn.target_system, conn.target_component,
        65535,  # ch1 roll  (65535 = passthrough)
        65535,        # ch2 pitch
        throttle_pwm, # ch3 throttle
        65535,        # ch4 yaw
        65535, 65535, 65535, 65535
    )


def main():
    conn = connect()

    print("\nSetting STABILIZE mode (0)...")
    set_mode(conn, 0)

    if not arm(conn):
        print("Cannot proceed — fix arm issue first")
        return

    print("\nSending throttle 1150 (low) for 3s...")
    print("Motors should spin slowly.")
    deadline = time.time() + 3.0
    while time.time() < deadline:
        send_throttle(conn, 1150)
        time.sleep(0.05)

    disarm(conn)
    print("\nIf motors did NOT spin:")
    print("  - Check BRD_SAFETYENABLE=0 in QGC")
    print("  - Check RC_OVERRIDE_TIME param (set to 3)")
    print("  - Check SERVO_BLH_MASK or SERVO output config")


if __name__ == '__main__':
    main()
