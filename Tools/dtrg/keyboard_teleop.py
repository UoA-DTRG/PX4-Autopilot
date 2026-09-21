#!/usr/bin/env python3
"""Fly PX4 SITL (Gazebo) from the keyboard.

Acts as a MAVLink joystick: streams MANUAL_CONTROL to PX4 at 25 Hz.
SITL already sets COM_RC_IN_MODE 1 (joystick only), so no param changes needed.

It can also drive the DTRG horizontal-thrust (HT) inputs. Those are read from
`rc_channels`, which MANUAL_CONTROL never reaches: the mavlink receiver turns
MANUAL_CONTROL into `manual_control_setpoint` only. So the HT keys stream a
second message, RC_CHANNELS_OVERRIDE, which becomes `input_rc` and then
`rc_channels` with the RC<n>_MIN/TRIM/MAX calibration applied. The channels
match the planarOcto airframe defaults (RC_MAP_HT_MODE/ROLL/PITCH = 8/9/10);
Tools/dtrg/rc_override.py does the same thing standalone.

Usage:  Tools/dtrg/keyboard_teleop.py [udpin:0.0.0.0:14540]
"""

import os
import sys
import time
import select
import shutil
import termios
import tty

# Channels 9-18 only exist as MAVLink 2 extension fields, and pymavlink picks
# the protocol version at import time.
os.environ.setdefault("MAVLINK20", "1")

from pymavlink import mavutil  # noqa: E402

CONN = sys.argv[1] if len(sys.argv) > 1 else "udpin:0.0.0.0:14540"

RATE_HZ = 25.0
STEP = 350.0      # stick deflection added per keypress
DECAY = 0.85      # per-tick return-to-center, like releasing a spring-loaded stick
THR_STEP = 40.0   # throttle is not spring-loaded, it holds

# MANUAL_CONTROL aux1..aux4, which PX4 exposes as manual_control_setpoint.aux1-4.
# These are NOT rc_channels 5-8; nothing that reads rc_channels sees them.
# Each key cycles its aux through the three detents a PX4 mode switch expects.
AUX_POSITIONS = (-1000, 0, 1000)
AUX_LABEL = {-1000: "low", 0: "mid", 1000: "high"}
# PX4 only reads aux<n> when bit (n+1) of enabled_extensions is set
# (src/modules/mavlink/mavlink_receiver.cpp:2140).
AUX_EXTENSIONS = (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5)

# DTRG horizontal thrust, over RC_CHANNELS_OVERRIDE. Must agree with the
# airframe: RC_MAP_HT_MODE 8, RC_MAP_HT_ROLL 9, RC_MAP_HT_PITCH 10.
HT_MODE_CH = 8
HT_ROLL_CH = 9
HT_PITCH_CH = 10
HT_STEP = 0.05    # normalized stick units per keypress; HT sticks do not decay
RC_CHANNELS = 18
# PX4's default RC<n>_MIN/TRIM/MAX. Pass different numbers here only if the
# SITL rootfs carries a saved transmitter calibration for these channels.
PWM_MIN, PWM_TRIM, PWM_MAX = 1000, 1500, 2000
# Unused channels: 1-7 go to MIN so every switch function would read OFF (a
# channel left at TRIM reads 0.5 after PX4's 0..1 rescale, which is above
# RC_KILLSWITCH_TH); 11-18 go to 0, which the receiver treats as "not present"
# (src/modules/mavlink/mavlink_receiver.cpp, ignore_zero for channels 9+).
PWM_UNUSED_LOW = PWM_MIN
PWM_ABSENT = 0

HELP = """
  i / k      pitch forward / back      m   arm
  j / l      roll left / right         n   disarm
  a / d      yaw left / right          t   takeoff
  w / s      throttle up / down        g   land
  space      center sticks             p   position mode
                                       h   hold (loiter)
  5 6 7 8    cycle aux1-4 (MANUAL_CONTROL): low -> mid -> high
  0          reset all aux channels to low

  DTRG horizontal thrust (RC_CHANNELS_OVERRIDE, starts on first use)
  e          toggle HT mode switch (ch8)
  z / x      HT roll  - / +  (ch9)
  c / v      HT pitch - / +  (ch10)
  b          center HT roll/pitch

  q / Ctrl-C quit
"""


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def normalized_to_pwm(value):
    """-1..1 stick units -> raw PWM, matching PX4's piecewise RC calibration."""
    value = clamp(value, -1.0, 1.0)
    span = (PWM_MAX - PWM_TRIM) if value >= 0 else (PWM_TRIM - PWM_MIN)
    return int(round(PWM_TRIM + value * span))


def enum_name(enum, value):
    entry = mavutil.mavlink.enums.get(enum, {}).get(value)
    return entry.name if entry else str(value)


def drain(m, limit=300):
    """Read pending telemetry, surfacing why commands were accepted or refused.

    Also stops PX4's 4 Mbit/s stream from filling the socket receive buffer.
    """
    for _ in range(limit):
        msg = m.recv_match(blocking=False)
        if msg is None:
            return
        kind = msg.get_type()
        if kind == "COMMAND_ACK":
            print("\r%s -> %s%s" % (enum_name("MAV_CMD", msg.command),
                                    enum_name("MAV_RESULT", msg.result),
                                    " " * 30))
        elif kind == "STATUSTEXT":
            print("\rPX4: %s%s" % (msg.text, " " * 30))


def main():
    print(f"Connecting on {CONN} ...")
    m = mavutil.mavlink_connection(CONN)
    m.wait_heartbeat()
    print(f"Heartbeat from system {m.target_system} component {m.target_component}")
    print(HELP)

    NAN = float("nan")

    def cmd(command, *params):
        p = list(params) + [0.0] * (7 - len(params))
        m.mav.command_long_send(m.target_system, m.target_component,
                                command, 0, *p)

    def set_mode(main_mode, sub_mode=0):
        cmd(mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            main_mode, sub_mode)

    def send_ht(mode_on, roll, pitch):
        """Stream the HT channels as RC_CHANNELS_OVERRIDE."""
        ch = [PWM_UNUSED_LOW] * 8 + [PWM_ABSENT] * (RC_CHANNELS - 8)
        ch[HT_MODE_CH - 1] = PWM_MAX if mode_on else PWM_MIN
        ch[HT_ROLL_CH - 1] = normalized_to_pwm(roll)
        ch[HT_PITCH_CH - 1] = normalized_to_pwm(pitch)
        m.mav.rc_channels_override_send(m.target_system, m.target_component, *ch)

    x = y = r = 0.0      # pitch, roll, yaw  (-1000..1000)
    z = 500.0            # throttle (0..1000); 500 = hold altitude in Position mode
    aux = [0, 0, 0, 0]   # index into AUX_POSITIONS for aux1..aux4

    ht_roll = ht_pitch = 0.0
    ht_mode_on = False
    # No RC_CHANNELS_OVERRIDE goes out until an HT key is pressed, so a session
    # that never uses HT leaves PX4's RC path exactly as it was.
    ht_streaming = False

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    period = 1.0 / RATE_HZ
    try:
        tty.setcbreak(fd)
        while True:
            t0 = time.time()
            drain(m)

            while select.select([sys.stdin], [], [], 0)[0]:
                c = sys.stdin.read(1)
                if c in ("q", "\x03"):
                    return
                elif c == "i":
                    x += STEP
                elif c == "k":
                    x -= STEP
                elif c == "j":
                    y -= STEP
                elif c == "l":
                    y += STEP
                elif c == "a":
                    r -= STEP
                elif c == "d":
                    r += STEP
                elif c == "w":
                    z += THR_STEP
                elif c == "s":
                    z -= THR_STEP
                elif c == " ":
                    x = y = r = 0.0
                    z = 500.0
                elif c == "m":
                    cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1.0)
                elif c == "n":
                    cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0.0)
                elif c == "t":
                    # NaN lat/lon/alt -> PX4 uses its own defaults (MIS_TAKEOFF_ALT)
                    cmd(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                        0.0, 0.0, 0.0, NAN, NAN, NAN, NAN)
                elif c == "g":
                    cmd(mavutil.mavlink.MAV_CMD_NAV_LAND,
                        0.0, 0.0, 0.0, NAN, NAN, NAN, NAN)
                elif c == "p":
                    set_mode(3)          # PX4 main mode 3 = Position
                elif c == "h":
                    set_mode(4, 3)       # Auto / Loiter
                elif c in ("5", "6", "7", "8"):
                    i = int(c) - 5
                    aux[i] = (aux[i] + 1) % len(AUX_POSITIONS)
                    print(f"\raux{i + 1} -> "
                          f"{AUX_LABEL[AUX_POSITIONS[aux[i]]]}" + " " * 30)
                elif c == "0":
                    aux = [0, 0, 0, 0]
                    print("\rall aux channels -> low" + " " * 30)
                elif c in ("e", "z", "x", "c", "v", "b"):
                    ht_streaming = True
                    if c == "e":
                        ht_mode_on = not ht_mode_on
                        print(f"\rHT mode (ch{HT_MODE_CH}) -> "
                              f"{'ON' if ht_mode_on else 'off'}" + " " * 30)
                    elif c == "z":
                        ht_roll -= HT_STEP
                    elif c == "x":
                        ht_roll += HT_STEP
                    elif c == "c":
                        ht_pitch -= HT_STEP
                    elif c == "v":
                        ht_pitch += HT_STEP
                    elif c == "b":
                        ht_roll = ht_pitch = 0.0

            x = clamp(x, -1000, 1000) * DECAY
            y = clamp(y, -1000, 1000) * DECAY
            r = clamp(r, -1000, 1000) * DECAY
            z = clamp(z, 0, 1000)

            ht_roll = clamp(ht_roll, -1.0, 1.0)
            ht_pitch = clamp(ht_pitch, -1.0, 1.0)

            a1, a2, a3, a4 = (AUX_POSITIONS[i] for i in aux)
            m.mav.manual_control_send(m.target_system,
                                      int(x), int(y), int(z), int(r), 0,
                                      buttons2=0,
                                      enabled_extensions=AUX_EXTENSIONS,
                                      s=0, t=0,
                                      aux1=a1, aux2=a2, aux3=a3, aux4=a4,
                                      aux5=0, aux6=0)

            if ht_streaming:
                send_ht(ht_mode_on, ht_roll, ht_pitch)

            ht_status = (f" | HT {'ON' if ht_mode_on else 'off'} "
                         f"r{ht_roll:+.2f} p{ht_pitch:+.2f}" if ht_streaming else "")
            aux_status = "".join(AUX_LABEL[v][0].upper() for v in (a1, a2, a3, a4))

            status = (f"p{int(x):+5d} r{int(y):+5d} t{int(z):4d} y{int(r):+5d}"
                      f" | aux {aux_status}{ht_status}")
            # A line wider than the terminal wraps, and \r then only rewinds the
            # last row, so every tick would scroll a fresh line. Never let it wrap.
            cols = shutil.get_terminal_size((80, 24)).columns
            sys.stdout.write("\r" + status[:cols - 1] + "\x1b[K")
            sys.stdout.flush()

            time.sleep(max(0.0, period - (time.time() - t0)))
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print()


if __name__ == "__main__":
    main()
