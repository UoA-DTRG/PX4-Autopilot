#!/usr/bin/env python3
"""Send RC_CHANNELS_OVERRIDE to a PX4 SITL instance.

SITL has no RC receiver, so `input_rc` is never published and `rc_update` never
produces `rc_channels`. The DTRG horizontal-thrust code in mc_pos_control reads
`rc_channels` directly (RC_MAP_HT_MODE / RC_MAP_HT_ROLL / RC_MAP_HT_PITCH), so
without a channel source the feature stays inert in simulation.

This script closes that gap: MAVLink RC_CHANNELS_OVERRIDE is turned into
`input_rc` by the mavlink receiver, and `rc_update` then publishes `rc_channels`
with the usual RC<n>_MIN/TRIM/MAX calibration applied (1000/1500/2000 by
default, so PWM 1500 is 0.0 and 2000 is +1.0).

Examples
--------
Hold the mode switch high and command a steady +0.5 roll on channel 6:

    Tools/dtrg/rc_override.py --set 5=2000 --set 6=1750

The same using normalized stick units:

    Tools/dtrg/rc_override.py --norm 5=1 --norm 6=0.5

Fly it by hand (arrow keys move roll/pitch, m toggles the mode switch):

    Tools/dtrg/rc_override.py --interactive --mode-ch 5 --roll-ch 6 --pitch-ch 7

PX4 side, once per session (or as airframe defaults):

    param set DTRG_HT_EN 1
    param set RC_MAP_HT_MODE 8
    param set RC_MAP_HT_ROLL 9
    param set RC_MAP_HT_PITCH 10
    param set DTRG_HT_MASK 3

Gotchas
-------
1. Do not reuse a channel that any other RC_MAP_* already claims. The DTRG
   preflight check rejects arming with "RC_MAP_X and RC_MAP_Y both use RC
   channel N". `param show RC_MAP_*` lists what is taken; a SITL rootfs that
   has ever had a real calibration loaded usually has channels 1-7 claimed.

2. A channel mapped to a switch reads ON when it sits at centre. PX4 rescales
   the -1..1 channel to 0..1 before comparing against RC_KILLSWITCH_TH and
   friends (0.25 by default), so a centred channel gives 0.5, which is above
   the threshold. Leaving everything at trim therefore engages the kill switch
   and blocks arming. Drive those channels low explicitly:

       --norm 6=-1 --norm 7=-1    # RC_MAP_ARM_SW, RC_MAP_KILL_SW

3. --pwm-min/--pwm-trim/--pwm-max apply to every channel, but PX4 calibrates
   per channel. If RC1-8 carry a saved transmitter calibration and RC9-18 are
   still at PX4 defaults, no single set of three values is right for all of
   them. Use --set with raw PWM when the exact value matters.
"""

from __future__ import annotations

import argparse
import os
import sys
import termios
import time
import tty

# Channels 9-18 only exist as MAVLink 2 extension fields; pymavlink picks the
# protocol version at import time, so this has to come first.
os.environ.setdefault("MAVLINK20", "1")

from pymavlink import mavutil  # noqa: E402

CHANNEL_COUNT = 18

# PX4 maps raw PWM to the -1..1 values in `rc_channels` with the per-channel
# RC<n>_MIN / RC<n>_TRIM / RC<n>_MAX calibration. These defaults are PX4's own
# defaults; if your RC<n>_* differ (a calibration saved from a real transmitter
# often does), pass --pwm-min/--pwm-trim/--pwm-max to match, or use --set to
# give raw PWM directly. Check with: build/px4_sitl_default/bin/px4-param show RC5_TRIM
PWM_MIN = 1000
PWM_TRIM = 1500
PWM_MAX = 2000


def normalized_to_pwm(value: float) -> int:
    value = max(-1.0, min(1.0, value))
    span = (PWM_MAX - PWM_TRIM) if value >= 0 else (PWM_TRIM - PWM_MIN)
    return int(round(PWM_TRIM + value * span))


def pwm_to_normalized(pwm: int) -> float:
    span = (PWM_MAX - PWM_TRIM) if pwm >= PWM_TRIM else (PWM_TRIM - PWM_MIN)
    return (pwm - PWM_TRIM) / span


def parse_assignment(text: str, converter) -> tuple[int, float]:
    if "=" not in text:
        raise argparse.ArgumentTypeError(f"expected CHANNEL=VALUE, got {text!r}")
    channel, _, value = text.partition("=")
    try:
        channel = int(channel)
    except ValueError:
        raise argparse.ArgumentTypeError(f"channel must be an integer, got {channel!r}") from None
    if not 1 <= channel <= CHANNEL_COUNT:
        raise argparse.ArgumentTypeError(f"channel must be 1-{CHANNEL_COUNT}, got {channel}")
    try:
        return channel, converter(value)
    except ValueError:
        raise argparse.ArgumentTypeError(f"bad value {value!r} for channel {channel}") from None


def raw_assignment(text: str) -> tuple[int, int]:
    channel, value = parse_assignment(text, int)
    if not 800 <= value <= 2200:
        raise argparse.ArgumentTypeError(f"PWM must be 800-2200, got {value}")
    return channel, int(value)


def norm_assignment(text: str) -> tuple[int, int]:
    channel, value = parse_assignment(text, float)
    return channel, normalized_to_pwm(value)


def send(master, channels: list[int]) -> None:
    master.mav.rc_channels_override_send(master.target_system, master.target_component, *channels)


def run_static(master, channels: list[int], rate_hz: float, seconds: float | None) -> None:
    period = 1.0 / rate_hz
    deadline = None if seconds is None else time.monotonic() + seconds
    active = {i + 1: pwm for i, pwm in enumerate(channels) if pwm != PWM_TRIM}
    summary = ", ".join(f"ch{c}={pwm} ({pwm_to_normalized(pwm):+.2f})" for c, pwm in sorted(active.items()))
    print(f"Sending RC_CHANNELS_OVERRIDE at {rate_hz:g} Hz: {summary or 'all channels centred'}", flush=True)
    print("Ctrl-C to stop.", flush=True)
    while deadline is None or time.monotonic() < deadline:
        send(master, channels)
        time.sleep(period)


def run_interactive(master, channels: list[int], rate_hz: float, mode_ch: int,
                    roll_ch: int, pitch_ch: int, step: float) -> None:
    roll = pitch = 0.0
    mode_on = False
    period = 1.0 / rate_hz
    stdin = sys.stdin.fileno()
    saved = termios.tcgetattr(stdin)
    print(f"Interactive: arrows/WASD move roll(ch{roll_ch})/pitch(ch{pitch_ch}), "
          f"m toggles mode(ch{mode_ch}), 0 centres, q quits.")
    try:
        tty.setcbreak(stdin)
        last_draw = 0.0
        while True:
            key = read_key(stdin, period)
            if key in ("q", "\x03"):
                break
            if key in ("a", "LEFT"):
                roll -= step
            elif key in ("d", "RIGHT"):
                roll += step
            elif key in ("w", "UP"):
                pitch += step
            elif key in ("s", "DOWN"):
                pitch -= step
            elif key == "m":
                mode_on = not mode_on
            elif key == "0":
                roll = pitch = 0.0

            roll = max(-1.0, min(1.0, roll))
            pitch = max(-1.0, min(1.0, pitch))
            channels[mode_ch - 1] = PWM_MAX if mode_on else PWM_MIN
            channels[roll_ch - 1] = normalized_to_pwm(roll)
            channels[pitch_ch - 1] = normalized_to_pwm(pitch)
            send(master, channels)

            now = time.monotonic()
            if now - last_draw > 0.1:
                sys.stdout.write(f"\rmode {'ON ' if mode_on else 'off'}  "
                                 f"roll {roll:+.2f}  pitch {pitch:+.2f}   ")
                sys.stdout.flush()
                last_draw = now
    finally:
        termios.tcsetattr(stdin, termios.TCSADRAIN, saved)
        print()


def read_key(stdin: int, timeout: float) -> str | None:
    import select

    ready, _, _ = select.select([stdin], [], [], timeout)
    if not ready:
        return None
    char = sys.stdin.read(1)
    if char != "\x1b":
        return char
    # Arrow keys arrive as ESC [ A..D; consume the rest without blocking.
    ready, _, _ = select.select([stdin], [], [], 0.01)
    if not ready:
        return char
    sys.stdin.read(1)
    ready, _, _ = select.select([stdin], [], [], 0.01)
    if not ready:
        return char
    return {"A": "UP", "B": "DOWN", "C": "RIGHT", "D": "LEFT"}.get(sys.stdin.read(1), char)


def main() -> None:
    global PWM_MIN, PWM_TRIM, PWM_MAX

    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--url", default="udpout:127.0.0.1:14580",
                        help="MAVLink endpoint (default: %(default)s, the SITL API link)")
    parser.add_argument("--rate", type=float, default=50.0, help="send rate in Hz (default: %(default)s)")
    parser.add_argument("--seconds", type=float, help="stop after this long (default: run until Ctrl-C)")
    parser.add_argument("--set", dest="raw", action="append", default=[], type=raw_assignment,
                        metavar="CH=PWM", help="set a channel to a raw PWM value, 1000-2000")
    parser.add_argument("--norm", action="append", default=[], type=norm_assignment,
                        metavar="CH=VALUE", help="set a channel from normalized stick units, -1 to 1")
    parser.add_argument("--interactive", action="store_true", help="drive roll/pitch from the keyboard")
    parser.add_argument("--mode-ch", type=int, default=5, help="interactive mode switch channel")
    parser.add_argument("--roll-ch", type=int, default=6, help="interactive roll channel")
    parser.add_argument("--pitch-ch", type=int, default=7, help="interactive pitch channel")
    parser.add_argument("--step", type=float, default=0.05, help="interactive step per key press")
    parser.add_argument("--target-system", type=int, default=1, help="MAVLink target system id")
    parser.add_argument("--pwm-min", type=int, default=PWM_MIN, help="RC<n>_MIN (default: %(default)s)")
    parser.add_argument("--pwm-trim", type=int, default=PWM_TRIM, help="RC<n>_TRIM (default: %(default)s)")
    parser.add_argument("--pwm-max", type=int, default=PWM_MAX, help="RC<n>_MAX (default: %(default)s)")
    args, _ = parser.parse_known_args()

    # --norm and the interactive mode convert through the calibration, so it has
    # to be in place before argparse runs the type converters for real.
    PWM_MIN, PWM_TRIM, PWM_MAX = args.pwm_min, args.pwm_trim, args.pwm_max
    args = parser.parse_args()

    channels = [PWM_TRIM] * CHANNEL_COUNT
    for channel, pwm in args.raw + args.norm:
        channels[channel - 1] = pwm

    master = mavutil.mavlink_connection(args.url, source_system=255, source_component=0)
    master.target_system = args.target_system
    master.target_component = 0
    print(f"Connected to {args.url} (target system {args.target_system}).", flush=True)

    try:
        if args.interactive:
            run_interactive(master, channels, args.rate, args.mode_ch,
                            args.roll_ch, args.pitch_ch, args.step)
        else:
            run_static(master, channels, args.rate, args.seconds)
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
