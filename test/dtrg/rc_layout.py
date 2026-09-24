"""Standard RC layout for every DTRG SITL test that streams RC.

One layout for all tests, so that no two functions ever share a channel (which
the RC channel conflict check would refuse to arm with). Channels 7 and 11-18
are left unassigned.
"""

from __future__ import annotations

PWM_MIN = 1000
PWM_CENTRE = 1500
PWM_MAX = 2000

CH_ROLL = 1
CH_PITCH = 2
CH_THROTTLE = 3
CH_YAW = 4
CH_FLTMODE = 5
CH_CMD_SIGN = 6  # bench test direction switch
CH_HT_MODE = 8  # horizontal thrust on/off
CH_HT_ROLL = 9
CH_HT_PITCH = 10

# COM_FLTMODEx values (see src/modules/commander/module.yaml), not nav states
FLTMODE_POSITION = 2
FLTMODE_STABILIZED = 8
FLTMODE_BENCH_TEST = 16

SLOT_STABILIZED = 1
SLOT_POSITION = 4
SLOT_BENCH_TEST = 6

RC_PARAMS = {
    # RC only, so the RC flight mode switch is used and RC loss is detected
    "COM_RC_IN_MODE": 0,
    # rc_update only marks manual control valid once RC is "calibrated" (RC_CHAN_CNT > 0)
    "RC_CHAN_CNT": 18,
    "RC_MAP_ROLL": CH_ROLL,
    "RC_MAP_PITCH": CH_PITCH,
    "RC_MAP_THROTTLE": CH_THROTTLE,
    "RC_MAP_YAW": CH_YAW,
    "RC_MAP_FLTMODE": CH_FLTMODE,
    "RC_MAP_CMD_SIGN": CH_CMD_SIGN,
    "RC_MAP_HT_MODE": CH_HT_MODE,
    "RC_MAP_HT_ROLL": CH_HT_ROLL,
    "RC_MAP_HT_PITCH": CH_HT_PITCH,
    "COM_FLTMODE1": FLTMODE_STABILIZED,
    "COM_FLTMODE2": -1,
    "COM_FLTMODE3": -1,
    "COM_FLTMODE4": FLTMODE_POSITION,
    "COM_FLTMODE5": -1,
    "COM_FLTMODE6": FLTMODE_BENCH_TEST,
}

NUM_MODE_SLOTS = 6

def deadzone(channel: int) -> int:
    """RCx_DZ default [us]: 10 on channels 1-8, 0 on 9-18 (src/modules/rc_update/params.c)."""
    return 10 if channel <= 8 else 0


def normalized(pwm: int, channel: int) -> float:
    """rc_channels value for ``pwm`` with the default calibration (rc_update.cpp: min, trim +- dz, max)."""
    dz = deadzone(channel)
    if pwm >= PWM_CENTRE + dz:
        return min(1.0, (pwm - PWM_CENTRE - dz) / (PWM_MAX - PWM_CENTRE - dz))
    if pwm <= PWM_CENTRE - dz:
        return max(-1.0, (pwm - PWM_CENTRE + dz) / (PWM_CENTRE - dz - PWM_MIN))
    return 0.0


def mode_slot_from_pwm(pwm: int) -> int:
    """Mirror of the slot decoding in src/modules/rc_update/rc_update.cpp (RC_MAP_FLTMODE)."""
    value = normalized(pwm, CH_FLTMODE)
    slot_width_half = 1.0 / NUM_MODE_SLOTS
    slot_min = -1.0 - 0.05
    slot_max = 1.0 + 0.05
    slot = int((((value - slot_min) * NUM_MODE_SLOTS) + slot_width_half) / (slot_max - slot_min) + slot_width_half) + 1
    return min(slot, NUM_MODE_SLOTS)


def slot_pwm(slot: int) -> int:
    """PWM in the middle of the band that rc_update decodes as ``slot`` (1-based)."""
    if not 1 <= slot <= NUM_MODE_SLOTS:
        raise ValueError(f"slot must be 1-{NUM_MODE_SLOTS}, got {slot}")
    band = [pwm for pwm in range(PWM_MIN, PWM_MAX + 1) if mode_slot_from_pwm(pwm) == slot]
    return band[len(band) // 2]


def initial_channels(overrides: dict[int, int] | None = None) -> dict[int, int]:
    """Safe starting RC: sticks centred, throttle low, Stabilized slot, switches off."""
    channels = {
        CH_ROLL: PWM_CENTRE,
        CH_PITCH: PWM_CENTRE,
        CH_THROTTLE: PWM_MIN,
        CH_YAW: PWM_CENTRE,
        CH_FLTMODE: slot_pwm(SLOT_STABILIZED),
        CH_CMD_SIGN: PWM_CENTRE,
        CH_HT_MODE: PWM_MIN,
        CH_HT_ROLL: PWM_CENTRE,
        CH_HT_PITCH: PWM_CENTRE,
    }
    channels.update(overrides or {})
    return channels
