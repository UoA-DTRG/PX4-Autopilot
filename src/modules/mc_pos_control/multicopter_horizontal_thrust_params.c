/**
 * @file multicopter_horizontal_thrust_params.c
 *
 * Parameters for DTRG horizontal thrust control (6DOF control).
 */

/**
 * DTRG Horizontal Thrust Enable
 *
 *
 *
 * Enable the horizontal thrust control feature for manual, position, and offboard flight modes.
 * HT mode is still dependent on the HT thrust Enable Channel.
 *
 * @reboot_required true
 * @boolean
 * @group DTRG Horizontal Thrust
 */
PARAM_DEFINE_INT32(DTRG_HT_EN, 0);

/**
 * DTRG Horizontal Thrust Enable Channel
 *
 *
 *
 * Raw RC channel (input_rc) that enables horizontal thrust control while it reads high.
 * Disabled by default; if you would like to have the UAV fly only in HT mode then this
 * can be set to the arming channel.
 *
 * The channel must not be used for any other function - arming is blocked while it is
 * also assigned to another RC_MAP parameter - and it must be correctly configured in the radio.
 *
 * @min 0
 * @max 16
 * @value 0 Disabled
 * @value 5 Channel 5
 * @value 6 Channel 6
 * @value 7 Channel 7
 * @value 8 Channel 8
 * @value 9 Channel 9
 * @value 10 Channel 10
 * @value 11 Channel 11
 * @value 12 Channel 12
 * @value 13 Channel 13
 * @value 14 Channel 14
 * @value 15 Channel 15
 * @value 16 Channel 16
 * @reboot_required true
 * @group DTRG Horizontal Thrust
 */
PARAM_DEFINE_INT32(RC_MAP_HT_MODE, 0);

/**
 * Horizontal Thrust XY Limit
 *
 * Saturation limit of the commanded horizontal thrust.
 *
 * @min 0.000
 * @max 1.000
 * @decimal 3
 * @group DTRG Horizontal Thrust
 */
PARAM_DEFINE_FLOAT(DTRG_HT_MAX, 0.500f);

/**
 * Horizontal Thrust Roll Angle Limit (degrees)
 *
 * Saturation limit of the commanded horizontal thrust.
 *
 * @unit deg
 * @min 0.000
 * @max 45.000
 * @decimal 1
 * @group DTRG Horizontal Thrust
 */
PARAM_DEFINE_FLOAT(DTRG_HT_R_MAX, 10.0f);

/**
 * Horizontal Thrust Pitch Angle Limit (degrees)
 *
 * Saturation limit of the commanded horizontal thrust.
 *
 * @unit deg
 * @min 0.000
 * @max 45.000
 * @decimal 1
 * @group DTRG Horizontal Thrust
 */
PARAM_DEFINE_FLOAT(DTRG_HT_P_MAX, 10.0f);

/**
 * Horizontal thrust Roll channel
 *
 * Raw RC channel (input_rc) for horizontal thrust roll control in 6DOF mode.
 *
 * @min 0
 * @max 16
 * @value 0 Disabled
 * @value 5 Channel 5
 * @value 6 Channel 6
 * @value 7 Channel 7
 * @value 8 Channel 8
 * @value 9 Channel 9
 * @value 10 Channel 10
 * @value 11 Channel 11
 * @value 12 Channel 12
 * @value 13 Channel 13
 * @value 14 Channel 14
 * @value 15 Channel 15
 * @value 16 Channel 16
 * @group DTRG Horizontal Thrust
 */
PARAM_DEFINE_INT32(RC_MAP_HT_ROLL, 0);

/**
 * Horizontal thrust Pitch channel
 *
 * Raw RC channel (input_rc) for horizontal thrust pitch control in 6DOF mode.
 *
 * @min 0
 * @max 16
 * @value 0 Disabled
 * @value 5 Channel 5
 * @value 6 Channel 6
 * @value 7 Channel 7
 * @value 8 Channel 8
 * @value 9 Channel 9
 * @value 10 Channel 10
 * @value 11 Channel 11
 * @value 12 Channel 12
 * @value 13 Channel 13
 * @value 14 Channel 14
 * @value 15 Channel 15
 * @value 16 Channel 16
 * @group DTRG Horizontal Thrust
 */
PARAM_DEFINE_INT32(RC_MAP_HT_PITCH, 0);

/**
 * Horizontal thrust axes
 *
 * The axes moved by horizontal thrust (HT axes) while horizontal thrust
 * is switched on (RC_MAP_HT_MODE). The other axis moves by tilting.
 * - 0: horizontal thrust on X and Y
 * - 1: horizontal thrust on X, Y by rolling
 * - 2: horizontal thrust on Y, X by pitching
 *
 * DTRG_HT_SPLIT_EN selects whether the HT axes also move by tilting.
 *
 * @min 0
 * @max 2
 * @value 0 Horizontal thrust X and Y
 * @value 1 Horizontal thrust X, roll for Y
 * @value 2 Horizontal thrust Y, pitch for X
 * @group DTRG Horizontal Thrust
 */
PARAM_DEFINE_INT32(DTRG_HT_MASK, 0);

/**
 * Split horizontal thrust axes between horizontal thrust and tilt
 *
 * Disabled: the axes selected by DTRG_HT_MASK move by horizontal thrust
 * only. Their tilt comes from the aux tilt channels (RC_MAP_HT_ROLL /
 * RC_MAP_HT_PITCH) or the offboard HT attitude, level when unassigned.
 *
 * Enabled: those axes move by horizontal thrust and by tilting, shared by
 * DTRG_HT_SPLIT. The aux tilt channels and the offboard HT attitude are
 * not used on those axes.
 *
 * @boolean
 * @group DTRG Horizontal Thrust
 */
PARAM_DEFINE_INT32(DTRG_HT_SPLIT_EN, 0);

/**
 * Horizontal thrust split
 *
 * With DTRG_HT_SPLIT_EN, the share of the movement on the axes selected by
 * DTRG_HT_MASK produced by horizontal thrust; the rest is produced by
 * tilting.
 * - 0: tilt only, no horizontal thrust
 * - 1: horizontal thrust only, no tilt
 *
 * Position/Offboard: the vehicle tilts for (1 - split) of the position
 * controller's thrust along those axes and horizontal thrust produces the rest.
 * Stabilized: on those axes the sticks command (1 - split) of the maximum
 * tilt (MPC_MAN_TILT_MAX) and split of the maximum horizontal thrust
 * (DTRG_HT_MAX).
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group DTRG Horizontal Thrust
 */
PARAM_DEFINE_FLOAT(DTRG_HT_SPLIT, 0.5f);
