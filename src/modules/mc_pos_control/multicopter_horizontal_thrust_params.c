/**
 * @file multicopter_horizontal_thrust_params.c
 *
 * Parameters for DTRG horizontal thrust control (6DOF control).
 */

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
 * @group DTRG
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
 * @group DTRG
 */
PARAM_DEFINE_INT32(RC_MAP_HT_PITCH, 0);

/**
 * Horizontal thrust control mask
 *
 * How the vehicle moves along each horizontal axis while horizontal thrust
 * is switched on (RC_MAP_HT_MODE):
 * - 0: horizontal thrust on X and Y, the vehicle stays level
 * - 1: horizontal thrust on X, Y by rolling
 * - 2: horizontal thrust on Y, X by pitching
 * - 3: no horizontal thrust, X and Y by pitching and rolling
 *
 * @min 0
 * @max 3
 * @value 0 Horizontal thrust X and Y
 * @value 1 Horizontal thrust X, roll for Y
 * @value 2 Horizontal thrust Y, pitch for X
 * @value 3 Pitch and roll, no horizontal thrust
 * @group DTRG
 */
PARAM_DEFINE_INT32(DTRG_HT_MASK, 0);
