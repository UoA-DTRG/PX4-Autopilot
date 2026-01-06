/**
 * @file multicopter_horizontal_thrust_params.c
 *
 * Parameters for DTRG horizontal thrust control (6DOF control).
 */

/**
 * Horizontal thrust Roll channel
 *
 * AUX channel for horizontal thrust roll control in 6DOF mode.
 *
 * @min 1
 * @max 16
 * @value 10 AUX10
 * @value 11 AUX11
 * @value 12 AUX12
 * @group DTRG
 */
PARAM_DEFINE_INT32(DTRG_HT_R, 10);

/**
 * Horizontal thrust Pitch channel
 *
 * AUX channel for horizontal thrust pitch control in 6DOF mode.
 *
 * @min 1
 * @max 16
 * @value 10 AUX10
 * @value 11 AUX11
 * @value 12 AUX12
 * @group DTRG
 */
PARAM_DEFINE_INT32(DTRG_HT_P, 11);

/**
 * Horizontal thrust control mask
 *
 * Bitmask to enable horizontal thrust control axes:
 * - 0: Disabled
 * - 1: Roll only
 * - 2: Pitch only
 * - 3: Roll and Pitch
 *
 * @min 0
 * @max 3
 * @value 0 Disabled
 * @value 1 Roll only
 * @value 2 Pitch only
 * @value 3 Roll and Pitch
 * @group DTRG
 */
PARAM_DEFINE_INT32(DTRG_HT_MASK, 0);
