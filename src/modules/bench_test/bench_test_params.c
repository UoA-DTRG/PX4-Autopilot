/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * Bench test sub-mode
 *
 * Selects which excitation profile the bench_test module applies.
 *
 * @value 0 Hover only
 * @value 1 Step
 * @value 2 Ramp
 * @group Bench Test
 */
PARAM_DEFINE_INT32(BT_MODE, 0);

/**
 * Bench test axis under excitation
 *
 * Which channel the step / ramp is applied to.
 * Body-frame convention: X forward, Y right, Z down.
 * Horizontal thrust axes (X, Y) are only physically realisable on
 * fully-actuated / omni / tilt-rotor airframes; on a standard multicopter
 * the control allocator will drop them.
 *
 * @value 0 Thrust X (body forward)
 * @value 1 Thrust Y (body right)
 * @value 2 Thrust Z (body down, collective)
 * @value 3 Roll torque (about X)
 * @value 4 Pitch torque (about Y)
 * @value 5 Yaw torque (about Z)
 * @group Bench Test
 */
PARAM_DEFINE_INT32(BT_AXIS, 2);

/**
 * Bench test sign / direction
 *
 * Sign applied to the step and ramp output. +1 or -1.
 *
 * @value -1 Negative
 * @value 1 Positive
 * @group Bench Test
 */
PARAM_DEFINE_INT32(BT_SIGN, 1);

/**
 * Bench test hover thrust
 *
 * Baseline hover thrust magnitude (normalised). Published as -BT_HOVER_THR
 * in the Z body axis (NED convention: negative Z is up).
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group Bench Test
 */
PARAM_DEFINE_FLOAT(BT_HOVER_THR, 0.5f);

/**
 * Bench test step magnitude
 *
 * Absolute magnitude of the step applied to the chosen axis.
 * Units are normalised thrust (for BT_AXIS=0) or normalised torque.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @group Bench Test
 */
PARAM_DEFINE_FLOAT(BT_STEP_MAG, 0.1f);

/**
 * Bench test step delay
 *
 * Delay from mode activation before the step is applied. During this
 * window only hover thrust is commanded.
 *
 * @unit s
 * @min 0.0
 * @max 30.0
 * @decimal 1
 * @group Bench Test
 */
PARAM_DEFINE_FLOAT(BT_STEP_DELAY, 2.0f);

/**
 * Bench test step duration
 *
 * How long the step is held before returning to hover.
 *
 * @unit s
 * @min 0.1
 * @max 30.0
 * @decimal 1
 * @group Bench Test
 */
PARAM_DEFINE_FLOAT(BT_STEP_DUR, 1.0f);

/**
 * Bench test ramp rate
 *
 * Slope of the linear ramp applied to the chosen axis.
 *
 * @unit 1/s
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @group Bench Test
 */
PARAM_DEFINE_FLOAT(BT_RAMP_RATE, 0.05f);

/**
 * Bench test ramp maximum
 *
 * Absolute safety clamp for the ramp. The ramp normally increases until a
 * motor saturates (upper or lower), at which point it freezes and holds. This
 * value is the hard limit at which the ramp also stops and holds, in case
 * saturation is never reached.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @group Bench Test
 */
PARAM_DEFINE_FLOAT(BT_MAX_VAL, 0.3f);

/**
 * Bench test arm enable
 *
 * Safety gate. The module publishes zero thrust and torque unless this
 * parameter is set to 1. Even when the mode is selected and the vehicle
 * is armed, no motor commands are issued while this is 0.
 *
 * @boolean
 * @group Bench Test
 */
PARAM_DEFINE_INT32(BT_ARM_ENABLE, 0);

/**
 * Bench test start switch (RC channel)
 *
 * Selects the raw RC input channel (input_rc) used as the start switch for
 * the step and ramp profiles. This reads input_rc directly, so any channel
 * up to 18 is available (e.g. 16 for CH16), not just the mapped aux channels.
 * The switch is considered high when its pulse width exceeds 1500 us.
 *
 * While the switch is low the module only commands the hover baseline; the
 * profile runs while the switch is high, and the test clock is reset when the
 * switch returns low so each flip re-runs the profile from the start.
 *
 * Set to 0 to disable the switch gate and start the profile immediately on
 * mode entry (legacy behaviour).
 *
 * @min 0
 * @max 18
 * @group Bench Test
 */
PARAM_DEFINE_INT32(BT_START_SW, 0);

/**
 * Bench test number of motors
 *
 * Number of connected motors to inspect for saturation when deciding to
 * freeze the ramp. Motors occupy the first indices of the control allocator
 * saturation array, so only indices 0..N-1 are checked. This avoids unused
 * actuator slots incorrectly freezing the ramp.
 *
 * Set to 0 to auto-detect the count from the actuator_motors output (number
 * of finite motor channels).
 *
 * @min 0
 * @max 12
 * @group Bench Test
 */
PARAM_DEFINE_INT32(BT_NUM_MOTORS, 0);

/**
 * Bench test saturation margin
 *
 * Margin used to detect motor saturation for the ramp freeze, instead of the
 * hard 0 / 1 limits. A motor is considered saturated when its normalised
 * output rises above (1 - margin) or falls below (0 + margin). For example a
 * margin of 0.05 triggers at 0.95 (upper) and 0.05 (lower).
 *
 * For reversible motors the lower threshold is (-1 + margin).
 *
 * @min 0.0
 * @max 0.5
 * @decimal 3
 * @group Bench Test
 */
PARAM_DEFINE_FLOAT(BT_SAT_MARGIN, 0.05f);

/**
 * Bench test throttle spin-up time
 *
 * Duration over which the hover baseline thrust (BT_HOVER_THR) is ramped up
 * from zero when the outputs first become active (armed and BT_ARM_ENABLE=1).
 * This provides a soft-start so the motors spin up gradually instead of
 * jumping straight to the hover thrust.
 *
 * Set to 0 to apply the hover thrust immediately (no spin-up).
 *
 * @unit s
 * @min 0.0
 * @max 30.0
 * @decimal 1
 * @group Bench Test
 */
PARAM_DEFINE_FLOAT(BT_SPINUP_T, 2.0f);
