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
 * Absolute clamp value at which the ramp stops and holds.
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
