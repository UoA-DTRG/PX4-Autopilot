/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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
 * @file bench_test_params.c
 *
 * Parameters for the bench_test module.
 * Parameter names: max 16 chars. Prefix: BT_
 */

#include <parameters/param.h>

/**
 * Number of motors to test
 *
 * Total number of motors on the vehicle to include in bench tests.
 *
 * @min 1
 * @max 8
 * @group Bench Test
 */
PARAM_DEFINE_INT32(BT_NUM_MOTORS, 8);

/**
 * Background motor throttle level (step_idle test)
 *
 * Normalized throttle applied to all non-target motors during
 * the step_idle test. Can be set to any operating point, e.g.
 * 0.0 for stopped, 0.10 for idle, or 0.35 for high-throttle
 * cruise to test under realistic loaded conditions.
 * 0.0 = stopped, 1.0 = full throttle.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Bench Test
 */
PARAM_DEFINE_FLOAT(BT_BG_LVL, 0.10f);

/**
 * Background motor settle time (step_idle test)
 *
 * Time in milliseconds to hold all background motors at BT_BG_LVL
 * before triggering the step on the target motor. Allows ESCs and
 * airflow to stabilise before the excitation.
 *
 * @unit ms
 * @min 500
 * @max 30000
 * @group Bench Test
 */
PARAM_DEFINE_INT32(BT_BG_SETL, 5000);

/**
 * Baseline throttle for step and impulse tests
 *
 * Normalized throttle at which all motors are held before and
 * between step/impulse excitations. Set to 0.0 for tests from
 * standstill, or to e.g. 0.10 to test from a running idle speed.
 * The step/impulse level is applied on top of this baseline.
 * 0.0 = stopped, 1.0 = full throttle.
 *
 * @min 0.0
 * @max 0.8
 * @decimal 2
 * @increment 0.01
 * @group Bench Test
 */
PARAM_DEFINE_FLOAT(BT_BASE_LVL, 0.00f);

/**
 * Step test throttle level
 *
 * Normalized throttle output value for single motor step tests.
 * 0.0 = minimum, 1.0 = full throttle.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group Bench Test
 */
PARAM_DEFINE_FLOAT(BT_STEP_LVL, 0.15f);

/**
 * Step test hold time
 *
 * Duration in milliseconds to hold the motor at the step level
 * before returning to zero.
 *
 * @unit ms
 * @min 100
 * @max 10000
 * @group Bench Test
 */
PARAM_DEFINE_INT32(BT_STEP_DUR, 1000);

/**
 * Impulse test throttle level
 *
 * Peak normalized throttle for the impulse (short burst) test.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group Bench Test
 */
PARAM_DEFINE_FLOAT(BT_IMP_LVL, 0.30f);

/**
 * Impulse test pulse time
 *
 * Duration in milliseconds of the impulse burst.
 *
 * @unit ms
 * @min 20
 * @max 2000
 * @group Bench Test
 */
PARAM_DEFINE_INT32(BT_IMP_DUR, 150);

/**
 * Chirp/tweet start frequency
 *
 * Starting frequency in Hz for the chirp (tweet) excitation test.
 * The motor output is modulated sinusoidally from this frequency
 * up to BT_TWE_F_END over the test duration.
 *
 * @unit Hz
 * @min 1.0
 * @max 200.0
 * @decimal 1
 * @group Bench Test
 */
PARAM_DEFINE_FLOAT(BT_TWE_F_STA, 5.0f);

/**
 * Chirp/tweet end frequency
 *
 * Ending frequency in Hz for the chirp (tweet) excitation test.
 *
 * @unit Hz
 * @min 1.0
 * @max 500.0
 * @decimal 1
 * @group Bench Test
 */
PARAM_DEFINE_FLOAT(BT_TWE_F_END, 80.0f);

/**
 * Chirp/tweet amplitude
 *
 * Normalized peak-to-peak amplitude of the sinusoidal chirp
 * excitation around the bias level. The motor output oscillates
 * in the range [bias - amp, bias + amp].
 *
 * @min 0.0
 * @max 0.5
 * @decimal 2
 * @increment 0.01
 * @group Bench Test
 */
PARAM_DEFINE_FLOAT(BT_TWE_AMP, 0.10f);

/**
 * Chirp/tweet bias level
 *
 * DC bias throttle around which the chirp oscillates.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group Bench Test
 */
PARAM_DEFINE_FLOAT(BT_TWE_BIAS, 0.20f);

/**
 * Chirp/tweet duration
 *
 * Total duration in milliseconds of the chirp sweep.
 *
 * @unit ms
 * @min 500
 * @max 30000
 * @group Bench Test
 */
PARAM_DEFINE_INT32(BT_TWE_DUR, 5000);

/**
 * Multi-motor test throttle
 *
 * Normalized throttle output for the all-motors simultaneous test.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group Bench Test
 */
PARAM_DEFINE_FLOAT(BT_MULTI_LVL, 0.15f);

/**
 * Multi-motor test duration
 *
 * Duration in milliseconds for the all-motors simultaneous test.
 *
 * @unit ms
 * @min 100
 * @max 10000
 * @group Bench Test
 */
PARAM_DEFINE_INT32(BT_MULTI_DUR, 2000);

/**
 * Current-sweep start level
 *
 * Normalized throttle at which the current-draw sweep begins.
 * The motor is stepped through increasing throttle levels from
 * this value to BT_ISWEEP_END.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group Bench Test
 */
PARAM_DEFINE_FLOAT(BT_ISWEEP_STA, 0.05f);

/**
 * Current-sweep end level
 *
 * Normalized throttle at which the current-draw sweep ends.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group Bench Test
 */
PARAM_DEFINE_FLOAT(BT_ISWEEP_END, 0.60f);

/**
 * Current-sweep number of steps
 *
 * Number of discrete throttle levels between start and end
 * during the current-draw sweep test.
 *
 * @min 2
 * @max 20
 * @group Bench Test
 */
PARAM_DEFINE_INT32(BT_ISWEEP_N, 6);

/**
 * Current-sweep dwell time
 *
 * Time in milliseconds to hold each throttle level during
 * the current-draw sweep.
 *
 * @unit ms
 * @min 200
 * @max 10000
 * @group Bench Test
 */
PARAM_DEFINE_INT32(BT_ISWEEP_DWL, 2000);

/**
 * Ramp-up time
 *
 * Time in milliseconds to linearly ramp the motor from zero
 * to the target throttle at the start of each test. Prevents
 * instantaneous step that may trip ESC current limiting.
 *
 * @unit ms
 * @min 0
 * @max 2000
 * @group Bench Test
 */
PARAM_DEFINE_INT32(BT_RAMP_TIME, 200);

/**
 * Inter-motor delay
 *
 * Delay in milliseconds between sequential single-motor tests
 * when iterating through all motors (e.g. step_all, impulse_all).
 *
 * @unit ms
 * @min 100
 * @max 5000
 * @group Bench Test
 */
PARAM_DEFINE_INT32(BT_INTER_DLY, 500);

/* ════════════════════════════════════════════════════════════════════════
 *  Flight test parameters
 * ════════════════════════════════════════════════════════════════════════ */

/**
 * Flight test hover throttle
 *
 * Normalized throttle applied to all motors during the simulated
 * hover phase of the flight test (takeoff ramp target and hold level).
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group Bench Test
 */
PARAM_DEFINE_FLOAT(BT_FLT_HOVR, 0.35f);

/**
 * Flight test hover settle time
 *
 * Time in milliseconds to hold steady hover throttle before
 * triggering the multisine excitation.
 *
 * @unit ms
 * @min 500
 * @max 30000
 * @group Bench Test
 */
PARAM_DEFINE_INT32(BT_FLT_HOVT, 3000);

/**
 * Flight test blip count
 *
 * Number of thrust blips to perform after the multisine
 * excitation phase completes.
 *
 * @min 1
 * @max 50
 * @group Bench Test
 */
PARAM_DEFINE_INT32(BT_FLT_BLPN, 5);

/**
 * Flight test blip amplitude
 *
 * Normalized throttle level for each thrust blip.
 * Added on top of the hover level for the blip duration.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group Bench Test
 */
PARAM_DEFINE_FLOAT(BT_FLT_BLPA, 0.20f);

/**
 * Flight test blip duration
 *
 * Duration in milliseconds of each thrust blip.
 *
 * @unit ms
 * @min 50
 * @max 5000
 * @group Bench Test
 */
PARAM_DEFINE_INT32(BT_FLT_BLPD, 300);

/**
 * Flight test blip interval
 *
 * Time in milliseconds between successive thrust blips (up or down).
 * During this interval the motors hold the hover throttle level to
 * allow the vehicle to stabilise before the next impulse.
 *
 * @unit ms
 * @min 200
 * @max 30000
 * @group Bench Test
 */
PARAM_DEFINE_INT32(BT_FLT_BLPI, 2000);

/**
 * Flight test impulse stop state of charge
 *
 * When set > 0, Phase 6 (impulse pairs) continues repeating until the
 * battery state of charge drops to or below this percentage, using
 * BT_FLT_BLPN only as a hard maximum count cap.
 * When set to 0 (default), only BT_FLT_BLPN pairs are executed.
 * Example: set to 20 to stop when 20% SoC remaining.
 * BT_FLT_VMIN still applies as a hard voltage safety cutoff.
 *
 * @unit %
 * @min 0
 * @max 90
 * @group Bench Test
 */
PARAM_DEFINE_FLOAT(BT_FLT_BLPS, 0.0f);

/**
 * Flight test high throttle level (step-up target)
 *
 * Normalized throttle for the slow step-up phase of the flight test.
 * All motors ramp to this level, hold for BT_FLT_STPH ms, then ramp
 * back to hover throttle.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group Bench Test
 */
PARAM_DEFINE_FLOAT(BT_FLT_HIHI, 0.60f);

/**
 * Flight test low throttle level (step-down target)
 *
 * Normalized throttle for the slow step-down phase of the flight test.
 * All motors ramp to this level, hold for BT_FLT_STPH ms, then ramp
 * back to hover throttle.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group Bench Test
 */
PARAM_DEFINE_FLOAT(BT_FLT_LOLO, 0.15f);

/**
 * Flight test step hold time
 *
 * Duration in milliseconds to hold the high or low throttle level
 * during the slow step phases of the flight test.
 *
 * @unit ms
 * @min 1000
 * @max 60000
 * @group Bench Test
 */
PARAM_DEFINE_INT32(BT_FLT_STPH, 10000);

/**
 * Flight test step ramp time
 *
 * Time in milliseconds to ramp from hover to the step level (and back).
 * Intentionally slower than BT_RAMP_TIME to simulate a realistic
 * steady-state operating-point change.
 *
 * @unit ms
 * @min 200
 * @max 10000
 * @group Bench Test
 */
PARAM_DEFINE_INT32(BT_FLT_STPR, 1000);

/**
 * Flight test minimum pack voltage cutoff
 *
 * If the battery pack voltage drops below this value during a flight test
 * the test is immediately aborted and all motors are ramped to zero.
 * Set to 0.0 to disable the check.
 *
 * @unit V
 * @min 0.0
 * @max 60.0
 * @decimal 1
 * @group Bench Test
 */
PARAM_DEFINE_FLOAT(BT_FLT_VMIN, 0.0f);

/* ════════════════════════════════════════════════════════════════════════
 *  Voltage compensator parameters
 *
 *  The compensator adjusts the motor command to counteract battery
 *  voltage sag.  It predicts the terminal voltage using a first-order
 *  RC battery model whose parameters are polynomials in SoC, then
 *  scales δ by Vb_op / Vb_predicted.
 *
 *  Speed estimator (per rotor, shared coefficients):
 *    ω_k = (θ_ω1·Vδ + θ_ω2·√Vδ + θ_ω3 − θ_ω4·ω_{k-1}) / (1 + θ_ω4)
 *
 *  Current estimator (total, shared):
 *    I = Σ (θ_I1·ω_i + θ_I2·ω_i² + θ_I3)
 *
 *  Battery model:
 *    V0(s)  = c0 + c1·s + c2·s² + c3·s³       (OCV polynomial in SoC s)
 *    R0(s)  = c0 + c1·s + c2·s² + c3·s³       (series resistance)
 *    R1(s)  = c0 + c1·s + c2·s² + c3·s³       (RC branch resistance)
 *    τ1(s)  = c0 + c1·s + c2·s² + c3·s³       (RC time constant)
 *    V_RC_k = V_RC_{k-1} + Ts·(R1/τ1·I − 1/τ1·V_RC_{k-1})
 *    Vb     = V0 − I·R0 − V_RC
 *    C_δ    = Vb_op / Vb
 *    δ'     = C_δ · δ
 * ════════════════════════════════════════════════════════════════════════ */

/**
 * Voltage compensator nominal battery voltage
 *
 * The assumed "operating point" battery voltage Vb_op.
 * The compensator scales the command so that the effective voltage
 * δ·Vb equals δ'·Vb_actual, keeping rotor speed independent of
 * battery state.  Typically the mid-SoC pack voltage.
 * Set to 0.0 to leave the compensator unconfigured.
 *
 * @unit V
 * @min 0.0
 * @max 60.0
 * @decimal 2
 * @group Bench Test
 */
PARAM_DEFINE_FLOAT(BT_VC_VBOP, 0.0f);

/**
 * Speed estimator coefficient θ_ω1
 *
 * Linear voltage term in the rotor speed estimator.
 * ω_k = (θ1·Vδ + θ2·√Vδ + θ3 − θ4·ω_{k-1}) / (1 + θ4)
 *
 * @decimal 4
 * @group Bench Test
 */
PARAM_DEFINE_FLOAT(BT_VC_TW1, 0.0f);

/**
 * Speed estimator coefficient θ_ω2
 *
 * Square-root voltage term in the rotor speed estimator.
 *
 * @decimal 4
 * @group Bench Test
 */
PARAM_DEFINE_FLOAT(BT_VC_TW2, 0.0f);

/**
 * Speed estimator coefficient θ_ω3
 *
 * Constant term in the rotor speed estimator.
 *
 * @decimal 4
 * @group Bench Test
 */
PARAM_DEFINE_FLOAT(BT_VC_TW3, 0.0f);

/**
 * Speed estimator coefficient θ_ω4
 *
 * Backward-Euler dynamics coefficient (relates to rotor time constant).
 * ω_k = (… − θ4·ω_{k-1}) / (1 + θ4)
 *
 * @decimal 6
 * @group Bench Test
 */
PARAM_DEFINE_FLOAT(BT_VC_TW4, 0.0f);

/**
 * Current estimator coefficient θ_I1
 *
 * Linear speed term in per-rotor current model.
 * I_rotor = θ1·ω + θ2·ω² + θ3
 *
 * @decimal 6
 * @group Bench Test
 */
PARAM_DEFINE_FLOAT(BT_VC_TI1, 0.0f);

/**
 * Current estimator coefficient θ_I2
 *
 * Quadratic speed term in per-rotor current model.
 *
 * @decimal 8
 * @group Bench Test
 */
PARAM_DEFINE_FLOAT(BT_VC_TI2, 0.0f);

/**
 * Current estimator coefficient θ_I3
 *
 * Constant term in per-rotor current model.
 *
 * @decimal 4
 * @group Bench Test
 */
PARAM_DEFINE_FLOAT(BT_VC_TI3, 0.0f);

/* ── Battery model: V0(SoC) = c0 + c1·s + c2·s² + c3·s³ ──────────── */

/**
 * Battery OCV polynomial c0 (constant)
 *
 * Open-circuit voltage polynomial coefficient, constant term.
 * SoC is in [0, 1].
 *
 * @decimal 4
 * @group Bench Test
 */
PARAM_DEFINE_FLOAT(BT_VC_V0C0, 0.0f);

/**
 * Battery OCV polynomial c1 (linear)
 * @decimal 4
 * @group Bench Test
 */
PARAM_DEFINE_FLOAT(BT_VC_V0C1, 0.0f);

/**
 * Battery OCV polynomial c2 (quadratic)
 * @decimal 4
 * @group Bench Test
 */
PARAM_DEFINE_FLOAT(BT_VC_V0C2, 0.0f);

/**
 * Battery OCV polynomial c3 (cubic)
 * @decimal 4
 * @group Bench Test
 */
PARAM_DEFINE_FLOAT(BT_VC_V0C3, 0.0f);

/* ── Battery model: R0(SoC) = c0 + c1·s + c2·s² + c3·s³ ──────────── */

/**
 * Battery R0 polynomial c0 (constant)
 *
 * Series resistance polynomial coefficient, constant term.
 *
 * @unit Ohm
 * @decimal 6
 * @group Bench Test
 */
PARAM_DEFINE_FLOAT(BT_VC_R0C0, 0.0f);

/**
 * Battery R0 polynomial c1 (linear)
 * @unit Ohm
 * @decimal 6
 * @group Bench Test
 */
PARAM_DEFINE_FLOAT(BT_VC_R0C1, 0.0f);

/**
 * Battery R0 polynomial c2 (quadratic)
 * @unit Ohm
 * @decimal 6
 * @group Bench Test
 */
PARAM_DEFINE_FLOAT(BT_VC_R0C2, 0.0f);

/**
 * Battery R0 polynomial c3 (cubic)
 * @unit Ohm
 * @decimal 6
 * @group Bench Test
 */
PARAM_DEFINE_FLOAT(BT_VC_R0C3, 0.0f);

/* ── Battery model: R1(SoC) = c0 + c1·s + c2·s² + c3·s³ ──────────── */

/**
 * Battery R1 polynomial c0 (constant)
 *
 * RC-branch resistance polynomial coefficient, constant term.
 *
 * @unit Ohm
 * @decimal 6
 * @group Bench Test
 */
PARAM_DEFINE_FLOAT(BT_VC_R1C0, 0.0f);

/**
 * Battery R1 polynomial c1 (linear)
 * @unit Ohm
 * @decimal 6
 * @group Bench Test
 */
PARAM_DEFINE_FLOAT(BT_VC_R1C1, 0.0f);

/**
 * Battery R1 polynomial c2 (quadratic)
 * @unit Ohm
 * @decimal 6
 * @group Bench Test
 */
PARAM_DEFINE_FLOAT(BT_VC_R1C2, 0.0f);

/**
 * Battery R1 polynomial c3 (cubic)
 * @unit Ohm
 * @decimal 6
 * @group Bench Test
 */
PARAM_DEFINE_FLOAT(BT_VC_R1C3, 0.0f);

/* ── Battery model: τ1(SoC) = c0 + c1·s + c2·s² + c3·s³ ──────────── */

/**
 * Battery τ1 polynomial c0 (constant)
 *
 * RC-branch time constant polynomial coefficient, constant term.
 *
 * @unit s
 * @decimal 4
 * @group Bench Test
 */
PARAM_DEFINE_FLOAT(BT_VC_T1C0, 0.0f);

/**
 * Battery τ1 polynomial c1 (linear)
 * @unit s
 * @decimal 4
 * @group Bench Test
 */
PARAM_DEFINE_FLOAT(BT_VC_T1C1, 0.0f);

/**
 * Battery τ1 polynomial c2 (quadratic)
 * @unit s
 * @decimal 4
 * @group Bench Test
 */
PARAM_DEFINE_FLOAT(BT_VC_T1C2, 0.0f);

/**
 * Battery τ1 polynomial c3 (cubic)
 * @unit s
 * @decimal 4
 * @group Bench Test
 */
PARAM_DEFINE_FLOAT(BT_VC_T1C3, 0.0f);
