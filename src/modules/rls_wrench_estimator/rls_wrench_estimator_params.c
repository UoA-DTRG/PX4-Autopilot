/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * @file rls_wrench_estimator_params.c
 *
 * Parameters used by the RLS Identification and Wrench estimator
 *
 * @author Pedro Mendes <pmen817@aucklanduni.ac.nz>
 */

/**
 * Enable the RLS wrench estimator
 *
 * If set, the rls_wrench_estimator module is started at boot.
 *
 * @boolean
 * @reboot_required true
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_INT32(RLS_EST_EN, 0);

/**
 * Vehicle Total Mass [kg]
 *
 * Used in to estimate actuator forces applied
 * to vehicle based on accelerometer/gyro data.
 * Set to 1 if unknown.
 *
 * @decimal 5
 * @min 0.01
 * @max 50.0
 * @unit kg
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_FLOAT(RLS_EST_MASS, 1.04f);

/**
 * Motor Low-Pass Filter Time Constant [sec]
 *
 * Define first-order motor dynamics used with the PWM speed model.
 * Input: PWM values, Output: Motor Speed [rad/s].
 * Not applied when the ESC RPM source is used.
 *
 * @decimal 5
 * @min 0.01
 * @max 2.0
 * @unit s
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_FLOAT(RLS_EST_LPF_M, 0.1f);

/**
 * Number of thrust-coefficient groups
 *
 * Rotors sharing the same aerodynamics can share one estimated thrust
 * constant. Set to 1 to identify a single constant for all rotors, or
 * assign rotors to distinct groups with RLS_ROTORn_GRP (e.g. put coaxial
 * bottom props in their own group).
 * Note: only ~3 independent constants are observable from acceleration.
 *
 * @min 1
 * @max 12
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_INT32(RLS_EST_N_GRP, 1);

/**
 * Motor speed source
 *
 * 0: PWM model (actuator_outputs through affine + voltage correction).
 * 1: ESC RPM from esc_status (with per-motor fall-back to the PWM model
 *    when a motor does not report RPM). Only the first 8 motors can source
 *    ESC RPM; motors 8-11 always use the PWM model.
 *
 * @value 0 PWM model
 * @value 1 ESC RPM
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_INT32(RLS_EST_SPD_SRC, 0);

/**
 * Initial thrust constant guess (k_f*1e6)
 *
 * Applied to every thrust-coefficient group. Note the internal speed is in
 * rad/s, so k_f must be tuned for the rad/s^2 magnitude of w^2.
 *
 * @decimal 5
 * @min 0.01
 * @max 50.0
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_FLOAT(RLS_EST_KF_INIT, 1.5f);

/**
 * Confidence in thrust constant initial guess
 *
 * Set to 0 if perfect knowledge
 *
 * @decimal 5
 * @min 0.0
 * @max 100.0
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_FLOAT(RLS_EST_KF_CONF, 0.001f);

/**
 * Accelerometer xy-noise for RLS parameter identification
 *
 * @decimal 5
 * @min 0.01
 * @max 100.0
 * @unit m/s^2
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_FLOAT(RLS_EST_XY_NOISE, 1.f);

/**
 * Accelerometer z-noise for RLS parameter identification
 *
 * @decimal 5
 * @min 0.01
 * @max 100.0
 * @unit m/s^2
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_FLOAT(RLS_EST_Z_NOISE, 10.f);

/**
 * PWM to speed (P1)
 *
 * (PWM*P1 - P2) = SPEED [rad/s]
 * Calibrate so the model output is in rad/s.
 *
 * @decimal 6
 * @min 0.01
 * @max 10.0
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_FLOAT(RLS_EST_SPE_P1, 1.823f);

/**
 * PWM to speed (P2)
 *
 * (PWM*P1 - P2) = SPEED [rad/s]
 * Calibrate so the model output is in rad/s.
 *
 * @decimal 6
 * @min 0.01
 * @max 10000.0
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_FLOAT(RLS_EST_SPE_P2, 1673.7f);

/**
 * PWM to speed (Voltage Correction)
 *
 * @decimal 6
 * @min 0.01
 * @max 10000.0
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_FLOAT(RLS_EST_SPE_V1, 11.7f);

/**
 * PWM to speed (Voltage Correction)
 *
 * @decimal 6
 * @min 0.01
 * @max 10000.0
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_FLOAT(RLS_EST_SPE_V2, 1.5f);

/**
 * Force Estimator Time Constant [sec]
 *
 *
 * @decimal 5
 * @min 0.01
 * @max 5.0
 * @unit s
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_FLOAT(RLS_EST_TAU_F, 1.0f);

/**
 * Moment Estimator Time Constant [sec]
 *
 *
 * @decimal 5
 * @min 0.01
 * @max 5.0
 * @unit s
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_FLOAT(RLS_EST_TAU_M, 1.0f);

/**
 * CoM x-offset initial guess [mm]
 *
 * @decimal 5
 * @min 0.0
 * @max 1000.0
 * @unit mm
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_FLOAT(RLS_EST_XO_INIT, 0.f);

/**
 * CoM y-offset initial guess [mm]
 *
 * @decimal 5
 * @min 0.0
 * @max 1000.0
 * @unit mm
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_FLOAT(RLS_EST_YO_INIT, 0.f);

/**
 * Confidence in x-offset initial guess
 *
 * Set to 0 if perfect knowledge
 *
 * @decimal 5
 * @min 0.001
 * @max 10000.0
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_FLOAT(RLS_EST_XO_CONF, 1000.0f);

/**
 * Confidence in y-offset initial guess
 *
 * Set to 0 if perfect knowledge
 *
 * @decimal 5
 * @min 0.001
 * @max 10000.0
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_FLOAT(RLS_EST_YO_CONF, 1000.0f);

/**
 * Motor output noise for RLS parameter identification
 *
 * @decimal 5
 * @min 0.001
 * @max 100.0
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_FLOAT(RLS_EST_F_NOISE, 1.0f);

/**
 * Vehicle moment of inertia about x-axis [kg m^2]*1e3
 *
 *
 * @decimal 5
 * @min 0.1
 * @max 500.0
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_FLOAT(RLS_EST_IXX, 2.5513f);

/**
 * Vehicle moment of inertia about y-axis [kg m^2]*1e3
 *
 *
 * @decimal 5
 * @min 0.1
 * @max 500.0
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_FLOAT(RLS_EST_IYY, 2.8425f);

/**
 * Vehicle moment of inertia about z-axis [kg m^2]*1e3
 *
 *
 * @decimal 5
 * @min 0.1
 * @max 500.0
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_FLOAT(RLS_EST_IZZ, 4.5935f);

/**
 * Thrust-coefficient group index for motor 0
 *
 * @min 0
 * @max 11
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_INT32(RLS_ROTOR0_GRP, 0);

/**
 * Thrust-coefficient group index for motor 1
 *
 * @min 0
 * @max 11
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_INT32(RLS_ROTOR1_GRP, 0);

/**
 * Thrust-coefficient group index for motor 2
 *
 * @min 0
 * @max 11
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_INT32(RLS_ROTOR2_GRP, 0);

/**
 * Thrust-coefficient group index for motor 3
 *
 * @min 0
 * @max 11
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_INT32(RLS_ROTOR3_GRP, 0);

/**
 * Thrust-coefficient group index for motor 4
 *
 * @min 0
 * @max 11
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_INT32(RLS_ROTOR4_GRP, 0);

/**
 * Thrust-coefficient group index for motor 5
 *
 * @min 0
 * @max 11
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_INT32(RLS_ROTOR5_GRP, 0);

/**
 * Thrust-coefficient group index for motor 6
 *
 * @min 0
 * @max 11
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_INT32(RLS_ROTOR6_GRP, 0);

/**
 * Thrust-coefficient group index for motor 7
 *
 * @min 0
 * @max 11
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_INT32(RLS_ROTOR7_GRP, 0);

/**
 * Thrust-coefficient group index for motor 8
 *
 * @min 0
 * @max 11
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_INT32(RLS_ROTOR8_GRP, 0);

/**
 * Thrust-coefficient group index for motor 9
 *
 * @min 0
 * @max 11
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_INT32(RLS_ROTOR9_GRP, 0);

/**
 * Thrust-coefficient group index for motor 10
 *
 * @min 0
 * @max 11
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_INT32(RLS_ROTOR10_GRP, 0);

/**
 * Thrust-coefficient group index for motor 11
 *
 * @min 0
 * @max 11
 * @group RLS Wrench Estimator
 */
PARAM_DEFINE_INT32(RLS_ROTOR11_GRP, 0);
