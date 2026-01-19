/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
 * @file effectiveness_estimator_params.c
 *
 * Parameters for the effectiveness estimator
 */

/**
 * Enable effectiveness estimator
 *
 * Enable the RLS-based effectiveness matrix estimator.
 * The estimator runs in the background and logs estimates
 * without affecting the control allocation.
 *
 * @boolean
 * @group Effectiveness Estimator
 */
PARAM_DEFINE_INT32(EFF_EST_ENABLE, 0);

/**
 * RLS forgetting factor
 *
 * Forgetting factor for the RLS algorithm.
 * Higher values (closer to 1.0) give more weight to past data.
 * Lower values allow faster adaptation to changes.
 *
 * @decimal 4
 * @min 0.9
 * @max 0.9999
 * @group Effectiveness Estimator
 */
PARAM_DEFINE_FLOAT(EFF_EST_LAMBDA, 0.99);

/**
 * Initial covariance value
 *
 * Initial diagonal value for the RLS covariance matrix.
 * Higher values indicate higher initial uncertainty.
 *
 * @decimal 1
 * @min 1.0
 * @max 10000.0
 * @group Effectiveness Estimator
 */
PARAM_DEFINE_FLOAT(EFF_EST_P_INIT, 1000.0);

/**
 * Number of rotors
 *
 * Number of rotors/motors on the vehicle.
 *
 * @min 1
 * @max 16
 * @group Effectiveness Estimator
 */
PARAM_DEFINE_INT32(EFF_EST_NUM_ROTORS, 4);

/**
 * Vehicle mass
 *
 * Total mass of the vehicle in kilograms.
 * Used for force effectiveness estimation.
 *
 * @decimal 2
 * @min 0.1
 * @max 100.0
 * @unit kg
 * @group Effectiveness Estimator
 */
PARAM_DEFINE_FLOAT(EFF_EST_MASS, 1.5);

/**
 * Moment of inertia about X axis
 *
 * Vehicle moment of inertia about the X (roll) axis.
 *
 * @decimal 4
 * @min 0.0001
 * @max 10.0
 * @unit kg*m^2
 * @group Effectiveness Estimator
 */
PARAM_DEFINE_FLOAT(EFF_EST_IXX, 0.029);

/**
 * Moment of inertia about Y axis
 *
 * Vehicle moment of inertia about the Y (pitch) axis.
 *
 * @decimal 4
 * @min 0.0001
 * @max 10.0
 * @unit kg*m^2
 * @group Effectiveness Estimator
 */
PARAM_DEFINE_FLOAT(EFF_EST_IYY, 0.029);

/**
 * Moment of inertia about Z axis
 *
 * Vehicle moment of inertia about the Z (yaw) axis.
 *
 * @decimal 4
 * @min 0.0001
 * @max 10.0
 * @unit kg*m^2
 * @group Effectiveness Estimator
 */
PARAM_DEFINE_FLOAT(EFF_EST_IZZ, 0.055);

/**
 * Estimator update rate
 *
 * Rate at which the effectiveness estimator runs.
 *
 * @decimal 1
 * @min 10.0
 * @max 500.0
 * @unit Hz
 * @group Effectiveness Estimator
 */
PARAM_DEFINE_FLOAT(EFF_EST_UPDATE_RATE, 50.0);

/**
 * Convergence variance threshold
 *
 * Maximum average parameter variance for convergence detection.
 * Lower values require tighter convergence.
 *
 * @decimal 2
 * @min 0.01
 * @max 10.0
 * @group Effectiveness Estimator
 */
PARAM_DEFINE_FLOAT(EFF_EST_CONV_VAR, 1.0);

/**
 * Convergence innovation threshold
 *
 * Maximum RMS innovation for convergence detection.
 * Lower values require better fit to measurements.
 *
 * @decimal 1
 * @min 0.1
 * @max 100.0
 * @unit N*m
 * @group Effectiveness Estimator
 */
PARAM_DEFINE_FLOAT(EFF_EST_CONV_INNOV, 10.0);

/**
 * Minimum excitation threshold
 *
 * Minimum actuator output norm required for RLS update.
 * Prevents updates with insufficient excitation.
 *
 * @decimal 3
 * @min 0.001
 * @max 0.5
 * @group Effectiveness Estimator
 */
PARAM_DEFINE_FLOAT(EFF_EST_MIN_EXCITE, 0.01);
