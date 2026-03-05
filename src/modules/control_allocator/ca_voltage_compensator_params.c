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
 * @file ca_voltage_compensator_params.c
 *
 * Parameters for the in-flight voltage compensator in the control allocator.
 * These mirror the bench-test BT_VC_* parameters but are independent so
 * flight and bench configurations can coexist.
 */

/**
 * Voltage compensator enable
 *
 * Enable the battery-voltage compensator in the control allocator.
 * When enabled, motor commands are scaled to keep effective motor voltage
 * constant regardless of battery sag. Requires CA_VC_VBOP > 0.
 *
 * 0 = disabled
 * 1 = predicted mode (full battery model: estimates ω, current, RC-branch
 *     voltage drop – compensates based on predicted terminal voltage)
 * 2 = simple mode (reads the instantaneous measured terminal voltage from
 *     battery_status and compensates directly – no model integration)
 *
 * @value 0 Disabled
 * @value 1 Predicted (battery model)
 * @value 2 Simple (measured voltage)
 * @group Control Allocator
 * @reboot_required true
 */
PARAM_DEFINE_INT32(CA_VC_EN, 0);

/**
 * VC nominal battery voltage (Vb_op)
 *
 * The assumed "operating point" battery voltage.
 * The compensator scales commands so that δ·Vb_op = δ'·Vb_actual.
 * Typically the mid-SoC pack voltage. Set to 0 to disable.
 *
 * @unit V
 * @min 0.0
 * @max 60.0
 * @decimal 2
 * @group Control Allocator
 */
PARAM_DEFINE_FLOAT(CA_VC_VBOP, 0.0f);

/**
 * VC number of motors
 *
 * Number of motors the battery model should account for.
 * Should match the actual motor count on the airframe.
 *
 * @min 1
 * @max 8
 * @group Control Allocator
 */
PARAM_DEFINE_INT32(CA_VC_NMOT, 4);

/* ── Speed estimator coefficients ─────────────────────────────────────── */

/**
 * Speed estimator θ_ω1 (linear voltage)
 *
 * ω_k = (θ1·Vδ + θ2·√Vδ + θ3 − θ4·ω_{k-1}) / (1 + θ4)
 *
 * @decimal 4
 * @group Control Allocator
 */
PARAM_DEFINE_FLOAT(CA_VC_TW1, 0.0f);

/**
 * Speed estimator θ_ω2 (sqrt voltage)
 * @decimal 4
 * @group Control Allocator
 */
PARAM_DEFINE_FLOAT(CA_VC_TW2, 0.0f);

/**
 * Speed estimator θ_ω3 (constant)
 * @decimal 4
 * @group Control Allocator
 */
PARAM_DEFINE_FLOAT(CA_VC_TW3, 0.0f);

/**
 * Speed estimator θ_ω4 (dynamics)
 * @decimal 6
 * @group Control Allocator
 */
PARAM_DEFINE_FLOAT(CA_VC_TW4, 0.0f);

/* ── Current estimator coefficients ───────────────────────────────────── */

/**
 * Current estimator θ_I1 (cubic speed)
 *
 * I_rotor = θ1·ω³ + θ3
 *
 * @decimal 6
 * @group Control Allocator
 */
PARAM_DEFINE_FLOAT(CA_VC_TI1, 0.0f);

/**
 * Current estimator θ_I2 (unused, reserved)
 * @decimal 8
 * @group Control Allocator
 */
PARAM_DEFINE_FLOAT(CA_VC_TI2, 0.0f);

/**
 * Current estimator θ_I3 (constant)
 * @decimal 4
 * @group Control Allocator
 */
PARAM_DEFINE_FLOAT(CA_VC_TI3, 0.0f);

/* ── Battery OCV: V0(SoC) = c0 + c1·s + c2·s² + c3·s³ ───────────────── */

/**
 * Battery OCV polynomial c0 (constant)
 * @decimal 4
 * @group Control Allocator
 */
PARAM_DEFINE_FLOAT(CA_VC_V0C0, 0.0f);

/**
 * Battery OCV polynomial c1 (linear)
 * @decimal 4
 * @group Control Allocator
 */
PARAM_DEFINE_FLOAT(CA_VC_V0C1, 0.0f);

/**
 * Battery OCV polynomial c2 (quadratic)
 * @decimal 4
 * @group Control Allocator
 */
PARAM_DEFINE_FLOAT(CA_VC_V0C2, 0.0f);

/**
 * Battery OCV polynomial c3 (cubic)
 * @decimal 4
 * @group Control Allocator
 */
PARAM_DEFINE_FLOAT(CA_VC_V0C3, 0.0f);

/* ── Battery R0(SoC) = c0 + c1·s + c2·s² + c3·s³ ─────────────────────── */

/**
 * Battery R0 polynomial c0 (constant)
 * @unit Ohm
 * @decimal 6
 * @group Control Allocator
 */
PARAM_DEFINE_FLOAT(CA_VC_R0C0, 0.0f);

/**
 * Battery R0 polynomial c1 (linear)
 * @unit Ohm
 * @decimal 6
 * @group Control Allocator
 */
PARAM_DEFINE_FLOAT(CA_VC_R0C1, 0.0f);

/**
 * Battery R0 polynomial c2 (quadratic)
 * @unit Ohm
 * @decimal 6
 * @group Control Allocator
 */
PARAM_DEFINE_FLOAT(CA_VC_R0C2, 0.0f);

/**
 * Battery R0 polynomial c3 (cubic)
 * @unit Ohm
 * @decimal 6
 * @group Control Allocator
 */
PARAM_DEFINE_FLOAT(CA_VC_R0C3, 0.0f);

/* ── Battery R1(SoC) = c0 + c1·s + c2·s² + c3·s³ ─────────────────────── */

/**
 * Battery R1 polynomial c0 (constant)
 * @unit Ohm
 * @decimal 6
 * @group Control Allocator
 */
PARAM_DEFINE_FLOAT(CA_VC_R1C0, 0.0f);

/**
 * Battery R1 polynomial c1 (linear)
 * @unit Ohm
 * @decimal 6
 * @group Control Allocator
 */
PARAM_DEFINE_FLOAT(CA_VC_R1C1, 0.0f);

/**
 * Battery R1 polynomial c2 (quadratic)
 * @unit Ohm
 * @decimal 6
 * @group Control Allocator
 */
PARAM_DEFINE_FLOAT(CA_VC_R1C2, 0.0f);

/**
 * Battery R1 polynomial c3 (cubic)
 * @unit Ohm
 * @decimal 6
 * @group Control Allocator
 */
PARAM_DEFINE_FLOAT(CA_VC_R1C3, 0.0f);

/* ── Battery τ1(SoC) = c0 + c1·s + c2·s² + c3·s³ ─────────────────────── */

/**
 * Battery τ1 polynomial c0 (constant)
 * @unit s
 * @decimal 4
 * @group Control Allocator
 */
PARAM_DEFINE_FLOAT(CA_VC_T1C0, 0.0f);

/**
 * Battery τ1 polynomial c1 (linear)
 * @unit s
 * @decimal 4
 * @group Control Allocator
 */
PARAM_DEFINE_FLOAT(CA_VC_T1C1, 0.0f);

/**
 * Battery τ1 polynomial c2 (quadratic)
 * @unit s
 * @decimal 4
 * @group Control Allocator
 */
PARAM_DEFINE_FLOAT(CA_VC_T1C2, 0.0f);

/**
 * Battery τ1 polynomial c3 (cubic)
 * @unit s
 * @decimal 4
 * @group Control Allocator
 */
PARAM_DEFINE_FLOAT(CA_VC_T1C3, 0.0f);
