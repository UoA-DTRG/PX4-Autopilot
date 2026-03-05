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
 * @file voltage_compensator_params.c
 *
 * Shared parameters for the battery-voltage compensator (VoltageCompensator).
 * Used by both the control allocator (in-flight) and the bench_test module.
 * Prefix: VC_
 *
 * Model overview
 * ══════════════
 *  Speed estimator (per rotor):
 *    Vδ = δ · Vb_op
 *    ω_k = (θω1·Vδ + θω2·√Vδ + θω3 − θω4·ω_{k-1}) / (1 + θω4)
 *
 *  Current estimator (total, shared):
 *    I = Σ (θI1·ω_i + θI2·ω_i² + θI3)
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
 */

#include <parameters/param.h>

/**
 * Voltage compensator nominal battery voltage (Vb_op)
 *
 * The assumed "operating point" battery voltage.
 * The compensator scales commands so that δ·Vb_op = δ'·Vb_actual,
 * keeping rotor speed independent of battery state.
 * Typically the mid-SoC pack voltage. Set to 0.0 to leave unconfigured.
 *
 * @unit V
 * @min 0.0
 * @max 60.0
 * @decimal 2
 * @group Voltage Compensator
 */
PARAM_DEFINE_FLOAT(VC_VBOP, 0.0f);

/**
 * Voltage compensator number of motors
 *
 * Number of motors the battery current model accounts for.
 * Should match the actual motor count on the airframe.
 *
 * @min 1
 * @max 8
 * @group Voltage Compensator
 */
PARAM_DEFINE_INT32(VC_NMOT, 4);

/* ── Speed estimator coefficients ─────────────────────────────────────── */

/**
 * Speed estimator θ_ω1 (linear voltage)
 *
 * ω_k = (θ1·Vδ + θ2·√Vδ + θ3 − θ4·ω_{k-1}) / (1 + θ4)
 *
 * @decimal 4
 * @group Voltage Compensator
 */
PARAM_DEFINE_FLOAT(VC_TW1, 0.0f);

/**
 * Speed estimator θ_ω2 (sqrt voltage)
 * @decimal 4
 * @group Voltage Compensator
 */
PARAM_DEFINE_FLOAT(VC_TW2, 0.0f);

/**
 * Speed estimator θ_ω3 (constant)
 * @decimal 4
 * @group Voltage Compensator
 */
PARAM_DEFINE_FLOAT(VC_TW3, 0.0f);

/**
 * Speed estimator θ_ω4 (dynamics)
 * @decimal 6
 * @group Voltage Compensator
 */
PARAM_DEFINE_FLOAT(VC_TW4, 0.0f);

/* ── Current estimator coefficients ───────────────────────────────────── */

/**
 * Current estimator θ_I1 (linear speed)
 *
 * I_rotor = θ1·ω + θ2·ω² + θ3
 *
 * @decimal 6
 * @group Voltage Compensator
 */
PARAM_DEFINE_FLOAT(VC_TI1, 0.0f);

/**
 * Current estimator θ_I2 (quadratic speed)
 * @decimal 8
 * @group Voltage Compensator
 */
PARAM_DEFINE_FLOAT(VC_TI2, 0.0f);

/**
 * Current estimator θ_I3 (constant)
 * @decimal 4
 * @group Voltage Compensator
 */
PARAM_DEFINE_FLOAT(VC_TI3, 0.0f);

/* ── Battery OCV: V0(SoC) = c0 + c1·s + c2·s² + c3·s³ ───────────────── */

/**
 * Battery OCV polynomial c0 (constant)
 * @decimal 4
 * @group Voltage Compensator
 */
PARAM_DEFINE_FLOAT(VC_V0C0, 0.0f);

/**
 * Battery OCV polynomial c1 (linear)
 * @decimal 4
 * @group Voltage Compensator
 */
PARAM_DEFINE_FLOAT(VC_V0C1, 0.0f);

/**
 * Battery OCV polynomial c2 (quadratic)
 * @decimal 4
 * @group Voltage Compensator
 */
PARAM_DEFINE_FLOAT(VC_V0C2, 0.0f);

/**
 * Battery OCV polynomial c3 (cubic)
 * @decimal 4
 * @group Voltage Compensator
 */
PARAM_DEFINE_FLOAT(VC_V0C3, 0.0f);

/* ── Battery R0(SoC) = c0 + c1·s + c2·s² + c3·s³ ─────────────────────── */

/**
 * Battery R0 polynomial c0 (constant)
 * @unit Ohm
 * @decimal 6
 * @group Voltage Compensator
 */
PARAM_DEFINE_FLOAT(VC_R0C0, 0.0f);

/**
 * Battery R0 polynomial c1 (linear)
 * @unit Ohm
 * @decimal 6
 * @group Voltage Compensator
 */
PARAM_DEFINE_FLOAT(VC_R0C1, 0.0f);

/**
 * Battery R0 polynomial c2 (quadratic)
 * @unit Ohm
 * @decimal 6
 * @group Voltage Compensator
 */
PARAM_DEFINE_FLOAT(VC_R0C2, 0.0f);

/**
 * Battery R0 polynomial c3 (cubic)
 * @unit Ohm
 * @decimal 6
 * @group Voltage Compensator
 */
PARAM_DEFINE_FLOAT(VC_R0C3, 0.0f);

/* ── Battery R1(SoC) = c0 + c1·s + c2·s² + c3·s³ ─────────────────────── */

/**
 * Battery R1 polynomial c0 (constant)
 * @unit Ohm
 * @decimal 6
 * @group Voltage Compensator
 */
PARAM_DEFINE_FLOAT(VC_R1C0, 0.0f);

/**
 * Battery R1 polynomial c1 (linear)
 * @unit Ohm
 * @decimal 6
 * @group Voltage Compensator
 */
PARAM_DEFINE_FLOAT(VC_R1C1, 0.0f);

/**
 * Battery R1 polynomial c2 (quadratic)
 * @unit Ohm
 * @decimal 6
 * @group Voltage Compensator
 */
PARAM_DEFINE_FLOAT(VC_R1C2, 0.0f);

/**
 * Battery R1 polynomial c3 (cubic)
 * @unit Ohm
 * @decimal 6
 * @group Voltage Compensator
 */
PARAM_DEFINE_FLOAT(VC_R1C3, 0.0f);

/* ── Battery τ1(SoC) = c0 + c1·s + c2·s² + c3·s³ ─────────────────────── */

/**
 * Battery τ1 polynomial c0 (constant)
 * @unit s
 * @decimal 4
 * @group Voltage Compensator
 */
PARAM_DEFINE_FLOAT(VC_T1C0, 0.0f);

/**
 * Battery τ1 polynomial c1 (linear)
 * @unit s
 * @decimal 4
 * @group Voltage Compensator
 */
PARAM_DEFINE_FLOAT(VC_T1C1, 0.0f);

/**
 * Battery τ1 polynomial c2 (quadratic)
 * @unit s
 * @decimal 4
 * @group Voltage Compensator
 */
PARAM_DEFINE_FLOAT(VC_T1C2, 0.0f);

/**
 * Battery τ1 polynomial c3 (cubic)
 * @unit s
 * @decimal 4
 * @group Voltage Compensator
 */
PARAM_DEFINE_FLOAT(VC_T1C3, 0.0f);
