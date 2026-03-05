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
 * @file VoltageCompensator.hpp
 *
 * Battery-voltage compensator for motor commands.
 *
 * Adjusts the motor command δ to counteract battery voltage sag so
 * that the effective voltage δ·Vb stays constant regardless of SoC
 * and current draw.
 *
 * Algorithm (forward Euler, called at the motor-command rate):
 *   1. V_delta = δ · Vb_op
 *   2. ω_k    = (θw1·Vδ + θw2·√Vδ + θw3 − θw4·ω_{k-1}) / (1+θw4)
 *   3. I_rot  = θI1·ω³ + θI3                  (per rotor)
 *   4. I_tot  = Σ I_rot
 *   5. V0, R0, R1, τ1 from 3rd-order polynomials in SoC
 *   6. V_RC_k = V_RC_{k-1} + Ts·(R1/τ1·I − 1/τ1·V_RC_{k-1})
 *   7. Vb_k   = V0 − I·R0 − V_RC_k
 *   8. δ'     = (Vb_op / Vb_k) · δ
 *
 * Shared between bench_test module and control_allocator for in-flight use.
 */

#pragma once

#include <math.h>

struct VoltageCompensator {

	static constexpr int MAX_ROTORS = 8;

	/** Result struct — captures all intermediate predicted states for logging */
	struct Result {
		float omega[MAX_ROTORS]{};   /**< predicted rotor speed per motor */
		float i_motor[MAX_ROTORS]{}; /**< predicted current per motor (A) */
		float i_total{0.0f};         /**< total predicted current (A) */
		float v_b_pred{0.0f};        /**< predicted terminal voltage (V) */
		float v0{0.0f};              /**< OCV V0(SoC) (V) */
		float r0{0.0f};              /**< series resistance R0(SoC) (Ohm) */
		float r1{0.0f};              /**< RC resistance R1(SoC) (Ohm) */
		float tau1{0.0f};            /**< RC time constant τ1(SoC) (s) */
		float v_rc{0.0f};            /**< RC branch voltage (V) */
		float c_delta{1.0f};         /**< correction factor Vb_op/Vb_pred */
	};

	/* Coefficients (set from params before first use) */
	float Vb_op{0.0f};                      /**< nominal battery voltage */

	float tw1{0.0f}, tw2{0.0f}, tw3{0.0f}, tw4{0.0f};  /**< speed estimator θ_ω */
	float ti1{0.0f}, ti2{0.0f}, ti3{0.0f};              /**< current estimator θ_I */

	float v0c[4]{};   /**< V0(SoC) polynomial c0..c3 */
	float r0c[4]{};   /**< R0(SoC) polynomial c0..c3 */
	float r1c[4]{};   /**< R1(SoC) polynomial c0..c3 */
	float t1c[4]{};   /**< τ1(SoC) polynomial c0..c3 */

	/* State */
	int   n_rotors{0};
	float omega_prev[MAX_ROTORS]{};          /**< previous rotor speed estimates */
	float delta_prev[MAX_ROTORS]{};          /**< last commanded delta per motor */
	float V_RC{0.0f};                        /**< RC-branch voltage state */

	/** Evaluate a 3rd-order polynomial: c0 + c1·x + c2·x² + c3·x³ */
	static float poly3(const float c[4], float x)
	{
		return c[0] + x * (c[1] + x * (c[2] + x * c[3]));
	}

	/** Reset dynamic state (call at the start of each test). */
	void reset(int num_rotors)
	{
		n_rotors = (num_rotors > MAX_ROTORS) ? MAX_ROTORS : num_rotors;

		for (int i = 0; i < MAX_ROTORS; i++) {
			omega_prev[i] = 0.0f;
			delta_prev[i] = 0.0f;
		}

		V_RC = 0.0f;
	}

	/** Check whether the compensator is configured (Vb_op > 0). */
	bool isConfigured() const { return Vb_op > 1.0f; }

	/**
	 * Compute the compensated command δ' for one motor.
	 *
	 * @param motor_index  0-based index of the motor being commanded
	 * @param delta        raw normalised command for this motor [0, 1]
	 * @param soc          battery state of charge [0, 1]
	 * @param dt           time step (s) since last call
	 * @param out_Vb       (out) predicted terminal battery voltage (V)
	 * @param out_I        (out) predicted total current draw (A)
	 * @param out_result   (out, optional) full predicted-state snapshot
	 * @return             compensated command δ', clamped to [0, 1]
	 */
	float update(int motor_index, float delta, float soc, float dt,
		     float &out_Vb, float &out_I, Result *out_result = nullptr)
	{
		out_Vb = 0.0f;
		out_I  = 0.0f;

		if (!isConfigured() || dt <= 0.0f) {
			return delta;
		}

		/* Update stored delta for this motor */
		if (motor_index >= 0 && motor_index < MAX_ROTORS) {
			delta_prev[motor_index] = delta;
		}

		/* ── Speed & current for every active motor ──────────── */
		const float denom = 1.0f + tw4;
		float I_total = 0.0f;

		for (int i = 0; i < n_rotors; i++) {
			const float d   = delta_prev[i];
			const float Vd  = d * Vb_op;
			const float sVd = (Vd > 0.0f) ? sqrtf(Vd) : 0.0f;

			float omega = (tw1 * Vd + tw2 * sVd + tw3
				       - tw4 * omega_prev[i]) / (denom * 10000.0f);

			if (omega < 0.0f) { omega = 0.0f; }

			const float I_rotor = ti1 * (omega * omega * omega) + ti3;
			I_total += I_rotor;
			omega_prev[i] = omega;

			if (out_result) {
				out_result->omega[i]   = omega;
				out_result->i_motor[i] = I_rotor;
			}
		}

		if (I_total < 0.0f) { I_total = 0.0f; }

		/* ── Battery model ───────────────────────────────────── */
		const float V0  = poly3(v0c, soc);
		const float R0  = poly3(r0c, soc);
		const float R1  = poly3(r1c, soc);
		const float tau = poly3(t1c, soc);

		if (tau > 1e-6f) {
			V_RC = V_RC + dt * (R1 / tau * I_total - V_RC / tau);
		}

		float Vb_pred = V0 - I_total * R0 - V_RC;

		if (Vb_pred < 1.0f) { Vb_pred = 1.0f; }

		/* ── Correction factor for this motor's command ──────── */
		float C_delta = Vb_op / Vb_pred;

		if (C_delta < 0.8f) { C_delta = 0.8f; }

		if (C_delta > 1.5f) { C_delta = 1.5f; }

		float delta_prime = C_delta * delta;

		if (delta_prime < 0.0f) { delta_prime = 0.0f; }

		if (delta_prime > 1.0f) { delta_prime = 1.0f; }

		out_Vb = Vb_pred;
		out_I  = I_total;

		if (out_result) {
			out_result->i_total = I_total;
			out_result->v_b_pred = Vb_pred;
			out_result->v0   = V0;
			out_result->r0   = R0;
			out_result->r1   = R1;
			out_result->tau1 = tau;
			out_result->v_rc = V_RC;
			out_result->c_delta = C_delta;
		}

		return delta_prime;
	}

	/**
	 * Update all motors at once (for in-flight use where all commands
	 * are available simultaneously).
	 *
	 * @param deltas       raw normalised commands per motor [0, 1], NAN for unused
	 * @param n_motors     number of motors to process
	 * @param soc          battery state of charge [0, 1]
	 * @param dt           time step (s) since last call
	 * @param out_deltas   (out) compensated commands per motor
	 * @param out_result   (out) full predicted-state snapshot
	 */
	void updateAll(const float *deltas, int n_motors, float soc, float dt,
		       float *out_deltas, Result &out_result)
	{
		out_result = {};

		if (!isConfigured() || dt <= 0.0f || soc < 0.0f) {
			for (int i = 0; i < n_motors && i < MAX_ROTORS; i++) {
				out_deltas[i] = deltas[i];
			}

			return;
		}

		/* ── Update stored deltas for all motors ─────────────── */
		const int nm = (n_motors > MAX_ROTORS) ? MAX_ROTORS : n_motors;

		for (int i = 0; i < nm; i++) {
			if (__builtin_isfinite(deltas[i])) {
				delta_prev[i] = deltas[i];

			} else {
				delta_prev[i] = 0.0f;
			}
		}

		/* ── Speed & current for every active motor ──────────── */
		const float denom = 1.0f + tw4;
		float I_total = 0.0f;

		for (int i = 0; i < n_rotors; i++) {
			const float d   = delta_prev[i];
			const float Vd  = d * Vb_op;
			const float sVd = (Vd > 0.0f) ? sqrtf(Vd) : 0.0f;

			float omega = (tw1 * Vd + tw2 * sVd + tw3
				       - tw4 * omega_prev[i]) / (denom * 10000.0f);

			if (omega < 0.0f) { omega = 0.0f; }

			const float I_rotor = ti1 * (omega * omega * omega) + ti3;
			I_total += I_rotor;
			omega_prev[i] = omega;

			out_result.omega[i]   = omega;
			out_result.i_motor[i] = I_rotor;
		}

		if (I_total < 0.0f) { I_total = 0.0f; }

		/* ── Battery model ───────────────────────────────────── */
		const float V0  = poly3(v0c, soc);
		const float R0  = poly3(r0c, soc);
		const float R1  = poly3(r1c, soc);
		const float tau = poly3(t1c, soc);

		if (tau > 1e-6f) {
			V_RC = V_RC + dt * (R1 / tau * I_total - V_RC / tau);
		}

		float Vb_pred = V0 - I_total * R0 - V_RC;

		if (Vb_pred < 1.0f) { Vb_pred = 1.0f; }

		/* ── Correction factor ───────────────────────────────── */
		float C_delta = Vb_op / Vb_pred;

		if (C_delta < 0.8f) { C_delta = 0.8f; }

		if (C_delta > 1.5f) { C_delta = 1.5f; }

		for (int i = 0; i < nm; i++) {
			if (__builtin_isfinite(deltas[i])) {
				float d = C_delta * deltas[i];

				if (d < 0.0f) { d = 0.0f; }

				if (d > 1.0f) { d = 1.0f; }

				out_deltas[i] = d;

			} else {
				out_deltas[i] = deltas[i]; // keep NAN
			}
		}

		out_result.i_total  = I_total;
		out_result.v_b_pred = Vb_pred;
		out_result.v0       = V0;
		out_result.r0       = R0;
		out_result.r1       = R1;
		out_result.tau1     = tau;
		out_result.v_rc     = V_RC;
		out_result.c_delta  = C_delta;
	}
};
