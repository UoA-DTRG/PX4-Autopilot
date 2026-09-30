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
 * @file bench_test_profile.h
 *
 * The excitation profile maths of the bench test mode: step and ramp outputs,
 * the throttle spin-up, the mapping of the excited axis onto the thrust and
 * torque setpoints, and the motor saturation test that freezes the ramp.
 *
 * Header-only and free of module state so the unit tests can exercise it
 * without running the module (BenchProfileTest.cpp).
 */

#pragma once

#include <math.h>
#include <stdint.h>

#include <mathlib/math/Limits.hpp>
#include <px4_platform_common/defines.h>
#include <uORB/topics/actuator_motors.h>

namespace bench_test
{

enum class Mode : int32_t {
	Hover = 0,
	Step  = 1,
	Ramp  = 2,
};

enum class Axis : int32_t {
	ThrustX = 0,
	ThrustY = 1,
	ThrustZ = 2,
	Roll    = 3,
	Pitch   = 4,
	Yaw     = 5,
};

/**
 * Step profile: nothing until @p delay, then sign * @p mag for @p dur seconds, then nothing again.
 */
static inline float stepOutput(float sign, float dt_since_start, float delay, float dur, float mag)
{
	if (dt_since_start < delay) {
		return 0.f;
	}

	if (dt_since_start < delay + dur) {
		return sign * mag;
	}

	return 0.f;
}

/// Ramp freeze state: once a motor saturates (or the clamp is hit) the ramp holds its value.
struct RampState {
	bool frozen{false};
	float value{0.f};

	void reset()
	{
		frozen = false;
		value = 0.f;
	}
};

/**
 * Ramp profile: sign * @p rate * t, clamped to +-@p max_val. Keeps increasing until a motor
 * saturates (upper or lower) or the clamp is reached, then freezes and holds that value
 * until @p state is reset.
 */
static inline float rampOutput(RampState &state, float sign, float dt_since_start, float rate, float max_val,
			       bool motor_saturated)
{
	if (!state.frozen) {
		const float ramped = sign * rate * dt_since_start;
		state.value = math::constrain(ramped, -max_val, max_val);

		if (motor_saturated || fabsf(state.value) >= max_val) {
			state.frozen = true;
		}
	}

	return state.value;
}

/**
 * Throttle spin-up factor in [0, 1]: the hover baseline is ramped up linearly over
 * @p spinup_t seconds from the moment the outputs became active. A non-positive
 * @p spinup_t disables the spin-up.
 */
static inline float spinupFactor(float since_output_s, float spinup_t)
{
	if (spinup_t > 0.f) {
		return math::constrain(since_output_s / spinup_t, 0.f, 1.f);
	}

	return 1.f;
}

/// Body frame thrust and torque setpoints, before the [-1, 1] clamp applied on publication.
struct Setpoints {
	float thrust[3] {};
	float torque[3] {};
};

/**
 * Hover baseline on Z body thrust (NED, so -Z is up) plus the excitation on the selected axis.
 * For ThrustZ the excitation adds thrust upwards on top of the hover baseline.
 */
static inline Setpoints composeSetpoints(Axis axis, float hover_thr, float spinup, float axis_output)
{
	Setpoints sp{};
	sp.thrust[2] = -hover_thr * spinup;

	switch (axis) {
	case Axis::ThrustX:
		sp.thrust[0] = axis_output;
		break;

	case Axis::ThrustY:
		sp.thrust[1] = axis_output;
		break;

	case Axis::ThrustZ:
		sp.thrust[2] -= axis_output; // additional thrust in -Z (up)
		break;

	case Axis::Roll:
		sp.torque[0] = axis_output;
		break;

	case Axis::Pitch:
		sp.torque[1] = axis_output;
		break;

	case Axis::Yaw:
		sp.torque[2] = axis_output;
		break;
	}

	return sp;
}

/// Number of finite outputs in @p motors. Unused motor slots are NaN.
static inline int countFiniteMotors(const actuator_motors_s &motors)
{
	int count = 0;

	for (int i = 0; i < actuator_motors_s::NUM_CONTROLS; i++) {
		if (PX4_ISFINITE(motors.control[i])) {
			count++;
		}
	}

	return count;
}

/**
 * True once any of the first @p num_motors outputs comes within @p margin of its limit.
 * Reversible motors span [-1, 1], standard motors [0, 1]. The margin is clamped to [0, 0.5],
 * and non-finite outputs are ignored.
 */
static inline bool motorsSaturated(const actuator_motors_s &motors, int num_motors, float margin)
{
	margin = math::constrain(margin, 0.f, 0.5f);
	const float upper = 1.f - margin;

	const int n = math::min(num_motors, static_cast<int>(actuator_motors_s::NUM_CONTROLS));

	for (int i = 0; i < n; i++) {
		const float c = motors.control[i];

		if (!PX4_ISFINITE(c)) {
			continue;
		}

		const bool reversible = (motors.reversible_flags & (1u << i)) != 0;
		const float lower = reversible ? (-1.f + margin) : margin;

		if (c >= upper || c <= lower) {
			return true;
		}
	}

	return false;
}

} // namespace bench_test
