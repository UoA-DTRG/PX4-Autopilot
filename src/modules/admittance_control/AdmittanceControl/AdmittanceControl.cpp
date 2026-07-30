/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file AdmittanceControl.cpp
 */

#include <AdmittanceControl.hpp>

using namespace matrix;

void AdmittanceControl::reset()
{
	_y.setZero();
	_accel.setZero();
}

void AdmittanceControl::updateSaturation(float dt, float raw_sat)
{
	if (!PX4_ISFINITE(raw_sat) || !PX4_ISFINITE(dt) || (dt <= 0.f)) {
		return;
	}

	// The saturation measure is a magnitude, so it can never be negative. Clamping
	// here is what keeps `A - sat_factor` from growing and stiffening the response,
	// which is the opposite of what saturation should do.
	raw_sat = math::constrain(raw_sat, 0.f, 1.f);

	const float tau = math::max(_params_bell.lpf_sat_factor, 0.f);
	const float alpha = dt / (tau + dt);
	_sat_factor = alpha * raw_sat + (1.f - alpha) * _sat_factor;
}

void AdmittanceControl::scheduleGains(const Vector<float, 4> &We, float target_dist)
{
	for (int i = 0; i < 4; i++) {
		// Denominator of the bell curve. Saturation softens the response by shrinking
		// it, but it must never reach zero.
		const float a = math::max(_params_bell.A[i] - _sat_factor, kMinBellDenominator);

		// Exponent, sharpened as the target gets closer.
		const float b = math::max(_params_bell.B1[i] - (_params_bell.B2[i] * expf(-_params_bell.B3[i] * target_dist)),
					  kMinBellExponent);

		const float bell = 1.f / (1.f + powf(fabsf(We(i) / a), 2.f * b));

		_params.M[i] = math::max(_params_bell.M_min[i] + ((_params_bell.M_max[i] - _params_bell.M_min[i]) * bell), kMinMass);
		_params.K[i] = math::max(_params_bell.K_min[i] + ((_params_bell.K_max[i] - _params_bell.K_min[i]) * bell), 0.f);
		_params.C[i] = 2.f * sqrtf(_params.K[i] * _params.M[i]);
	}
}

/**
 * Deviation dynamics, per axis:
 *
 *   d_ddot = (We - C d_dot - K d) / M
 *
 * State layout y = [dx, dx_dot, dy, dy_dot, dz, dz_dot, dyaw, dyaw_dot],
 * input u = [fx, fy, fz, mz].
 */
static Vector<float, 8> deviationDynamics(float t, const Matrix<float, 8, 1> &y, const Matrix<float, 4, 1> &u,
		const AdmittanceParameters &params)
{
	(void)t; // time invariant

	const float states[8] = {
		y(1, 0), ((u(0, 0) - (params.C[0] * y(1, 0)) - (params.K[0] * y(0, 0))) / params.M[0]),
		y(3, 0), ((u(1, 0) - (params.C[1] * y(3, 0)) - (params.K[1] * y(2, 0))) / params.M[1]),
		y(5, 0), ((u(2, 0) - (params.C[2] * y(5, 0)) - (params.K[2] * y(4, 0))) / params.M[2]),
		y(7, 0), ((u(3, 0) - (params.C[3] * y(7, 0)) - (params.K[3] * y(6, 0))) / params.M[3])
	};

	return Vector<float, 8>(states);
}

void AdmittanceControl::update(float dt, const Vector<float, 4> &We, float target_dist)
{
	scheduleGains(We, target_dist);

	// Capture the accelerations at the start of the step so the caller can publish
	// them as an acceleration feed-forward.
	const Vector<float, 8> dydt = deviationDynamics(0.f, _y, We, _params);
	_accel(0) = dydt(1);
	_accel(1) = dydt(3);
	_accel(2) = dydt(5);
	_accel(3) = dydt(7);

	_y = _integrate(dt, We);

	applyLimits();
}

Vector<float, 8> AdmittanceControl::_integrate(float dt, const Vector<float, 4> &We)
{
	Vector<float, 8> y = _y;
	_integrate_rk4(deviationDynamics, y, We, 0.f, dt, dt, y, _params);
	return y;
}

void AdmittanceControl::applyLimits()
{
	// Position states at even indices, rate states at odd indices.
	for (int axis = 0; axis < 3; axis++) {
		const int i_pos = 2 * axis;
		const int i_rate = i_pos + 1;

		if (!PX4_ISFINITE(_y(i_pos)) || !PX4_ISFINITE(_y(i_rate))) {
			reset();
			return;
		}

		const float clamped = math::constrain(_y(i_pos), -_deviation_max, _deviation_max);

		if (fabsf(clamped - _y(i_pos)) > FLT_EPSILON) {
			// Stop integrating further into the clamp, otherwise the state winds up and
			// the deviation stays pinned long after the contact has gone.
			if (clamped * _y(i_rate) > 0.f) {
				_y(i_rate) = 0.f;
			}

			_y(i_pos) = clamped;
		}

		_y(i_rate) = math::constrain(_y(i_rate), -_deviation_rate_max, _deviation_rate_max);
	}

	if (!PX4_ISFINITE(_y(6)) || !PX4_ISFINITE(_y(7))) {
		reset();
		return;
	}

	const float yaw_clamped = math::constrain(_y(6), -kMaxYawDeviation, kMaxYawDeviation);

	if (fabsf(yaw_clamped - _y(6)) > FLT_EPSILON) {
		if (yaw_clamped * _y(7) > 0.f) {
			_y(7) = 0.f;
		}

		_y(6) = yaw_clamped;
	}
}

bool AdmittanceControl::isFinite() const
{
	for (int i = 0; i < 8; i++) {
		if (!PX4_ISFINITE(_y(i))) {
			return false;
		}
	}

	for (int i = 0; i < 4; i++) {
		if (!PX4_ISFINITE(_accel(i)) || !PX4_ISFINITE(_params.M[i]) || !PX4_ISFINITE(_params.C[i])
		    || !PX4_ISFINITE(_params.K[i])) {
			return false;
		}
	}

	return true;
}

int AdmittanceControl::_integrate_rk4(
	Vector<float, 8> (*f)(float, const Matrix<float, 8, 1> &x, const Matrix<float, 4, 1> &u,
			      const AdmittanceParameters &params),
	const Matrix<float, 8, 1> &y0,
	const Matrix<float, 4, 1> &u,
	float t0,
	float tf,
	float h0,
	Matrix<float, 8, 1> &y1,
	const AdmittanceParameters &params
)
{
	// https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods
	float t1 = t0;
	y1 = y0;
	float h = h0;
	Vector<float, 8> k1, k2, k3, k4;

	if (tf < t0) { return -1; } // make sure t1 > t0

	while (t1 < tf) {
		if (t1 + h0 < tf) {
			h = h0;

		} else {
			h = tf - t1;
		}

		k1 = f(t1, y1, u, params);
		k2 = f(t1 + h / 2, y1 + k1 * h / 2, u, params);
		k3 = f(t1 + h / 2, y1 + k2 * h / 2, u, params);
		k4 = f(t1 + h, y1 + k3 * h, u, params);
		y1 += (k1 + k2 * 2 + k3 * 2 + k4) * (h / 6);
		t1 += h;
	}

	return 0;
}
