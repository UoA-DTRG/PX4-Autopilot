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
 * @file AdmittanceControl.hpp
 *
 * Admittance controller expressed as a deviation from the commanded setpoint.
 *
 * The vehicle is modelled as a virtual mass-spring-damper driven by the
 * estimated external wrench. Writing the classical form
 *
 *     M x_ddot + C x_dot + K x = We + Fd,  Fd = M a_sp + C v_sp + K p_sp
 *
 * with x = p_sp + d (the commanded setpoint plus a deviation) cancels the feed
 * forward term exactly and leaves
 *
 *     M d_ddot + C d_dot + K d = We
 *
 * so this class only ever integrates the deviation. That keeps the state a
 * relative quantity (meaningful to rotate), makes reset a true zero, and lets
 * the caller add the result to whatever reference it has, including one that is
 * only velocity controlled.
 *
 * M and K are gain scheduled by a generalised bell curve on the external wrench
 * magnitude and the distance to the target; C is chosen for critical damping.
 *
 * All quantities are in the yaw-aligned heading frame. The caller is
 * responsible for rotating the wrench in and the deviation out.
 */

#pragma once

#include <float.h>

#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>

struct AdmittanceParameters {
	float M[4];	///< virtual mass       [x y z yaw]
	float C[4];	///< virtual damping    [x y z yaw]
	float K[4];	///< virtual stiffness  [x y z yaw]
};

struct BellParameters {
	float K_max[4];
	float K_min[4];
	float M_max[4];
	float M_min[4];
	float A[4];
	float B1[4];
	float B2[4];
	float B3[4];
	float lpf_sat_factor;
};

class AdmittanceControl
{
public:
	AdmittanceControl() = default;
	~AdmittanceControl() = default;

	/** Smallest permitted bell-curve denominator, guards A - sat_factor -> 0. */
	static constexpr float kMinBellDenominator = 0.05f;
	/** Smallest permitted bell-curve exponent, guards powf(0, negative). */
	static constexpr float kMinBellExponent = 0.01f;
	/** Smallest permitted virtual mass, guards the division in the ODE. */
	static constexpr float kMinMass = 0.01f;
	/** Yaw deviation clamp [rad]. */
	static constexpr float kMaxYawDeviation = 0.785398f; // pi/4

	/**
	 * Update the bell-curve parameters without disturbing the integrator state.
	 * Safe to call at any time, including in flight.
	 */
	void setParams(const BellParameters &bell_params) { _params_bell = bell_params; }

	/** Zero the deviation state. The controller resumes from the reference. */
	void reset();

	/** Clamps applied to the deviation and its rate after every integration step. */
	void setLimits(float deviation_max, float deviation_rate_max)
	{
		_deviation_max = deviation_max;
		_deviation_rate_max = deviation_rate_max;
	}

	/**
	 * Low-pass the raw actuator saturation measure.
	 *
	 * @param dt      time step [s]
	 * @param raw_sat 0 when actuators sit mid-range, 1 at either rail
	 */
	void updateSaturation(float dt, float raw_sat);

	/**
	 * Integrate one step of the deviation dynamics.
	 *
	 * @param dt          time step [s]
	 * @param We          external wrench in the heading frame [fx fy fz mz]
	 * @param target_dist distance to the interaction target [m]
	 */
	void update(float dt, const matrix::Vector<float, 4> &We, float target_dist);

	matrix::Vector3f getDeviation() const { return matrix::Vector3f(_y(0), _y(2), _y(4)); }
	matrix::Vector3f getDeviationRate() const { return matrix::Vector3f(_y(1), _y(3), _y(5)); }
	matrix::Vector3f getDeviationAccel() const { return matrix::Vector3f(_accel(0), _accel(1), _accel(2)); }
	float getYawDeviation() const { return _y(6); }
	float getYawDeviationRate() const { return _y(7); }

	AdmittanceParameters getAdmittanceParameters() const { return _params; }
	float getSaturationFactor() const { return _sat_factor; }

	/** True when every state and scheduled gain is finite. */
	bool isFinite() const;

private:
	void scheduleGains(const matrix::Vector<float, 4> &We, float target_dist);

	matrix::Vector<float, 8> _integrate(float dt, const matrix::Vector<float, 4> &We);
	void applyLimits();

	static int _integrate_rk4(
		matrix::Vector<float, 8> (*f)(float, const matrix::Matrix<float, 8, 1> &x,
					      const matrix::Matrix<float, 4, 1> &u, const AdmittanceParameters &params),
		const matrix::Matrix<float, 8, 1> &y0,
		const matrix::Matrix<float, 4, 1> &u,
		float t0,
		float tf,
		float h0,
		matrix::Matrix<float, 8, 1> &y1,
		const AdmittanceParameters &params);

	/// [dx, dx_dot, dy, dy_dot, dz, dz_dot, dyaw, dyaw_dot] in the heading frame
	matrix::Vector<float, 8> _y{};
	/// deviation accelerations [x y z yaw]
	matrix::Vector<float, 4> _accel{};

	AdmittanceParameters _params{};
	BellParameters _params_bell{};

	float _sat_factor{0.f};
	float _deviation_max{1.f};
	float _deviation_rate_max{1.f};
};
