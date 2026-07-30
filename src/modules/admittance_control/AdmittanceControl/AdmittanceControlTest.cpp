/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
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

#include <gtest/gtest.h>
#include <AdmittanceControl.hpp>

using namespace matrix;

class AdmittanceControlBasicTest : public ::testing::Test
{
public:
	AdmittanceControlBasicTest()
	{
		// Symmetric, easy to reason about defaults. Individual tests override
		// whichever entries they care about.
		for (int i = 0; i < 4; i++) {
			_bell_params.A[i] = 2.5f;
			_bell_params.B1[i] = 5.f;
			_bell_params.B2[i] = 3.5f;
			_bell_params.B3[i] = 2.f;
			_bell_params.M_min[i] = .1f;
			_bell_params.M_max[i] = 1.f;
			_bell_params.K_min[i] = 1.f;
			_bell_params.K_max[i] = 10.f;
		}

		_bell_params.lpf_sat_factor = 1.f;

		_admittance_control.setParams(_bell_params);
		_admittance_control.setLimits(10.f, 10.f);
	}

	/** Integrate for `seconds` at a fixed step with a constant wrench. */
	void run(float seconds, const Vector<float, 4> &wrench, float target_dist = 0.f)
	{
		const int steps = static_cast<int>(seconds / _dt);

		for (int i = 0; i < steps; i++) {
			_admittance_control.update(_dt, wrench, target_dist);
		}
	}

	AdmittanceControl _admittance_control;
	BellParameters _bell_params{};
	float _dt{0.01f};
};

TEST_F(AdmittanceControlBasicTest, BellCurveSchedulesGains)
{
	Vector<float, 4> wrench;
	wrench(0) = 0.f;
	wrench(1) = 1.f;
	wrench(2) = 2.f;
	wrench(3) = 5.f;

	_admittance_control.update(_dt, wrench, 0.f);

	const AdmittanceParameters params = _admittance_control.getAdmittanceParameters();

	// Zero wrench sits at the peak of the bell curve, so M and K take their maxima.
	EXPECT_FLOAT_EQ(params.K[0], 10.f);
	EXPECT_FLOAT_EQ(params.M[0], 1.f);

	// Growing wrench moves down the curve monotonically towards the minima.
	EXPECT_LT(params.K[1], params.K[0]);
	EXPECT_LT(params.K[2], params.K[1]);
	EXPECT_LT(params.K[3], params.K[2]);

	EXPECT_LT(params.M[1], params.M[0]);
	EXPECT_LT(params.M[2], params.M[1]);
	EXPECT_LT(params.M[3], params.M[2]);

	// Every gain stays inside the configured envelope.
	for (int i = 0; i < 4; i++) {
		EXPECT_GE(params.K[i], _bell_params.K_min[i]);
		EXPECT_LE(params.K[i], _bell_params.K_max[i]);
		EXPECT_GE(params.M[i], _bell_params.M_min[i]);
		EXPECT_LE(params.M[i], _bell_params.M_max[i]);

		// Critical damping.
		EXPECT_FLOAT_EQ(params.C[i], 2.f * sqrtf(params.K[i] * params.M[i]));
	}
}

TEST_F(AdmittanceControlBasicTest, ZeroWrenchProducesNoDeviation)
{
	const Vector<float, 4> wrench{};

	run(5.f, wrench);

	EXPECT_FLOAT_EQ(_admittance_control.getDeviation()(0), 0.f);
	EXPECT_FLOAT_EQ(_admittance_control.getDeviation()(1), 0.f);
	EXPECT_FLOAT_EQ(_admittance_control.getDeviation()(2), 0.f);
	EXPECT_FLOAT_EQ(_admittance_control.getYawDeviation(), 0.f);
	EXPECT_TRUE(_admittance_control.isFinite());
}

TEST_F(AdmittanceControlBasicTest, StepWrenchSettlesAtForceOverStiffness)
{
	// Fix the schedule so the steady state is analytic: with M and K constant the
	// deviation settles at We/K.
	for (int i = 0; i < 4; i++) {
		_bell_params.M_min[i] = 1.f;
		_bell_params.M_max[i] = 1.f;
		_bell_params.K_min[i] = 4.f;
		_bell_params.K_max[i] = 4.f;
	}

	_admittance_control.setParams(_bell_params);

	Vector<float, 4> wrench;
	wrench(0) = 2.f;
	wrench(1) = -1.f;
	wrench(2) = 0.f;
	wrench(3) = 0.4f;

	run(20.f, wrench);

	EXPECT_NEAR(_admittance_control.getDeviation()(0), 2.f / 4.f, 1e-3f);
	EXPECT_NEAR(_admittance_control.getDeviation()(1), -1.f / 4.f, 1e-3f);
	EXPECT_NEAR(_admittance_control.getDeviation()(2), 0.f, 1e-3f);
	EXPECT_NEAR(_admittance_control.getYawDeviation(), 0.4f / 4.f, 1e-3f);

	// Critically damped, so it comes to rest.
	EXPECT_NEAR(_admittance_control.getDeviationRate()(0), 0.f, 1e-3f);
	EXPECT_NEAR(_admittance_control.getDeviationRate()(1), 0.f, 1e-3f);
}

TEST_F(AdmittanceControlBasicTest, ResetClearsState)
{
	Vector<float, 4> wrench;
	wrench(0) = 2.f;

	run(2.f, wrench);
	EXPECT_GT(_admittance_control.getDeviation()(0), 0.f);

	_admittance_control.reset();

	EXPECT_FLOAT_EQ(_admittance_control.getDeviation()(0), 0.f);
	EXPECT_FLOAT_EQ(_admittance_control.getDeviationRate()(0), 0.f);
}

TEST_F(AdmittanceControlBasicTest, SetParamsDoesNotDisturbState)
{
	Vector<float, 4> wrench;
	wrench(0) = 2.f;

	run(2.f, wrench);
	const float deviation_before = _admittance_control.getDeviation()(0);
	ASSERT_GT(deviation_before, 0.f);

	// A parameter write in flight must not step the setpoint.
	_admittance_control.setParams(_bell_params);

	EXPECT_FLOAT_EQ(_admittance_control.getDeviation()(0), deviation_before);
}

TEST_F(AdmittanceControlBasicTest, DeviationIsClamped)
{
	for (int i = 0; i < 4; i++) {
		_bell_params.K_min[i] = 0.1f;
		_bell_params.K_max[i] = 0.1f;
		_bell_params.M_min[i] = 1.f;
		_bell_params.M_max[i] = 1.f;
	}

	_admittance_control.setParams(_bell_params);
	_admittance_control.setLimits(0.3f, 0.2f);

	Vector<float, 4> wrench;
	wrench(0) = 10.f; // unclamped steady state would be 100 m

	run(10.f, wrench);

	EXPECT_LE(_admittance_control.getDeviation()(0), 0.3f + 1e-4f);
	EXPECT_LE(fabsf(_admittance_control.getDeviationRate()(0)), 0.2f + 1e-4f);
	EXPECT_TRUE(_admittance_control.isFinite());
}

TEST_F(AdmittanceControlBasicTest, YawDeviationIsClamped)
{
	for (int i = 0; i < 4; i++) {
		_bell_params.K_min[i] = 0.01f;
		_bell_params.K_max[i] = 0.01f;
		_bell_params.M_min[i] = 1.f;
		_bell_params.M_max[i] = 1.f;
	}

	_admittance_control.setParams(_bell_params);

	Vector<float, 4> wrench;
	wrench(3) = 5.f;

	run(20.f, wrench);

	EXPECT_LE(_admittance_control.getYawDeviation(), AdmittanceControl::kMaxYawDeviation + 1e-4f);
}

TEST_F(AdmittanceControlBasicTest, SaturationFactorStaysInRange)
{
	// Saturation is a magnitude and must never drive the bell denominator negative,
	// which would stiffen rather than soften the response.
	_admittance_control.updateSaturation(_dt, -5.f);
	EXPECT_GE(_admittance_control.getSaturationFactor(), 0.f);

	for (int i = 0; i < 1000; i++) {
		_admittance_control.updateSaturation(_dt, 5.f);
	}

	EXPECT_LE(_admittance_control.getSaturationFactor(), 1.f);
	EXPECT_GE(_admittance_control.getSaturationFactor(), 0.f);
}

TEST_F(AdmittanceControlBasicTest, SurvivesDegenerateBellParameters)
{
	// A -> 0 used to divide by zero once the saturation factor caught up with it,
	// and B2 > B1 drives the exponent negative.
	for (int i = 0; i < 4; i++) {
		_bell_params.A[i] = 0.01f;
		_bell_params.B1[i] = 1.f;
		_bell_params.B2[i] = 5.f;
		_bell_params.M_min[i] = 0.f;
		_bell_params.M_max[i] = 0.f;
	}

	_bell_params.lpf_sat_factor = 0.f;
	_admittance_control.setParams(_bell_params);
	_admittance_control.updateSaturation(_dt, 1.f);

	Vector<float, 4> wrench;
	wrench(0) = 0.f; // exercises powf(0, negative)
	wrench(1) = 3.f;

	run(1.f, wrench);

	EXPECT_TRUE(_admittance_control.isFinite());
}
