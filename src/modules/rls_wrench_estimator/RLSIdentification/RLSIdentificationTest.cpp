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
#include <cmath>
#include <RLSIdentification.hpp>

using namespace matrix;

class RLSIdentificationBasicTest : public ::testing::Test
{
public:
	RLSIdentificationBasicTest()
	{
		VehicleParameters params{};
		params.mass = 1.04f;
		params.num_rotors = 4;
		params.n_groups = 1;
		params.lpf_motor_tau = 0.1f;

		for (int i = 0; i < RLS_MAX_ROTORS; i++) {
			params.axis[i] = Vector3f(0.f, 0.f, -1.f);   // thrust straight up (FRD -z)
			params.thrust_coef[i] = 1.f;
			params.moment_ratio[i] = 0.05f;
			params.motor_group[i] = 0;                   // single shared thrust group
			params.position[i] = Vector3f();
		}

		// Symmetric quad-X arm positions
		params.position[0] = Vector3f(0.2f, 0.2f, 0.f);
		params.position[1] = Vector3f(-0.2f, -0.2f, 0.f);
		params.position[2] = Vector3f(0.2f, -0.2f, 0.f);
		params.position[3] = Vector3f(-0.2f, 0.2f, 0.f);

		const float x_init[3] = {1.2f, 0.f, 0.f};
		const float x_confidence[3] = {0.001f, 1000.f, 1000.f};
		const float R_diag[5] = {1.f, 1.f, 10.f, 1.f, 1.f};
		_rls_identification.initialize(x_init, x_confidence, R_diag, params);
	}

	RLSIdentification _rls_identification;

	float _dt{0.004f};
	Vector<float, RLS_MAX_ROTORS> _speeds{};
	Vector3f _acc{};
	Quatf _q;

};


TEST_F(RLSIdentificationBasicTest, Test1)
{
	_speeds.setAll(0.f);
	_speeds(0) = 500.f;
	_speeds(1) = 500.f;
	_speeds(2) = 500.f;
	_speeds(3) = 500.f;
	_dt = 0.004f;
	_acc(0) = 0.f;
	_acc(1) = 0.f;
	_acc(2) = -9.80665f;
	_q = Quatf(1.f, 0.f, 0.f, 0.f);

	Vector<float, RLS_MAX_GROUPS> x;
	Vector3f fi;

	for (size_t i = 0; i < 100000; i++) {
		_rls_identification.updateThrust(_acc * 1.04f, _speeds, _dt, false, true);
		_rls_identification.updateOffset(_q, false);
		x = _rls_identification.getEstimationThrust();
		fi = _rls_identification.getActuatorForceVector();
	}

	// Purely vertical thrust axis => no body x/y force
	EXPECT_FLOAT_EQ(fi(0), 0.f);
	EXPECT_FLOAT_EQ(fi(1), 0.f);
	// Thrust points up (negative z in FRD)
	EXPECT_LT(fi(2), 0.f);
	// Estimated group thrust constant is finite
	EXPECT_TRUE(std::isfinite(x(0)));
}
