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

#include <gtest/gtest.h>
#include <AlignmentEstimator.hpp>

using namespace matrix;

namespace
{
const Vector3f HOVER_THRUST(0.f, 0.f, -10.2f); // body-frame actuator force in hover

// Residual a vehicle would actually produce: a body-fixed misalignment plus a
// world-fixed external force, seen in the body frame.
Vector3f makeResidual(const Vector2f &eps, const Vector2f &wind_ned, const Quatf &q)
{
	const Vector3f eps3(eps(0), eps(1), 0.f);
	const Vector3f from_misalignment = eps3 % HOVER_THRUST;           // eps x F
	const Vector3f wind_body = Dcmf(q).transpose() * Vector3f(wind_ned(0), wind_ned(1), 0.f);
	return from_misalignment + wind_body;
}

// Feed the estimator a hover at a set of headings.
void flyHeadings(AlignmentEstimator &est, const Vector2f &eps, const Vector2f &wind,
		 float yaw_start_deg, float yaw_end_deg, int steps)
{
	for (int i = 0; i < steps; i++) {
		const float frac = (steps > 1) ? (float)i / (float)(steps - 1) : 0.f;
		const float yaw = math::radians(yaw_start_deg + frac * (yaw_end_deg - yaw_start_deg));
		const Quatf q(Eulerf(0.f, 0.f, yaw));
		est.update(makeResidual(eps, wind, q), HOVER_THRUST, q);
	}
}
}

TEST(AlignmentEstimatorTest, RecoversMisalignmentAndForceOverAFullSweep)
{
	AlignmentEstimator est;
	const Vector2f eps(math::radians(3.0f), math::radians(-0.4f));
	const Vector2f wind(0.35f, -0.20f);

	flyHeadings(est, eps, wind, -180.f, 180.f, 2000);

	EXPECT_TRUE(est.isValid());
	EXPECT_NEAR(est.getMisalignment()(0), eps(0), math::radians(0.15f));
	EXPECT_NEAR(est.getMisalignment()(1), eps(1), math::radians(0.15f));
	EXPECT_NEAR(est.getExternalForce()(0), wind(0), 0.05f);
	EXPECT_NEAR(est.getExternalForce()(1), wind(1), 0.05f);
}

TEST(AlignmentEstimatorTest, CorrectionIsTheNegatedMisalignment)
{
	// The parameter to write is the rotation that undoes the misalignment.
	AlignmentEstimator est;
	const Vector2f eps(math::radians(3.0f), math::radians(-0.4f));
	flyHeadings(est, eps, Vector2f(), -180.f, 180.f, 2000);

	EXPECT_NEAR(est.getAlignmentCorrection()(0), -eps(0), math::radians(0.15f));
	EXPECT_NEAR(est.getAlignmentCorrection()(1), -eps(1), math::radians(0.15f));
}

TEST(AlignmentEstimatorTest, RejectsAnEstimateTakenAtOneHeading)
{
	// At a fixed heading a misalignment and an external force are the same two
	// degrees of freedom. The estimator must not claim a result.
	AlignmentEstimator est;
	flyHeadings(est, Vector2f(math::radians(3.0f), 0.f), Vector2f(), -90.f, -90.f, 2000);

	EXPECT_FALSE(est.hasEnoughRotation());
	EXPECT_FALSE(est.isValid());
	EXPECT_LT(est.getCoverageDeg(), AlignmentEstimator::MIN_COVERAGE_DEG);
}

TEST(AlignmentEstimatorTest, ExternalForceLeaksIntoMisalignmentWithoutRotation)
{
	// Why the gate exists. A pure external force at a single heading is partly
	// attributed to misalignment, because at one heading nothing in the data can
	// tell them apart - only the priors decide, and they cannot be right in general.
	// Rotate, and the same force is attributed correctly.
	const Vector2f wind(0.5f, 0.f);

	AlignmentEstimator fixed_heading;
	flyHeadings(fixed_heading, Vector2f(), wind, -90.f, -90.f, 2000);

	AlignmentEstimator rotating;
	flyHeadings(rotating, Vector2f(), wind, -180.f, 180.f, 2000);

	EXPECT_GT(fabsf(fixed_heading.getMisalignment()(0)), math::radians(0.1f));
	EXPECT_LT(fabsf(rotating.getMisalignment()(0)), math::radians(0.1f));

	// The leak is bounded by the priors rather than unbounded, but it is still an
	// error of the same order as the misalignments being chased, so the estimate
	// must not be usable without rotation.
	EXPECT_FALSE(fixed_heading.isValid());
	EXPECT_TRUE(rotating.isValid());
}

TEST(AlignmentEstimatorTest, HalfSweepIsEnoughToSeparateThem)
{
	AlignmentEstimator est;
	const Vector2f eps(math::radians(3.0f), 0.f);
	const Vector2f wind(0.4f, 0.f);
	flyHeadings(est, eps, wind, -90.f, 90.f, 2000);

	EXPECT_TRUE(est.hasEnoughRotation());
	EXPECT_GE(est.getCoverageDeg(), AlignmentEstimator::GOOD_COVERAGE_DEG);
	EXPECT_NEAR(est.getMisalignment()(0), eps(0), math::radians(0.3f));
	EXPECT_NEAR(est.getExternalForce()(0), wind(0), 0.1f);
}

TEST(AlignmentEstimatorTest, IgnoresSamplesWithoutThrust)
{
	AlignmentEstimator est;
	const Quatf q(Eulerf(0.f, 0.f, 0.f));

	for (int i = 0; i < 100; i++) {
		est.update(Vector3f(0.1f, 0.2f, 0.f), Vector3f(0.f, 0.f, 0.f), q);
	}

	EXPECT_EQ(est.getSampleCount(), 0);
	EXPECT_EQ(est.getMisalignment(), Vector2f());
}

TEST(AlignmentEstimatorTest, ResetClearsEstimateAndCoverage)
{
	AlignmentEstimator est;
	flyHeadings(est, Vector2f(math::radians(3.0f), 0.f), Vector2f(), -180.f, 180.f, 500);
	ASSERT_GT(est.getCoverageDeg(), 0.f);

	est.reset();

	EXPECT_EQ(est.getMisalignment(), Vector2f());
	EXPECT_EQ(est.getExternalForce(), Vector2f());
	EXPECT_FLOAT_EQ(est.getCoverageDeg(), 0.f);
	EXPECT_EQ(est.getSampleCount(), 0);
	EXPECT_FALSE(est.isValid());
}
