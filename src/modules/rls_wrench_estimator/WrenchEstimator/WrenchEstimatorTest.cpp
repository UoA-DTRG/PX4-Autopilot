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
#include <WrenchEstimator.hpp>

using namespace matrix;

namespace
{
constexpr float TAU = 1.f;                       // force/moment estimator time constant [s]
const Vector3f INERTIA(0.05f, 0.06f, 0.09f);     // [kg m^2]

// Run the moment observer for `duration` seconds at a fixed step.
Vector3f runMoment(WrenchEstimator &estimator, const Vector3f &prediction_error, const Vector3f &omega,
		   float dt, float duration)
{
	for (int i = 0; i < (int)(duration / dt); i++) {
		estimator.updateMoment(prediction_error, omega, dt, true);
	}

	return estimator.getExternalMoment();
}
}

TEST(WrenchEstimatorTest, ForceIsZeroWithoutInteraction)
{
	WrenchEstimator estimator;
	estimator.initialize(TAU, TAU, INERTIA);

	estimator.updateForce(Vector3f(1.f, 2.f, 3.f), 0.01f, false);
	estimator.updateMoment(Vector3f(1.f, 2.f, 3.f), Vector3f(0.1f, 0.2f, 0.3f), 0.01f, false);

	EXPECT_EQ(estimator.getExternalForce(), Vector3f());
	EXPECT_EQ(estimator.getExternalMoment(), Vector3f());
}

TEST(WrenchEstimatorTest, ForceConvergesToPredictionError)
{
	WrenchEstimator estimator;
	estimator.initialize(TAU, TAU, INERTIA);

	const Vector3f error(1.f, -2.f, 3.f);

	for (int i = 0; i < 2000; i++) { // 20 s = 20 time constants
		estimator.updateForce(error, 0.01f, true);
	}

	EXPECT_TRUE((estimator.getExternalForce() - error).norm() < 1e-3f);
}

TEST(WrenchEstimatorTest, MomentUsesAngularVelocity)
{
	// The angular velocity argument has to reach the momentum term: with the same
	// prediction error, a rotating vehicle must not give the same estimate as a
	// stationary one.
	WrenchEstimator still;
	still.initialize(TAU, TAU, INERTIA);
	const Vector3f m_still = runMoment(still, Vector3f(0.1f, 0.f, 0.f), Vector3f(), 0.01f, 1.f);

	WrenchEstimator rotating;
	rotating.initialize(TAU, TAU, INERTIA);
	const Vector3f m_rotating = runMoment(rotating, Vector3f(0.1f, 0.f, 0.f), Vector3f(0.f, 0.f, 2.f), 0.01f, 1.f);

	EXPECT_GT((m_rotating - m_still).norm(), 1e-3f);
}

TEST(WrenchEstimatorTest, MomentIsIndependentOfUpdateRate)
{
	// The momentum term belongs to the output of the observer. If it is folded into
	// the integrator state instead, halving the step size doubles how often it is
	// applied and the estimate changes with the scheduling rate.
	const Vector3f error(0.05f, -0.02f, 0.01f);
	const Vector3f omega(0.3f, -0.2f, 0.5f);

	WrenchEstimator slow;
	slow.initialize(TAU, TAU, INERTIA);
	const Vector3f m_slow = runMoment(slow, error, omega, 0.01f, 2.f);

	WrenchEstimator fast;
	fast.initialize(TAU, TAU, INERTIA);
	const Vector3f m_fast = runMoment(fast, error, omega, 0.001f, 2.f);

	EXPECT_TRUE((m_fast - m_slow).norm() < 1e-3f) << "slow: " << m_slow(0) << " fast: " << m_fast(0);
}

TEST(WrenchEstimatorTest, MomentResetsWhenInteractionEnds)
{
	WrenchEstimator estimator;
	estimator.initialize(TAU, TAU, INERTIA);

	runMoment(estimator, Vector3f(0.2f, 0.1f, -0.1f), Vector3f(0.1f, 0.f, 0.f), 0.01f, 1.f);
	ASSERT_GT(estimator.getExternalMoment().norm(), 1e-4f);

	estimator.updateMoment(Vector3f(0.2f, 0.1f, -0.1f), Vector3f(0.1f, 0.f, 0.f), 0.01f, false);
	EXPECT_EQ(estimator.getExternalMoment(), Vector3f());

	// The integrator state has to be cleared with it, otherwise the next interaction
	// resumes from where the previous one left off.
	estimator.updateMoment(Vector3f(), Vector3f(), 0.01f, true);
	EXPECT_TRUE(estimator.getExternalMoment().norm() < 1e-6f);
}
