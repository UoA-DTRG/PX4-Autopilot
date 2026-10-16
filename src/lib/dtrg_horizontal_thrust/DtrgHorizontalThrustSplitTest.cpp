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
 * @file DtrgHorizontalThrustSplitTest.cpp
 *
 * How Position / Offboard divide the position controller's horizontal thrust between
 * horizontal thrust and tilt for every DTRG_HT_MASK and DTRG_HT_SPLIT. The split there
 * is not a single multiplication: the vehicle tilts for part of the thrust and the full
 * thrust is rotated into the tilted body frame, so this runs the HT branch of
 * MulticopterPositionControl::Run() with the real ControlMath and checks the forces
 * that result.
 */

#include <gtest/gtest.h>

#include <math.h>

#include <ControlMath.hpp>
#include <matrix/math.hpp>
#include <uORB/topics/vehicle_attitude_setpoint.h>

#include "dtrg_horizontal_thrust.hpp"

using namespace dtrg_ht;
using namespace matrix;

namespace
{

constexpr float kHtMax = 0.5f; // DTRG_HT_MAX

// a hover with ~10 deg worth of horizontal demand, in the heading frame
constexpr float kForward = 0.08f;
constexpr float kRight = -0.05f;
constexpr float kDown = -0.5f;

constexpr float kSplits[] = {0.f, 0.25f, 0.5f, 0.8f, 1.f};
constexpr float kYaws[] = {0.f, 0.7f, M_PI_2_F, -2.5f};

// second order effects of the tilt; still far below the difference between two splits
constexpr float kTol = 2e-3f;

struct Setpoint {
	Quatf q;
	Vector3f thrust_body;
	HorizontalThrust ht;
};

/// the HT branch of MulticopterPositionControl::Run(), step by step
Setpoint positionHtSetpoint(int32_t mask, bool split_en, float split, const Vector3f &thrust, float yaw,
			    float ht_roll = 0.f, float ht_pitch = 0.f)
{
	vehicle_attitude_setpoint_s attitude_RP{};

	if (split_en) {
		const NorthEast thrust_tilt = tiltThrust(mask, split_en, split, thrust(0), thrust(1), yaw);
		ControlMath::thrustToAttitude(Vector3f(thrust_tilt.north, thrust_tilt.east, thrust(2)), yaw, attitude_RP);

	} else {
		// PositionControl::getAttitudeSetpoint()
		ControlMath::thrustToAttitude(thrust, yaw, attitude_RP);
	}

	const Eulerf euler_RP(Quatf(attitude_RP.q_d));
	const Tilt tilt = positionControlTilt(mask, split_en, ht_roll, ht_pitch, euler_RP.phi(), euler_RP.theta());

	Setpoint sp;
	sp.q = Quatf(Eulerf(tilt.roll, tilt.pitch, yaw));
	const Vector3f thrust_frd = sp.q.rotateVectorInverse(thrust);
	sp.ht = horizontalThrust(mask, thrust_frd(0), thrust_frd(1), kHtMax);
	sp.thrust_body = Vector3f(sp.ht.x, sp.ht.y, thrust_frd(2));
	return sp;
}

/// NED thrust of a demand given in the heading frame
Vector3f fromHeading(float forward, float right, float down, float yaw)
{
	return Vector3f(cosf(yaw) * forward - sinf(yaw) * right, sinf(yaw) * forward + cosf(yaw) * right, down);
}

/// a NED vector in the heading frame: forward, right, down
Vector3f toHeading(const Vector3f &ned, float yaw)
{
	return Vector3f(cosf(yaw) * ned(0) + sinf(yaw) * ned(1), -sinf(yaw) * ned(0) + cosf(yaw) * ned(1), ned(2));
}

/// force from tilting the collective thrust, in the heading frame
Vector3f tiltForce(const Setpoint &sp, float yaw)
{
	return toHeading(sp.q.rotateVector(Vector3f(0.f, 0.f, sp.thrust_body(2))), yaw);
}

/// force from horizontal thrust, in the heading frame
Vector3f htForce(const Setpoint &sp, float yaw)
{
	return toHeading(sp.q.rotateVector(Vector3f(sp.thrust_body(0), sp.thrust_body(1), 0.f)), yaw);
}

} // namespace

TEST(DtrgHorizontalThrustSplit, SplitDividesTheDemand)
{
	// on an HT axis horizontal thrust gives DTRG_HT_SPLIT of the demand and tilting the rest;
	// the other axis is moved by tilting only. Together they deliver the whole demand.
	for (int32_t mask = 0; mask <= kMaxSelectableMask; mask++) {
		for (const float split : kSplits) {
			for (const float yaw : kYaws) {
				SCOPED_TRACE(testing::Message() << "mask " << mask << " split " << split << " yaw " << yaw);

				const Setpoint sp = positionHtSetpoint(mask, true, split, fromHeading(kForward, kRight, kDown, yaw), yaw);
				const Vector3f tilt = tiltForce(sp, yaw);
				const Vector3f ht = htForce(sp, yaw);

				const float x_share = maskUsesX(mask) ? split : 0.f;
				const float y_share = maskUsesY(mask) ? split : 0.f;

				EXPECT_NEAR(ht(0), x_share * kForward, kTol);
				EXPECT_NEAR(tilt(0), (1.f - x_share) * kForward, kTol);
				EXPECT_NEAR(ht(1), y_share * kRight, kTol);
				EXPECT_NEAR(tilt(1), (1.f - y_share) * kRight, kTol);

				const Vector3f total = tilt + ht;
				EXPECT_NEAR(total(0), kForward, kTol);
				EXPECT_NEAR(total(1), kRight, kTol);
				EXPECT_NEAR(total(2), kDown, kTol);

				EXPECT_FALSE(sp.ht.x_sat);
				EXPECT_FALSE(sp.ht.y_sat);
			}
		}
	}
}

TEST(DtrgHorizontalThrustSplit, SplitKeepsTheHtAxesFromTheAuxTilt)
{
	// with the split the HT axes tilt with the controller: an aux / offboard tilt changes nothing
	for (int32_t mask = 0; mask <= kMaxSelectableMask; mask++) {
		const Vector3f thrust = fromHeading(kForward, kRight, kDown, 0.7f);
		const Setpoint level = positionHtSetpoint(mask, true, 0.5f, thrust, 0.7f);
		const Setpoint aux = positionHtSetpoint(mask, true, 0.5f, thrust, 0.7f, 0.15f, -0.15f);
		EXPECT_TRUE(isEqual(level.q, aux.q)) << "mask " << mask;
		EXPECT_TRUE(isEqual(level.thrust_body, aux.thrust_body)) << "mask " << mask;
	}
}

TEST(DtrgHorizontalThrustSplit, WithoutSplitHtAxesAreThrustOnly)
{
	// without the split an HT axis stays at the HT tilt (level here) and horizontal thrust gives
	// the whole demand; the other axis tilts for all of it, whatever DTRG_HT_SPLIT says
	for (int32_t mask = 0; mask <= kMaxSelectableMask; mask++) {
		for (const float yaw : kYaws) {
			SCOPED_TRACE(testing::Message() << "mask " << mask << " yaw " << yaw);

			const Setpoint sp = positionHtSetpoint(mask, false, 0.2f, fromHeading(kForward, kRight, kDown, yaw), yaw);
			const Vector3f tilt = tiltForce(sp, yaw);
			const Vector3f ht = htForce(sp, yaw);
			const Eulerf euler(sp.q);

			if (maskUsesX(mask)) {
				EXPECT_NEAR(euler.theta(), 0.f, 1e-6f);
				EXPECT_NEAR(ht(0), kForward, kTol);
				EXPECT_NEAR(tilt(0), 0.f, kTol);

			} else {
				EXPECT_NEAR(ht(0), 0.f, kTol);
				EXPECT_NEAR(tilt(0), kForward, kTol);
			}

			if (maskUsesY(mask)) {
				EXPECT_NEAR(euler.phi(), 0.f, 1e-6f);
				EXPECT_NEAR(ht(1), kRight, kTol);
				EXPECT_NEAR(tilt(1), 0.f, kTol);

			} else {
				EXPECT_NEAR(ht(1), 0.f, kTol);
				EXPECT_NEAR(tilt(1), kRight, kTol);
			}
		}
	}
}

TEST(DtrgHorizontalThrustSplit, SplitZeroIsNoHorizontalThrust)
{
	// DTRG_HT_SPLIT_EN=1, DTRG_HT_SPLIT=0 (the old mask 3): the vehicle tilts for everything,
	// as without HT, and no horizontal thrust is commanded
	for (int32_t mask = 0; mask <= kMaxSelectableMask; mask++) {
		for (const float yaw : kYaws) {
			const Vector3f thrust = fromHeading(kForward, kRight, kDown, yaw);
			const Setpoint sp = positionHtSetpoint(mask, true, 0.f, thrust, yaw);

			vehicle_attitude_setpoint_s standard{};
			ControlMath::thrustToAttitude(thrust, yaw, standard);

			EXPECT_NEAR(sp.thrust_body(0), 0.f, 1e-5f) << "mask " << mask << " yaw " << yaw;
			EXPECT_NEAR(sp.thrust_body(1), 0.f, 1e-5f) << "mask " << mask << " yaw " << yaw;
			EXPECT_NEAR(sp.thrust_body(2), standard.thrust_body[2], 1e-5f);
			EXPECT_TRUE(isEqual(Dcmf(sp.q), Dcmf(Quatf(standard.q_d)), 1e-5f)) << "mask " << mask << " yaw " << yaw;
		}
	}
}

TEST(DtrgHorizontalThrustSplit, SplitOneIsLevelOnTheHtAxes)
{
	// split 1: horizontal thrust only on the HT axes, which stay level
	for (int32_t mask = 0; mask <= kMaxSelectableMask; mask++) {
		const Setpoint sp = positionHtSetpoint(mask, true, 1.f, fromHeading(kForward, kRight, kDown, 0.f), 0.f);
		const Eulerf euler(sp.q);

		if (maskUsesX(mask)) {
			EXPECT_NEAR(euler.theta(), 0.f, 1e-6f) << "mask " << mask;
			EXPECT_NEAR(sp.thrust_body(0), kForward, kTol) << "mask " << mask;
		}

		if (maskUsesY(mask)) {
			EXPECT_NEAR(euler.phi(), 0.f, 1e-6f) << "mask " << mask;
			EXPECT_NEAR(sp.thrust_body(1), kRight, kTol) << "mask " << mask;
		}
	}
}

TEST(DtrgHorizontalThrustSplit, ThrustShareIsLimitedButTiltShareIsNot)
{
	// a demand whose thrust share exceeds DTRG_HT_MAX: horizontal thrust is clipped and flagged,
	// the tilt still gives its share. Split 0.8 of 0.7 forward is ~0.54 of body X thrust at the
	// ~16 deg tilt for the other 0.14.
	const float forward = 0.7f;
	const float split = 0.8f;
	const Setpoint sp = positionHtSetpoint(0, true, split, Vector3f(forward, 0.f, kDown), 0.f);

	EXPECT_FLOAT_EQ(sp.ht.x, kHtMax);
	EXPECT_TRUE(sp.ht.x_sat);
	EXPECT_FALSE(sp.ht.y_sat);
	EXPECT_NEAR(Eulerf(sp.q).theta(), -atan2f((1.f - split) * forward, -kDown), 1e-5f);
}

TEST(DtrgHorizontalThrustSplit, SplitIsExactOnlyForSmallTilts)
{
	// The split is applied to the thrust the vehicle tilts for, and the full thrust is then
	// rotated into the tilted body, so the horizontal thrust share shrinks as the tilt grows:
	// split 0.5 of 1.2 forward over a 0.5 hover tilts 50 deg and leaves 0.38 of body X thrust,
	// not 0.6. Pinned so a change to this behaviour is noticed.
	const Setpoint sp = positionHtSetpoint(0, true, 0.5f, Vector3f(1.2f, 0.f, kDown), 0.f);
	EXPECT_NEAR(sp.ht.x, 0.384f, 1e-3f);
	EXPECT_FALSE(sp.ht.x_sat);
	EXPECT_NEAR(htForce(sp, 0.f)(0) + tiltForce(sp, 0.f)(0), 1.2f, 1e-5f);
}
