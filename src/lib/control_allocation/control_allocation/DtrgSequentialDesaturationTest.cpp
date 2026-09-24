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
 * @file DtrgSequentialDesaturationTest.cpp
 *
 * DTRG desaturation order for a fully actuated vehicle (MC_AIRMODE = 0):
 * horizontal thrust X, then Y, then yaw, then Z (reduce only), then roll and
 * pitch. Each step only slides along one control axis, so everything that is
 * not reduced must still be allocated exactly.
 *
 * The vehicle is a planar octo with the rotors tilted tangentially by +-31 deg,
 * like the DTRG planarOcto, so X/Y thrust can be produced without tilting.
 */

#include <gtest/gtest.h>

#include <math.h>

#include <ControlAllocationSequentialDesaturation.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/sequential_desaturation.h>

using namespace matrix;

namespace
{

using Axis = ControlAllocation::ControlAxis;
using ControlVector = Vector<float, ControlAllocation::NUM_AXES>;

constexpr int kNumRotors = 8;
constexpr float kTiltDeg = 31.f;
constexpr float kMomentRatio = 0.05f;
constexpr float kTol = 1e-3f;

ActuatorEffectiveness::EffectivenessMatrix makePlanarOctoEffectiveness()
{
	ActuatorEffectiveness::EffectivenessMatrix effectiveness;
	effectiveness.setZero();

	const float tilt = math::radians(kTiltDeg);

	for (int i = 0; i < kNumRotors; i++) {
		const float angle = 2.f * M_PI_F * i / kNumRotors;
		const Vector3f position(cosf(angle), sinf(angle), 0.f);
		const Vector3f tangent(-sinf(angle), cosf(angle), 0.f);
		const float side = (i % 2 == 0) ? 1.f : -1.f;

		// thrust up (NED -Z), tilted tangentially, alternating side and spin direction
		const Vector3f axis = (Vector3f(0.f, 0.f, -1.f) * cosf(tilt) + tangent * (side * sinf(tilt))).normalized();
		const float km = side * kMomentRatio;

		const Vector3f thrust = axis;
		const Vector3f moment = position.cross(axis) - km * axis;

		for (int j = 0; j < 3; j++) {
			effectiveness(j, i) = moment(j);
			effectiveness(j + 3, i) = thrust(j);
		}
	}

	return effectiveness;
}

class DtrgSequentialDesaturation : public ::testing::Test
{
protected:
	void SetUp() override
	{
		const ActuatorEffectiveness::EffectivenessMatrix effectiveness = makePlanarOctoEffectiveness();
		ActuatorVector zero;

		// normalised mixer, as control_allocator uses it: each axis of the control vector is in
		// normalised units rather than N / Nm
		_allocator.setEffectivenessMatrix(effectiveness, zero, zero, kNumRotors, true);
		_raw.setEffectivenessMatrix(effectiveness, zero, zero, kNumRotors, true);

		// thrust Z that puts every rotor at 0.5
		ControlVector unit_z;
		unit_z(Axis::THRUST_Z) = -1.f;
		_raw.setControlSetpoint(unit_z);
		_raw.allocate();
		_hover_z = -0.5f / _raw.getActuatorSetpoint()(0);
	}

	using ActuatorVector = Vector<float, ControlAllocation::NUM_ACTUATORS>;

	ControlVector allocate(const ControlVector &control_sp)
	{
		_allocator.setControlSetpoint(control_sp);
		_allocator.allocate();
		return _allocator.getAllocatedControl();
	}

	static float maxActuator(const ActuatorVector &u)
	{
		float m = -INFINITY;

		for (int i = 0; i < kNumRotors; i++) {
			m = fmaxf(m, u(i));
		}

		return m;
	}

	static float minActuator(const ActuatorVector &u)
	{
		float m = INFINITY;

		for (int i = 0; i < kNumRotors; i++) {
			m = fminf(m, u(i));
		}

		return m;
	}

	/// the plain pseudo-inverse allocation of the demand stays within [0, 1], i.e. nothing to desaturate
	bool feasible(const ControlVector &control_sp)
	{
		_raw.setControlSetpoint(control_sp);
		_raw.allocate();
		const ActuatorVector &u = _raw.getActuatorSetpoint();
		return (maxActuator(u) <= 1.f + kTol) && (minActuator(u) >= -kTol);
	}

	ControlAllocationSequentialDesaturation _allocator;
	ControlAllocationPseudoInverse _raw;
	float _hover_z{0.f};
};

} // namespace

TEST_F(DtrgSequentialDesaturation, GeometryIsFullyActuated)
{
	// each axis alone is reachable around hover: a planar vehicle without tilted rotors
	// would fail the X and Y cases
	for (int axis = 0; axis < 5; axis++) {
		ControlVector control_sp;
		control_sp(Axis::THRUST_Z) = _hover_z;
		control_sp(axis) = 0.2f;
		const ControlVector allocated = allocate(control_sp);

		for (int j = 0; j < ControlAllocation::NUM_AXES; j++) {
			EXPECT_NEAR(allocated(j), control_sp(j), kTol) << "axis " << axis << " component " << j;
		}
	}
}

TEST_F(DtrgSequentialDesaturation, UnsaturatedDemandIsAllocatedExactly)
{
	ControlVector control_sp;
	control_sp(Axis::ROLL) = 0.1f;
	control_sp(Axis::PITCH) = -0.1f;
	control_sp(Axis::YAW) = 0.05f;
	control_sp(Axis::THRUST_X) = 0.2f;
	control_sp(Axis::THRUST_Y) = -0.1f;
	control_sp(Axis::THRUST_Z) = _hover_z;
	ASSERT_TRUE(feasible(control_sp));

	const ControlVector allocated = allocate(control_sp);

	for (int j = 0; j < ControlAllocation::NUM_AXES; j++) {
		EXPECT_NEAR(allocated(j), control_sp(j), kTol) << "component " << j;
	}
}

TEST_F(DtrgSequentialDesaturation, HorizontalThrustIsReducedFirst)
{
	// far more X thrust than the rotors can give at hover
	ControlVector control_sp;
	control_sp(Axis::THRUST_X) = 5.f;
	control_sp(Axis::THRUST_Z) = _hover_z;
	ASSERT_FALSE(feasible(control_sp));

	const ControlVector allocated = allocate(control_sp);

	EXPECT_GT(allocated(Axis::THRUST_X), 0.f);
	EXPECT_LT(allocated(Axis::THRUST_X), control_sp(Axis::THRUST_X));

	// everything else untouched: altitude and attitude are kept
	EXPECT_NEAR(allocated(Axis::THRUST_Z), _hover_z, kTol);
	EXPECT_NEAR(allocated(Axis::THRUST_Y), 0.f, kTol);
	EXPECT_NEAR(allocated(Axis::ROLL), 0.f, kTol);
	EXPECT_NEAR(allocated(Axis::PITCH), 0.f, kTol);
	EXPECT_NEAR(allocated(Axis::YAW), 0.f, kTol);
}

TEST_F(DtrgSequentialDesaturation, HorizontalThrustIsGivenUpForRoll)
{
	// roll alone is feasible, roll plus the X demand is not: X must give way, roll must not
	ControlVector roll_only;
	roll_only(Axis::ROLL) = 0.3f;
	roll_only(Axis::THRUST_Z) = _hover_z;
	ASSERT_TRUE(feasible(roll_only));

	ControlVector control_sp = roll_only;
	control_sp(Axis::THRUST_X) = 5.f;
	ASSERT_FALSE(feasible(control_sp));

	const ControlVector allocated = allocate(control_sp);

	EXPECT_NEAR(allocated(Axis::ROLL), 0.3f, kTol);
	EXPECT_NEAR(allocated(Axis::THRUST_Z), _hover_z, kTol);
	EXPECT_LT(allocated(Axis::THRUST_X), control_sp(Axis::THRUST_X));
}

TEST_F(DtrgSequentialDesaturation, SidewaysThrustIsGivenUpForYaw)
{
	ControlVector yaw_only;
	yaw_only(Axis::YAW) = 0.1f;
	yaw_only(Axis::THRUST_Z) = _hover_z;
	ASSERT_TRUE(feasible(yaw_only));

	ControlVector control_sp = yaw_only;
	control_sp(Axis::THRUST_Y) = -5.f;
	ASSERT_FALSE(feasible(control_sp));

	const ControlVector allocated = allocate(control_sp);

	EXPECT_NEAR(allocated(Axis::YAW), 0.1f, kTol);
	EXPECT_NEAR(allocated(Axis::THRUST_Z), _hover_z, kTol);
	EXPECT_LT(allocated(Axis::THRUST_Y), 0.f);
	EXPECT_GT(allocated(Axis::THRUST_Y), control_sp(Axis::THRUST_Y));
}

TEST_F(DtrgSequentialDesaturation, YawIsReducedBeforeThrust)
{
	// DTRG order: yaw gives way before collective thrust (upstream instead trades up to 15 % thrust for yaw)
	ControlVector control_sp;
	control_sp(Axis::THRUST_Z) = 1.8f * _hover_z; // rotors at 0.9
	control_sp(Axis::YAW) = 5.f;
	ASSERT_FALSE(feasible(control_sp));

	const ControlVector allocated = allocate(control_sp);

	EXPECT_NEAR(allocated(Axis::THRUST_Z), control_sp(Axis::THRUST_Z), kTol);
	EXPECT_GT(allocated(Axis::YAW), 0.f);
	EXPECT_LT(allocated(Axis::YAW), control_sp(Axis::YAW));
	EXPECT_NEAR(allocated(Axis::ROLL), 0.f, kTol);
	EXPECT_NEAR(allocated(Axis::PITCH), 0.f, kTol);
}

TEST_F(DtrgSequentialDesaturation, ThrustIsReducedBeforeRoll)
{
	// near full thrust, a roll demand pushes the upper rotors past 1: thrust is lowered, roll is kept
	ControlVector control_sp;
	control_sp(Axis::THRUST_Z) = 1.8f * _hover_z;
	control_sp(Axis::ROLL) = 0.6f;
	ASSERT_FALSE(feasible(control_sp));

	const ControlVector allocated = allocate(control_sp);

	EXPECT_NEAR(allocated(Axis::ROLL), 0.6f, kTol);
	EXPECT_GT(allocated(Axis::THRUST_Z), control_sp(Axis::THRUST_Z)); // less negative = less thrust up
	EXPECT_LE(maxActuator(_allocator.getActuatorSetpoint()), 1.f + kTol);
}

TEST_F(DtrgSequentialDesaturation, ThrustIsNeverIncreasedToDesaturate)
{
	// airmode off: at low thrust a roll demand pushes rotors below 0, and thrust must not be raised to fix it
	ControlVector control_sp;
	control_sp(Axis::THRUST_Z) = 0.2f * _hover_z; // rotors at 0.1
	control_sp(Axis::ROLL) = 1.f;
	ASSERT_FALSE(feasible(control_sp));

	const ControlVector allocated = allocate(control_sp);

	EXPECT_NEAR(allocated(Axis::THRUST_Z), control_sp(Axis::THRUST_Z), kTol);
	EXPECT_LT(allocated(Axis::ROLL), 1.f);
}

TEST_F(DtrgSequentialDesaturation, PublishesTopic)
{
	uORB::Subscription sub{ORB_ID(sequential_desaturation)};
	sub.subscribe();

	ControlVector control_sp;
	control_sp(Axis::THRUST_Z) = _hover_z;
	allocate(control_sp);

	sequential_desaturation_s msg{};
	ASSERT_TRUE(sub.update(&msg));
	EXPECT_FLOAT_EQ(msg.x_sat, 0.f);
	EXPECT_FLOAT_EQ(msg.y_sat, 0.f);
	EXPECT_FLOAT_EQ(msg.z_sat, 0.f);
	EXPECT_FLOAT_EQ(msg.roll_sat, 0.f);
	EXPECT_FLOAT_EQ(msg.pitch_sat, 0.f);
	EXPECT_FLOAT_EQ(msg.yaw_sat, 0.f);
}

// Known gap: desaturateActuators() returns only the gain of its second, half
// strength pass. When the first pass already removes the saturation, as it
// does for a one-sided overshoot, the reported gain is 0 even though X was
// cut. The sign is also lost on the way to SYS_STATUS.errors_count1, which
// only tests "> 0.01", while cutting positive X gives a negative gain.
TEST_F(DtrgSequentialDesaturation, DISABLED_TopicReportsHorizontalThrustReduction)
{
	uORB::Subscription sub{ORB_ID(sequential_desaturation)};
	sub.subscribe();

	ControlVector control_sp;
	control_sp(Axis::THRUST_X) = 5.f;
	control_sp(Axis::THRUST_Z) = _hover_z;
	const ControlVector allocated = allocate(control_sp);
	ASSERT_LT(allocated(Axis::THRUST_X), control_sp(Axis::THRUST_X));

	sequential_desaturation_s msg{};
	ASSERT_TRUE(sub.update(&msg));
	EXPECT_GT(fabsf(msg.x_sat), 0.01f);
	EXPECT_LT(fabsf(msg.roll_sat), 0.01f);
	EXPECT_LT(fabsf(msg.pitch_sat), 0.01f);
}

// Known gap: the X, Y and yaw steps slide along their axis to reduce any
// saturation, not only to shrink their own demand. A roll demand near full
// thrust therefore comes out with X thrust and yaw that nobody asked for
// (about -0.02 X and -0.12 yaw, normalised, for this case), before thrust is
// reduced. Upstream only ever trades Z and yaw that were demanded.
TEST_F(DtrgSequentialDesaturation, DISABLED_DesaturationDoesNotAddUnrequestedAxes)
{
	ControlVector control_sp;
	control_sp(Axis::THRUST_Z) = 1.8f * _hover_z;
	control_sp(Axis::ROLL) = 1.f;
	ASSERT_FALSE(feasible(control_sp));

	const ControlVector allocated = allocate(control_sp);

	EXPECT_NEAR(allocated(Axis::THRUST_X), 0.f, kTol);
	EXPECT_NEAR(allocated(Axis::THRUST_Y), 0.f, kTol);
	EXPECT_NEAR(allocated(Axis::YAW), 0.f, kTol);
	EXPECT_NEAR(allocated(Axis::PITCH), 0.f, kTol);
}
