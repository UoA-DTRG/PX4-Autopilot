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
 * @file DtrgBenchProfileTest.cpp
 *
 * Excitation profile of the bench test mode: step timing, ramp clamp and
 * freeze, throttle spin-up, axis mapping and the motor saturation test.
 */

#include <gtest/gtest.h>

#include <math.h>

#include "bench_test_profile.h"

using namespace bench_test;

namespace
{

actuator_motors_s makeMotors(int num_connected, float value)
{
	actuator_motors_s motors{};

	for (int i = 0; i < actuator_motors_s::NUM_CONTROLS; i++) {
		motors.control[i] = (i < num_connected) ? value : NAN;
	}

	return motors;
}

} // namespace

// Step ------------------------------------------------------------------------

TEST(DtrgBenchProfile, StepIsZeroBeforeDelay)
{
	EXPECT_FLOAT_EQ(stepOutput(1.f, 0.f, 2.f, 1.f, 0.1f), 0.f);
	EXPECT_FLOAT_EQ(stepOutput(1.f, 1.999f, 2.f, 1.f, 0.1f), 0.f);
}

TEST(DtrgBenchProfile, StepIsActiveForDuration)
{
	EXPECT_FLOAT_EQ(stepOutput(1.f, 2.f, 2.f, 1.f, 0.1f), 0.1f);
	EXPECT_FLOAT_EQ(stepOutput(1.f, 2.999f, 2.f, 1.f, 0.1f), 0.1f);
	EXPECT_FLOAT_EQ(stepOutput(1.f, 3.f, 2.f, 1.f, 0.1f), 0.f);
	EXPECT_FLOAT_EQ(stepOutput(1.f, 100.f, 2.f, 1.f, 0.1f), 0.f);
}

TEST(DtrgBenchProfile, StepFollowsSign)
{
	EXPECT_FLOAT_EQ(stepOutput(-1.f, 2.5f, 2.f, 1.f, 0.1f), -0.1f);
	EXPECT_FLOAT_EQ(stepOutput(0.f, 2.5f, 2.f, 1.f, 0.1f), 0.f);
}

TEST(DtrgBenchProfile, StepWithoutDelayStartsImmediately)
{
	EXPECT_FLOAT_EQ(stepOutput(1.f, 0.f, 0.f, 1.f, 0.2f), 0.2f);
}

// Ramp ------------------------------------------------------------------------

TEST(DtrgBenchProfile, RampIncreasesLinearly)
{
	RampState ramp{};
	EXPECT_FLOAT_EQ(rampOutput(ramp, 1.f, 0.f, 0.1f, 0.5f, false), 0.f);
	EXPECT_FLOAT_EQ(rampOutput(ramp, 1.f, 1.f, 0.1f, 0.5f, false), 0.1f);
	EXPECT_FLOAT_EQ(rampOutput(ramp, 1.f, 2.f, 0.1f, 0.5f, false), 0.2f);
	EXPECT_FALSE(ramp.frozen);
}

TEST(DtrgBenchProfile, RampNegativeSign)
{
	RampState ramp{};
	EXPECT_FLOAT_EQ(rampOutput(ramp, -1.f, 2.f, 0.1f, 0.5f, false), -0.2f);
}

TEST(DtrgBenchProfile, RampStopsAtMaxValAndFreezes)
{
	RampState ramp{};
	EXPECT_FLOAT_EQ(rampOutput(ramp, 1.f, 10.f, 0.1f, 0.5f, false), 0.5f);
	EXPECT_TRUE(ramp.frozen);

	// holds the clamp even though the time keeps running
	EXPECT_FLOAT_EQ(rampOutput(ramp, 1.f, 20.f, 0.1f, 0.5f, false), 0.5f);
}

TEST(DtrgBenchProfile, RampNegativeStopsAtMinusMaxVal)
{
	RampState ramp{};
	EXPECT_FLOAT_EQ(rampOutput(ramp, -1.f, 10.f, 0.1f, 0.5f, false), -0.5f);
	EXPECT_TRUE(ramp.frozen);
}

TEST(DtrgBenchProfile, RampFreezesOnMotorSaturation)
{
	RampState ramp{};
	EXPECT_FLOAT_EQ(rampOutput(ramp, 1.f, 1.f, 0.1f, 0.5f, false), 0.1f);
	EXPECT_FLOAT_EQ(rampOutput(ramp, 1.f, 2.f, 0.1f, 0.5f, true), 0.2f);
	EXPECT_TRUE(ramp.frozen);

	// frozen: later calls hold the value at saturation, whether or not the motor is still saturated
	EXPECT_FLOAT_EQ(rampOutput(ramp, 1.f, 3.f, 0.1f, 0.5f, false), 0.2f);
	EXPECT_FLOAT_EQ(rampOutput(ramp, 1.f, 4.f, 0.1f, 0.5f, true), 0.2f);
}

TEST(DtrgBenchProfile, RampResetClearsFreeze)
{
	RampState ramp{};
	rampOutput(ramp, 1.f, 10.f, 0.1f, 0.5f, false);
	ASSERT_TRUE(ramp.frozen);

	ramp.reset();
	EXPECT_FALSE(ramp.frozen);
	EXPECT_FLOAT_EQ(ramp.value, 0.f);
	EXPECT_FLOAT_EQ(rampOutput(ramp, 1.f, 1.f, 0.1f, 0.5f, false), 0.1f);
}

// Spin-up ---------------------------------------------------------------------

TEST(DtrgBenchProfile, SpinupRampsToOne)
{
	EXPECT_FLOAT_EQ(spinupFactor(0.f, 2.f), 0.f);
	EXPECT_FLOAT_EQ(spinupFactor(1.f, 2.f), 0.5f);
	EXPECT_FLOAT_EQ(spinupFactor(2.f, 2.f), 1.f);
	EXPECT_FLOAT_EQ(spinupFactor(5.f, 2.f), 1.f);
}

TEST(DtrgBenchProfile, SpinupDisabled)
{
	EXPECT_FLOAT_EQ(spinupFactor(0.f, 0.f), 1.f);
	EXPECT_FLOAT_EQ(spinupFactor(0.f, -1.f), 1.f);
}

// Axis mapping ----------------------------------------------------------------

TEST(DtrgBenchProfile, HoverBaselineIsUpwardThrustOnly)
{
	const Setpoints sp = composeSetpoints(Axis::Roll, 0.4f, 0.5f, 0.f);
	EXPECT_FLOAT_EQ(sp.thrust[0], 0.f);
	EXPECT_FLOAT_EQ(sp.thrust[1], 0.f);
	EXPECT_FLOAT_EQ(sp.thrust[2], -0.2f); // NED: -Z is up
	EXPECT_FLOAT_EQ(sp.torque[0], 0.f);
	EXPECT_FLOAT_EQ(sp.torque[1], 0.f);
	EXPECT_FLOAT_EQ(sp.torque[2], 0.f);
}

TEST(DtrgBenchProfile, EachAxisDrivesOnlyItsSetpoint)
{
	struct Case {
		Axis axis;
		int thrust_index; // -1 when the axis is a torque
		int torque_index; // -1 when the axis is a thrust
	};

	const Case cases[] = {
		{Axis::ThrustX, 0, -1},
		{Axis::ThrustY, 1, -1},
		{Axis::Roll, -1, 0},
		{Axis::Pitch, -1, 1},
		{Axis::Yaw, -1, 2},
	};

	for (const Case &c : cases) {
		const Setpoints sp = composeSetpoints(c.axis, 0.3f, 1.f, 0.1f);

		for (int i = 0; i < 3; i++) {
			const float expected_thrust = (i == c.thrust_index) ? 0.1f : ((i == 2) ? -0.3f : 0.f);
			const float expected_torque = (i == c.torque_index) ? 0.1f : 0.f;
			EXPECT_FLOAT_EQ(sp.thrust[i], expected_thrust) << "axis " << static_cast<int>(c.axis) << " thrust " << i;
			EXPECT_FLOAT_EQ(sp.torque[i], expected_torque) << "axis " << static_cast<int>(c.axis) << " torque " << i;
		}
	}
}

TEST(DtrgBenchProfile, ThrustZExcitationAddsUpwardThrust)
{
	// positive excitation on Z means more lift, i.e. more negative NED thrust
	const Setpoints sp = composeSetpoints(Axis::ThrustZ, 0.2f, 1.f, 0.1f);
	EXPECT_FLOAT_EQ(sp.thrust[2], -0.3f);
}

TEST(DtrgBenchProfile, ExcitationIsNotScaledBySpinup)
{
	// only the hover baseline is soft-started, the excitation is applied as is
	const Setpoints sp = composeSetpoints(Axis::Roll, 0.4f, 0.f, 0.1f);
	EXPECT_FLOAT_EQ(sp.thrust[2], 0.f);
	EXPECT_FLOAT_EQ(sp.torque[0], 0.1f);
}

// Motor saturation ------------------------------------------------------------

TEST(DtrgBenchProfile, CountFiniteMotors)
{
	EXPECT_EQ(countFiniteMotors(makeMotors(0, 0.5f)), 0);
	EXPECT_EQ(countFiniteMotors(makeMotors(4, 0.5f)), 4);
	EXPECT_EQ(countFiniteMotors(makeMotors(8, 0.5f)), 8);
}

TEST(DtrgBenchProfile, MidRangeIsNotSaturated)
{
	EXPECT_FALSE(motorsSaturated(makeMotors(4, 0.5f), 4, 0.05f));
}

TEST(DtrgBenchProfile, UpperMarginSaturates)
{
	actuator_motors_s motors = makeMotors(4, 0.5f);
	motors.control[2] = 0.95f;
	EXPECT_TRUE(motorsSaturated(motors, 4, 0.05f));

	motors.control[2] = 0.94f;
	EXPECT_FALSE(motorsSaturated(motors, 4, 0.05f));
}

TEST(DtrgBenchProfile, LowerMarginSaturates)
{
	actuator_motors_s motors = makeMotors(4, 0.5f);
	motors.control[1] = 0.05f;
	EXPECT_TRUE(motorsSaturated(motors, 4, 0.05f));

	motors.control[1] = 0.06f;
	EXPECT_FALSE(motorsSaturated(motors, 4, 0.05f));
}

TEST(DtrgBenchProfile, ReversibleMotorUsesSymmetricRange)
{
	actuator_motors_s motors = makeMotors(4, 0.5f);
	motors.reversible_flags = 1u << 0;

	// 0 is mid-range for a reversible motor, but the lower stop for a standard one
	motors.control[0] = 0.f;
	EXPECT_FALSE(motorsSaturated(motors, 4, 0.05f));

	motors.control[0] = -0.95f;
	EXPECT_TRUE(motorsSaturated(motors, 4, 0.05f));
}

TEST(DtrgBenchProfile, UnconnectedMotorsAreIgnored)
{
	// motor 5 is saturated, but only 4 motors are connected
	actuator_motors_s motors = makeMotors(8, 0.5f);
	motors.control[4] = 1.f;
	EXPECT_FALSE(motorsSaturated(motors, 4, 0.05f));
	EXPECT_TRUE(motorsSaturated(motors, 5, 0.05f));
}

TEST(DtrgBenchProfile, NonFiniteOutputsAreIgnored)
{
	actuator_motors_s motors = makeMotors(4, 0.5f);
	motors.control[3] = NAN;
	EXPECT_FALSE(motorsSaturated(motors, 4, 0.05f));
}

TEST(DtrgBenchProfile, MarginIsClamped)
{
	// a margin above 0.5 behaves exactly like 0.5
	for (float value : {0.f, 0.3f, 0.5f, 0.7f, 1.f}) {
		const actuator_motors_s motors = makeMotors(4, value);
		EXPECT_EQ(motorsSaturated(motors, 4, 2.f), motorsSaturated(motors, 4, 0.5f)) << "output " << value;
	}

	// a negative margin is clamped to 0: only the hard stops count
	actuator_motors_s motors = makeMotors(4, 0.5f);
	motors.control[0] = 0.99f;
	EXPECT_FALSE(motorsSaturated(motors, 4, -1.f));
	motors.control[0] = 1.f;
	EXPECT_TRUE(motorsSaturated(motors, 4, -1.f));
}

TEST(DtrgBenchProfile, MotorCountAboveArrayIsBounded)
{
	EXPECT_FALSE(motorsSaturated(makeMotors(actuator_motors_s::NUM_CONTROLS, 0.5f), 1000, 0.05f));
}
