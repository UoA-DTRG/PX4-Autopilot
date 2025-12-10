/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
 * @file multisine_excitation_test.cpp
 *
 * Unit tests for the MultisineExcitation class
 */

#include <gtest/gtest.h>
#include "multisine_excitation.hpp"
#include <cmath>

using namespace multisine;

class MultisineExcitationTest : public ::testing::Test
{
protected:
	MultisineExcitation generator;
};

TEST_F(MultisineExcitationTest, ConfigureValid)
{
	// Test valid configuration matching MATLAB: 8 motors, 15s period, 0.1-1.0 Hz
	EXPECT_TRUE(generator.configure(8, 15.0f, 0.1f, 1.0f, 0.1f));
	EXPECT_TRUE(generator.isConfigured());
	EXPECT_EQ(generator.getNumMotors(), 8);
	EXPECT_FLOAT_EQ(generator.getPeriodPerMotor(), 15.0f);
	EXPECT_FLOAT_EQ(generator.getTotalDuration(), 120.0f); // 15 * 8
}

TEST_F(MultisineExcitationTest, ConfigureInvalidZeroMotors)
{
	EXPECT_FALSE(generator.configure(0, 15.0f, 0.1f, 1.0f, 0.1f));
	EXPECT_FALSE(generator.isConfigured());
}

TEST_F(MultisineExcitationTest, ConfigureInvalidTooManyMotors)
{
	EXPECT_FALSE(generator.configure(MAX_MOTORS + 1, 15.0f, 0.1f, 1.0f, 0.1f));
	EXPECT_FALSE(generator.isConfigured());
}

TEST_F(MultisineExcitationTest, ConfigureInvalidPeriod)
{
	EXPECT_FALSE(generator.configure(8, 0.0f, 0.1f, 1.0f, 0.1f));
	EXPECT_FALSE(generator.configure(8, -1.0f, 0.1f, 1.0f, 0.1f));
}

TEST_F(MultisineExcitationTest, ConfigureInvalidFrequency)
{
	// f_min <= 0
	EXPECT_FALSE(generator.configure(8, 15.0f, 0.0f, 1.0f, 0.1f));
	// f_max <= f_min
	EXPECT_FALSE(generator.configure(8, 15.0f, 1.0f, 0.5f, 0.1f));
}

TEST_F(MultisineExcitationTest, ConfigureInvalidAmplitude)
{
	EXPECT_FALSE(generator.configure(8, 15.0f, 0.1f, 1.0f, 0.0f));
	EXPECT_FALSE(generator.configure(8, 15.0f, 0.1f, 1.0f, 1.5f));
}

TEST_F(MultisineExcitationTest, ConfigureNotEnoughHarmonics)
{
	// With T=15s, f0=1/15=0.0667Hz
	// If f_min=0.1 and f_max=0.15, only ~1 harmonic available
	// Can't assign 8 motors with only 1 harmonic
	EXPECT_FALSE(generator.configure(8, 15.0f, 0.1f, 0.15f, 0.1f));
}

TEST_F(MultisineExcitationTest, StartStop)
{
	ASSERT_TRUE(generator.configure(8, 15.0f, 0.1f, 1.0f, 0.1f));

	EXPECT_FALSE(generator.isActive());
	generator.start();
	EXPECT_TRUE(generator.isActive());
	EXPECT_FLOAT_EQ(generator.getElapsedTime(), 0.0f);

	generator.stop();
	EXPECT_FALSE(generator.isActive());
}

TEST_F(MultisineExcitationTest, UpdateSequential)
{
	ASSERT_TRUE(generator.configure(8, 15.0f, 0.1f, 1.0f, 0.1f));
	generator.setSequentialMode(true);
	generator.start();

	float excitation[MAX_MOTORS];

	// Run for a few iterations
	const float dt = 0.004f; // 250Hz

	for (int i = 0; i < 100; i++) {
		bool active = generator.update(dt, excitation);
		EXPECT_TRUE(active);

		// In sequential mode, only current motor should have non-zero excitation
		uint8_t current_motor = generator.getCurrentMotor();

		for (uint8_t m = 0; m < 8; m++) {
			if (m == current_motor) {
				// Current motor should have some excitation (might be small though)
				// Don't check exact value, just that it's set
			} else {
				EXPECT_FLOAT_EQ(excitation[m], 0.0f);
			}
		}
	}
}

TEST_F(MultisineExcitationTest, SignalBounded)
{
	ASSERT_TRUE(generator.configure(8, 15.0f, 0.1f, 1.0f, 0.1f));
	generator.start();

	float excitation[MAX_MOTORS];
	const float dt = 0.004f;
	const float amplitude = 0.1f;

	// Run through entire excitation sequence
	while (generator.update(dt, excitation)) {
		for (uint8_t m = 0; m < 8; m++) {
			// Signal should be bounded by [-amplitude, amplitude]
			EXPECT_LE(excitation[m], amplitude + 0.001f);
			EXPECT_GE(excitation[m], -amplitude - 0.001f);
		}
	}
}

TEST_F(MultisineExcitationTest, CompletionSequential)
{
	ASSERT_TRUE(generator.configure(4, 1.0f, 0.5f, 5.0f, 0.1f)); // 4 motors, 1s each = 4s total
	generator.setSequentialMode(true);
	generator.start();

	float excitation[MAX_MOTORS];
	const float dt = 0.01f; // 100Hz
	int iterations = 0;
	const int max_iterations = 1000; // Safety limit

	while (generator.update(dt, excitation) && iterations < max_iterations) {
		iterations++;
	}

	// Should complete in approximately 4s / 0.01s = 400 iterations
	EXPECT_GT(iterations, 350);
	EXPECT_LT(iterations, 450);
	EXPECT_FALSE(generator.isActive());
}

TEST_F(MultisineExcitationTest, SimultaneousMode)
{
	// Use shorter period for faster testing
	ASSERT_TRUE(generator.configure(8, 2.0f, 0.5f, 10.0f, 0.1f));
	ASSERT_TRUE(generator.isConfigured()) << "Generator should be configured";

	// Check harmonics were assigned
	int total_harmonics = 0;

	for (uint8_t m = 0; m < 8; m++) {
		int nh = generator.getNumHarmonics(m);
		total_harmonics += nh;
	}

	EXPECT_GT(total_harmonics, 0) << "Should have some harmonics assigned";

	// Test simultaneous mode directly
	generator.setSequentialMode(false);
	generator.start();
	ASSERT_TRUE(generator.isActive()) << "Generator should be active after start";

	float excitation[MAX_MOTORS];
	const float dt = 0.004f;

	// Run multiple updates and find max absolute excitation
	float max_excitation = 0.0f;
	int num_updates = 250; // 1 second at 250Hz

	for (int i = 0; i < num_updates; i++) {
		bool still_active = generator.update(dt, excitation);
		EXPECT_TRUE(still_active) << "Should still be active at iteration " << i;

		for (uint8_t m = 0; m < 8; m++) {
			float abs_val = fabsf(excitation[m]);

			if (abs_val > max_excitation) {
				max_excitation = abs_val;
			}
		}
	}

	// Over 1 second of updates, should have seen non-zero excitation
	EXPECT_GT(max_excitation, 0.0f) << "Simultaneous mode should produce non-zero excitation over time";

	// Also test sequential mode produces non-zero values
	generator.stop();
	generator.setSequentialMode(true);
	generator.start();
	ASSERT_TRUE(generator.isActive());

	float max_seq_excitation = 0.0f;

	for (int i = 0; i < num_updates; i++) {
		bool still_active = generator.update(dt, excitation);
		EXPECT_TRUE(still_active);

		for (uint8_t m = 0; m < 8; m++) {
			float abs_val = fabsf(excitation[m]);

			if (abs_val > max_seq_excitation) {
				max_seq_excitation = abs_val;
			}
		}
	}

	EXPECT_GT(max_seq_excitation, 0.0f) << "Sequential mode should produce non-zero excitation over time";
}

TEST_F(MultisineExcitationTest, MotorProgression)
{
	ASSERT_TRUE(generator.configure(4, 1.0f, 0.5f, 5.0f, 0.1f));
	generator.setSequentialMode(true);
	generator.start();

	float excitation[MAX_MOTORS];
	const float dt = 0.01f;

	// Check motor progression
	EXPECT_EQ(generator.getCurrentMotor(), 0);

	// Advance to motor 1
	for (int i = 0; i < 110; i++) { // Just past 1 second
		generator.update(dt, excitation);
	}

	EXPECT_EQ(generator.getCurrentMotor(), 1);

	// Advance to motor 2
	for (int i = 0; i < 100; i++) {
		generator.update(dt, excitation);
	}

	EXPECT_EQ(generator.getCurrentMotor(), 2);
}
