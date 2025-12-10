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
 * @file multisine_excitation.hpp
 *
 * Orthogonal Multisine Excitation Generator for System Identification
 *
 * Based on Morelli's method: "Practical Aspects of Multiple-Input Design
 * for Aircraft System Identification Flight Tests" (NASA, 2012)
 *
 * Generates orthogonal multisine signals for each motor channel using:
 * 1. Harmonic combing - assigns non-overlapping frequency sets to each input
 * 2. Schroeder phase optimization - minimizes peak factor / crest factor
 *
 * @author Jaap Peetsma <jaap.peetsma@gmail.com>
 */

#pragma once

#include <mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>
#include <cmath>
#include <cstdint>

namespace multisine
{

/**
 * Maximum number of motors supported
 */
static constexpr uint8_t MAX_MOTORS = 16;

/**
 * Maximum number of harmonics per motor
 * With T=15s, f_min=0.1Hz, f_max=1Hz, f0=1/15=0.0667Hz
 * h_min = ceil(0.1/0.0667) = 2, h_max = floor(1.0/0.0667) = 15
 * Total harmonics = 14, per motor (8) = ~2 harmonics each
 * Allow for larger frequency ranges
 */
static constexpr uint8_t MAX_HARMONICS_PER_MOTOR = 32;

/**
 * @class MultisineExcitation
 *
 * Generates orthogonal multisine excitation signals for system identification.
 * Each motor receives a unique set of frequencies (harmonic combing) to ensure
 * orthogonality. Schroeder phases are used to minimize the crest factor.
 */
class MultisineExcitation
{
public:
	MultisineExcitation() = default;
	~MultisineExcitation() = default;

	/**
	 * Configure the multisine generator
	 *
	 * @param num_motors Number of motors to generate signals for (1-MAX_MOTORS)
	 * @param period_s Period of excitation per motor in seconds
	 * @param f_min_hz Minimum frequency of interest in Hz
	 * @param f_max_hz Maximum frequency of interest in Hz
	 * @param amplitude Amplitude scaling factor (0-1)
	 * @return true if configuration is valid
	 */
	bool configure(uint8_t num_motors, float period_s, float f_min_hz, float f_max_hz, float amplitude);

	/**
	 * Start/reset the excitation sequence
	 * Call this when beginning a new excitation
	 */
	void start();

	/**
	 * Stop the excitation
	 */
	void stop();

	/**
	 * Update and compute excitation values for the current time
	 *
	 * @param dt Time since last update in seconds
	 * @param excitation Output array of excitation values for each motor
	 * @return true if excitation is still active
	 */
	bool update(float dt, float excitation[MAX_MOTORS]);

	/**
	 * Get the total duration of the excitation sequence
	 * @return Total duration in seconds
	 */
	float getTotalDuration() const { return _total_duration_s; }

	/**
	 * Get the elapsed time since start
	 * @return Elapsed time in seconds
	 */
	float getElapsedTime() const { return _elapsed_time_s; }

	/**
	 * Check if the excitation is currently active
	 * @return true if active
	 */
	bool isActive() const { return _active; }

	/**
	 * Get the current motor being excited
	 * @return Motor index (0-indexed), or 0xFF if sequential mode is off
	 */
	uint8_t getCurrentMotor() const { return _current_motor; }

	/**
	 * Get the number of configured motors
	 */
	uint8_t getNumMotors() const { return _num_motors; }

	/**
	 * Get configured period per motor
	 */
	float getPeriodPerMotor() const { return _period_s; }

	/**
	 * Check if configuration is valid
	 */
	bool isConfigured() const { return _configured; }

	/**
	 * Enable/disable sequential motor excitation mode
	 * In sequential mode, only one motor is excited at a time
	 * In simultaneous mode, all motors receive their orthogonal signals
	 * @param sequential true for sequential, false for simultaneous
	 */
	void setSequentialMode(bool sequential) { _sequential_mode = sequential; }

	/**
	 * Get sequential mode setting
	 */
	bool getSequentialMode() const { return _sequential_mode; }

	/**
	 * Get number of harmonics for a motor (for testing/debugging)
	 */
	uint8_t getNumHarmonics(uint8_t motor) const
	{
		if (motor < MAX_MOTORS) {
			return _num_harmonics[motor];
		}

		return 0;
	}

	/**
	 * Get harmonic value for a motor (for testing/debugging)
	 */
	uint16_t getHarmonic(uint8_t motor, uint8_t idx) const
	{
		if (motor < MAX_MOTORS && idx < MAX_HARMONICS_PER_MOTOR) {
			return _harmonics[motor][idx];
		}

		return 0;
	}

	/**
	 * Get normalization value for a motor (for testing/debugging)
	 */
	float getNormalization(uint8_t motor) const
	{
		if (motor < MAX_MOTORS) {
			return _normalization[motor];
		}

		return 1.0f;
	}

	/**
	 * Get fundamental frequency (for testing/debugging)
	 */
	float getF0() const { return _f0_hz; }

private:
	/**
	 * Compute the harmonic assignments for each motor using the combing method
	 */
	void computeHarmonics();

	/**
	 * Compute Schroeder phases for each motor's harmonics
	 * phi_k = pi * k^2 / M  where M is the number of harmonics
	 */
	void computeSchroederPhases();

	/**
	 * Compute the normalization factor for each motor's signal
	 * to ensure output is in range [-1, 1]
	 */
	void computeNormalization();

	/**
	 * Generate the multisine signal for a single motor at time t
	 *
	 * @param motor_idx Motor index
	 * @param t Time in seconds (within the period)
	 * @return Signal value (normalized to [-1, 1])
	 */
	float generateSignal(uint8_t motor_idx, float t) const;

	// Configuration
	uint8_t _num_motors{0};
	float _period_s{15.0f};
	float _f_min_hz{0.1f};
	float _f_max_hz{1.0f};
	float _amplitude{0.1f};
	float _f0_hz{0.0f};  // Fundamental frequency = 1/period
	bool _configured{false};
	bool _sequential_mode{true};  // Default: excite one motor at a time

	// State
	bool _active{false};
	float _elapsed_time_s{0.0f};
	float _total_duration_s{0.0f};
	uint8_t _current_motor{0};

	// Harmonic data per motor
	// harmonics[motor][harmonic_idx] = harmonic number
	uint16_t _harmonics[MAX_MOTORS][MAX_HARMONICS_PER_MOTOR]{};
	uint8_t _num_harmonics[MAX_MOTORS]{};

	// Schroeder phases per motor
	// phases[motor][harmonic_idx] = phase in radians
	float _phases[MAX_MOTORS][MAX_HARMONICS_PER_MOTOR]{};

	// Normalization factors per motor (inverse of max amplitude)
	float _normalization[MAX_MOTORS]{};
};

} // namespace multisine
