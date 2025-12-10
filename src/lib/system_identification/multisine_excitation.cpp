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
 * @file multisine_excitation.cpp
 *
 * Orthogonal Multisine Excitation Generator implementation
 */

#include "multisine_excitation.hpp"
#include <cstring>

namespace multisine
{

bool MultisineExcitation::configure(uint8_t num_motors, float period_s, float f_min_hz, float f_max_hz, float amplitude)
{
	// Validate inputs
	if (num_motors == 0 || num_motors > MAX_MOTORS) {
		return false;
	}

	if (period_s <= 0.0f) {
		return false;
	}

	if (f_min_hz <= 0.0f || f_max_hz <= f_min_hz) {
		return false;
	}

	if (amplitude <= 0.0f || amplitude > 1.0f) {
		return false;
	}

	_num_motors = num_motors;
	_period_s = period_s;
	_f_min_hz = f_min_hz;
	_f_max_hz = f_max_hz;
	_amplitude = amplitude;

	// Fundamental frequency (frequency resolution)
	_f0_hz = 1.0f / _period_s;

	// Check if we have enough harmonics for the number of motors
	// h_min = ceil(f_min / f0), h_max = floor(f_max / f0)
	int h_min = static_cast<int>(ceilf(_f_min_hz / _f0_hz));
	int h_max = static_cast<int>(floorf(_f_max_hz / _f0_hz));
	int total_harmonics = h_max - h_min + 1;

	if (total_harmonics < static_cast<int>(_num_motors)) {
		// Not enough frequency bins for the number of motors
		// Either increase period (decrease f0) or increase frequency range
		return false;
	}

	// Compute total duration for sequential mode
	_total_duration_s = _period_s * static_cast<float>(_num_motors);

	// Compute harmonic assignments, phases, and normalization
	computeHarmonics();
	computeSchroederPhases();
	computeNormalization();

	_configured = true;
	return true;
}

void MultisineExcitation::computeHarmonics()
{
	// Clear existing harmonics
	memset(_harmonics, 0, sizeof(_harmonics));
	memset(_num_harmonics, 0, sizeof(_num_harmonics));

	// Compute harmonic range
	int h_min = static_cast<int>(ceilf(_f_min_hz / _f0_hz));
	int h_max = static_cast<int>(floorf(_f_max_hz / _f0_hz));

	// Use the combing method: assign every N-th harmonic to the same motor
	// This ensures orthogonality as each motor gets a unique set of frequencies
	int harmonic_idx = 0;

	for (int h = h_min; h <= h_max; h++) {
		uint8_t motor_idx = static_cast<uint8_t>(harmonic_idx % _num_motors);

		if (_num_harmonics[motor_idx] < MAX_HARMONICS_PER_MOTOR) {
			_harmonics[motor_idx][_num_harmonics[motor_idx]] = static_cast<uint16_t>(h);
			_num_harmonics[motor_idx]++;
		}

		harmonic_idx++;
	}
}

void MultisineExcitation::computeSchroederPhases()
{
	// Clear existing phases
	memset(_phases, 0, sizeof(_phases));

	// Schroeder's formula: phi_k = pi * k^2 / M
	// where k is the harmonic index (1-based) and M is the total number of harmonics
	for (uint8_t motor = 0; motor < _num_motors; motor++) {
		uint8_t M = _num_harmonics[motor];

		if (M == 0) {
			continue;
		}

		for (uint8_t k = 0; k < M; k++) {
			// k is 0-indexed here, formula uses 1-indexed
			float k1 = static_cast<float>(k + 1);
			_phases[motor][k] = M_PI_F * k1 * k1 / static_cast<float>(M);
		}
	}
}

void MultisineExcitation::computeNormalization()
{
	// Compute the maximum absolute value of each motor's signal
	// by evaluating at multiple time points and finding the peak
	// This ensures the normalized signal stays within [-1, 1]

	// First, initialize all normalization values to 1.0
	for (uint8_t i = 0; i < MAX_MOTORS; i++) {
		_normalization[i] = 1.0f;
	}

	constexpr int NUM_SAMPLES = 1000;
	const float dt = _period_s / static_cast<float>(NUM_SAMPLES);

	for (uint8_t motor = 0; motor < _num_motors; motor++) {
		if (_num_harmonics[motor] == 0) {
			continue; // Already set to 1.0
		}

		float max_val = 0.0f;

		for (int i = 0; i < NUM_SAMPLES; i++) {
			float t = static_cast<float>(i) * dt;

			// Compute raw signal (sum of sinusoids)
			float signal = 0.0f;

			for (uint8_t h_idx = 0; h_idx < _num_harmonics[motor]; h_idx++) {
				float freq = static_cast<float>(_harmonics[motor][h_idx]) * _f0_hz;
				float phase = _phases[motor][h_idx];
				signal += sinf(2.0f * M_PI_F * freq * t + phase);
			}

			float abs_val = fabsf(signal);

			if (abs_val > max_val) {
				max_val = abs_val;
			}
		}

		// Store inverse of max for efficient multiplication during runtime
		// We want to scale the signal to [-1, 1], so divide by max_val
		if (max_val > 1e-6f) {
			_normalization[motor] = 1.0f / max_val;
		}
		// else keep the default 1.0
	}
}

void MultisineExcitation::start()
{
	if (!_configured) {
		return;
	}

	_active = true;
	_elapsed_time_s = 0.0f;
	_current_motor = 0;
}

void MultisineExcitation::stop()
{
	_active = false;
	_elapsed_time_s = 0.0f;
	_current_motor = 0;
}

bool MultisineExcitation::update(float dt, float excitation[MAX_MOTORS])
{
	// Initialize all excitation values to 0
	for (uint8_t i = 0; i < MAX_MOTORS; i++) {
		excitation[i] = 0.0f;
	}

	if (!_active || !_configured) {
		return false;
	}

	// Update elapsed time
	_elapsed_time_s += dt;

	// Determine the duration based on mode
	float total_duration = _sequential_mode ? _total_duration_s : _period_s;

	// Check if we're done
	if (_elapsed_time_s >= total_duration) {
		_active = false;
		return false;
	}

	if (_sequential_mode) {
		// Sequential mode: one motor at a time
		_current_motor = static_cast<uint8_t>(_elapsed_time_s / _period_s);

		if (_current_motor >= _num_motors) {
			_current_motor = _num_motors - 1;
		}

		// Time within the current motor's excitation period
		float t_motor = fmodf(_elapsed_time_s, _period_s);

		// Generate signal only for the current motor
		excitation[_current_motor] = generateSignal(_current_motor, t_motor) * _amplitude;

	} else {
		// Simultaneous mode: all motors excited at once with orthogonal signals
		float t = fmodf(_elapsed_time_s, _period_s);

		for (uint8_t motor = 0; motor < _num_motors; motor++) {
			float sig = generateSignal(motor, t);
			excitation[motor] = sig * _amplitude;
		}
	}

	return true;
}

float MultisineExcitation::generateSignal(uint8_t motor_idx, float t) const
{
	if (motor_idx >= _num_motors || _num_harmonics[motor_idx] == 0) {
		return 0.0f;
	}

	float signal = 0.0f;

	// Sum all sinusoids for this motor
	for (uint8_t h_idx = 0; h_idx < _num_harmonics[motor_idx]; h_idx++) {
		float freq = static_cast<float>(_harmonics[motor_idx][h_idx]) * _f0_hz;
		float phase = _phases[motor_idx][h_idx];
		float component = sinf(2.0f * M_PI_F * freq * t + phase);
		signal += component;
	}

	// Normalize to [-1, 1]
	float result = signal * _normalization[motor_idx];

	return result;
}

} // namespace multisine
