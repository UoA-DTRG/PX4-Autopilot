/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file ControlAllocationPseudoInverse.cpp
 *
 * Simple Control Allocation Algorithm
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "ControlAllocationPseudoInverse.hpp"

#include <px4_platform_common/defines.h>

void
ControlAllocationPseudoInverse::setEffectivenessMatrix(
	const matrix::Matrix<float, ControlAllocation::NUM_AXES, ControlAllocation::NUM_ACTUATORS> &effectiveness,
	const ActuatorVector &actuator_trim, const ActuatorVector &linearization_point, int num_actuators,
	bool update_normalization_scale)
{
	ControlAllocation::setEffectivenessMatrix(effectiveness, actuator_trim, linearization_point, num_actuators,
			update_normalization_scale);
	_mix_update_needed = true;
	_normalization_needs_update = update_normalization_scale;

	if (_metric_allocation && update_normalization_scale) {
		// adding #include <px4_platform_common/log.h> + PX4_WARN leads to failed linking on test
		_normalization_needs_update = false;
	}
}

void
ControlAllocationPseudoInverse::updatePseudoInverse()
{
	if (_mix_update_needed) {
		//csv ovveride
		if (_csv_mixer.get()) {
			loadCsvMixer();

		} else {
			_csv_mixer_result = CsvMixerResult{};
			matrix::geninv(_effectiveness, _mix);
		}

		if (!_metric_allocation) {
			if (_normalization_needs_update && !_had_actuator_failure) {
				updateControlAllocationMatrixScale();
				_normalization_needs_update = false;
			}

			normalizeControlAllocationMatrix();
		}

		_mix_update_needed = false;

	}
}

void
ControlAllocationPseudoInverse::loadCsvMixer()
{
	// Rows beyond the end of the file are 0, not left over from a previous mixer
	matrix::Matrix<float, NUM_ACTUATORS, NUM_AXES> csv_mix{};
	CsvMixerResult result{};

	if (readMixerFromCSV(_csv_mixer_path, csv_mix, &result)) {
		if (result.num_rows != _num_actuators) {
			result.status = CsvMixerStatus::ROW_COUNT_MISMATCH;

		} else if (csv_mix.abs().max() <= 0.f) {
			result.status = CsvMixerStatus::ALL_ZERO;
		}
	}

	_csv_mixer_result = result;

	if (result.status == CsvMixerStatus::LOADED) {
		_csv_mix_last_valid = csv_mix;
		_csv_mix_valid = true;
	}

	if (_csv_mix_valid) {
		_mix = _csv_mix_last_valid;

		//check for disabled normalization
		if (!_mixer_normalization.get()) {
			// PX4_INFO("mixer normalization disabled");
			_normalization_needs_update = false;
		}

	} else {
		// No valid file: use an empty mixer rather than a stale or half-read one, so no
		// actuator is driven by a mixer that was not intended. Commander refuses to arm.
		_mix.setZero();
	}
}

bool
ControlAllocationPseudoInverse::readMixerFromCSV(const char *filename,
		matrix::Matrix<float, NUM_ACTUATORS, NUM_AXES> &mixer, CsvMixerResult *result)
{
	CsvMixerResult local_result{};

	if (result == nullptr) {
		result = &local_result;
	}

	*result = CsvMixerResult{};

	// Open the CSV file
	FILE *file = fopen(filename, "r");

	if (file == NULL) {
		// PX4_WARN("Error: Could not open the mixer file");
		result->status = CsvMixerStatus::FILE_NOT_FOUND;
		return false;
	}

	// Skip a UTF-8 BOM
	unsigned char bom[3];

	if (fread(bom, 1, sizeof(bom), file) != sizeof(bom) || bom[0] != 0xEF || bom[1] != 0xBB || bom[2] != 0xBF) {
		fseek(file, 0, SEEK_SET);
	}

	// Parse into a copy so that a rejected file leaves the mixer untouched
	matrix::Matrix<float, NUM_ACTUATORS, NUM_AXES> parsed = mixer;

	// Read one character at a time so that a line of any length is one row: a full
	// precision export (e.g. -0.35355339059327373) is ~20 characters per cell.
	// Only a single cell has to fit in the buffer.
	char cell[32];
	size_t cell_len = 0;
	int row = 0;
	int col = 0;
	int line = 1;
	bool blank_line = true;
	bool valid = true;

	while (valid && row < NUM_ACTUATORS) {
		const int c = fgetc(file);

		if (c == ',' || c == '\n' || c == EOF) {
			if (c == ',') {
				blank_line = false;
			}

			// Split on every comma. strtok() would merge consecutive commas and shift the
			// rest of the row left on an empty cell; an empty cell reads as 0 instead.
			if (!blank_line) {
				cell[cell_len] = '\0';

				if (col < NUM_AXES) {
					char *end = cell;
					const float value = strtof(cell, &end);

					// Text (e.g. a header row), nan or inf would otherwise silently read as a
					// number. An empty cell (end == cell == "") reads as 0.
					if ((*end != '\0') || !PX4_ISFINITE(value)) {
						result->status = CsvMixerStatus::INVALID_VALUE;
						valid = false;
					}

					printf("Row %d, Col %d: %f\n", row, col, (double)value);
					parsed(row, col) = value;
				}

				col++;
			}

			cell_len = 0;

			if (c != ',') {
				// Blank lines are not rows. A short row would leave the missing axes at
				// whatever the matrix held before.
				if (valid && !blank_line) {
					if (col < NUM_AXES) {
						result->status = CsvMixerStatus::SHORT_ROW;
						valid = false;

					} else {
						row++;
					}
				}

				col = 0;
				blank_line = true;

				if (c == EOF || !valid) {
					break;
				}

				line++;
			}

		} else if (!isspace(c)) {
			// Whitespace, including the \r of a Windows line ending, is dropped
			blank_line = false;

			if (cell_len < sizeof(cell) - 1) {
				cell[cell_len++] = (char)c;

			} else {
				result->status = CsvMixerStatus::INVALID_VALUE; // too long to be a number
				valid = false;
			}
		}
	}

	// Close the file
	fclose(file);

	result->num_rows = row;

	if (!valid) {
		result->line = line;
		return false;
	}

	// An empty file would leave the allocator without a mixer
	if (row == 0) {
		result->status = CsvMixerStatus::EMPTY;
		return false;
	}

	result->status = CsvMixerStatus::LOADED;
	mixer = parsed;
	return true;

}

bool
ControlAllocationPseudoInverse::getMixer(matrix::Matrix<float, NUM_ACTUATORS, NUM_AXES> &mixer)
{
	if (_mix_update_needed) {
		updatePseudoInverse();
	}

	mixer = _mix;
	return true;
}

void
ControlAllocationPseudoInverse::updateControlAllocationMatrixScale()
{
	// Same scale on roll and pitch
	if (_normalize_rpy) {

		int num_non_zero_roll_torque = 0;
		int num_non_zero_pitch_torque = 0;

		for (int i = 0; i < _num_actuators; i++) {

			if (fabsf(_mix(i, 0)) > 1e-3f) {
				++num_non_zero_roll_torque;
			}

			if (fabsf(_mix(i, 1)) > 1e-3f) {
				++num_non_zero_pitch_torque;
			}
		}

		float roll_norm_scale = 1.f;

		if (num_non_zero_roll_torque > 0) {
			roll_norm_scale = sqrtf(_mix.col(0).norm_squared() / (num_non_zero_roll_torque / 2.f));
		}

		float pitch_norm_scale = 1.f;

		if (num_non_zero_pitch_torque > 0) {
			pitch_norm_scale = sqrtf(_mix.col(1).norm_squared() / (num_non_zero_pitch_torque / 2.f));
		}

		_control_allocation_scale(0) = fmaxf(roll_norm_scale, pitch_norm_scale);
		_control_allocation_scale(1) = _control_allocation_scale(0);

		// Scale yaw separately
		_control_allocation_scale(2) = _mix.col(2).max();

	} else {
		_control_allocation_scale(0) = 1.f;
		_control_allocation_scale(1) = 1.f;
		_control_allocation_scale(2) = 1.f;
	}

	// Scale thrust by the sum of the individual thrust axes, and use the scaling for the Z axis if there's no actuators
	// (for tilted actuators)
	_control_allocation_scale(THRUST_Z) = 1.f;

	for (int axis_idx = 2; axis_idx >= 0; --axis_idx) {
		int num_non_zero_thrust = 0;
		float norm_sum = 0.f;

		for (int i = 0; i < _num_actuators; i++) {
			float norm = fabsf(_mix(i, 3 + axis_idx));
			norm_sum += norm;

			if (norm > FLT_EPSILON) {
				++num_non_zero_thrust;
			}
		}

		if (num_non_zero_thrust > 0) {
			_control_allocation_scale(3 + axis_idx) = norm_sum / num_non_zero_thrust;

		} else {
			_control_allocation_scale(3 + axis_idx) = _control_allocation_scale(THRUST_Z);
		}
	}
}

void
ControlAllocationPseudoInverse::normalizeControlAllocationMatrix()
{
	if (_control_allocation_scale(0) > FLT_EPSILON) {
		_mix.col(0) /= _control_allocation_scale(0);
		_mix.col(1) /= _control_allocation_scale(1);
	}

	if (_control_allocation_scale(2) > FLT_EPSILON) {
		_mix.col(2) /= _control_allocation_scale(2);
	}

	if (_control_allocation_scale(3) > FLT_EPSILON) {
		_mix.col(3) /= _control_allocation_scale(3);
		_mix.col(4) /= _control_allocation_scale(4);
		_mix.col(5) /= _control_allocation_scale(5);
	}

	// Set all the small elements to 0 to avoid issues
	// in the control allocation algorithms
	for (int i = 0; i < _num_actuators; i++) {
		for (int j = 0; j < NUM_AXES; j++) {
			if (fabsf(_mix(i, j)) < 1e-3f) {
				_mix(i, j) = 0.f;
			}
		}
	}
}

void
ControlAllocationPseudoInverse::allocate()
{
	//Compute new gains if needed
	updatePseudoInverse();

	_prev_actuator_sp = _actuator_sp;

	// Allocate
	_actuator_sp = _actuator_trim + _mix * (_control_sp - _control_trim);
}
