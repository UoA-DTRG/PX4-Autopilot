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
 * @file ControlAllocationPseudoInverse.hpp
 *
 * Simple Control Allocation Algorithm
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */
#include <cstdio>
#include <cstdlib>
#include "ControlAllocationPseudoInverse.hpp"

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
}

void
ControlAllocationPseudoInverse::updatePseudoInverse()
{
	if (_mix_update_needed) {
		//csv ovveride
		if (_csv_mixer.get()) {
			// PX4_INFO("loading mixer from csv file using DTRG mixer override");
			if (readMixerFromCSV("/fs/microsd/etc/mixer.csv", _mix)) {
				_normalization_needs_update = false; //check if this is desired

			} else {
				// PX4_ERR("failed to load mixer from csv file");
			}

		} else {
			matrix::geninv(_effectiveness, _mix);
		}

		if (_normalization_needs_update && !_had_actuator_failure) {
			updateControlAllocationMatrixScale();
			_normalization_needs_update = false;
		}

		normalizeControlAllocationMatrix();
		_mix_update_needed = false;
	}
}

bool
ControlAllocationPseudoInverse::readMixerFromCSV(const char *filename,
		matrix::Matrix<float, NUM_ACTUATORS, NUM_AXES> &mixer)
{
	// Open the CSV file
	FILE *file = fopen(filename, "r");

	if (file == NULL) {
		// PX4_WARN("Error: Could not open the mixer file");
		return 0;

	} else {
		char value[100]; //  just needs to be rougly > 70
		int row = 0;

		while (row < NUM_ACTUATORS && fgets(value, sizeof(value), file) != NULL) {
			size_t len = strlen(value);

			// Check and remove newline character if present
			if (len > 0 && value[len - 1] == '\n') {
				value[len - 1] = '\0';
			}

			// Check and remove BOM
			if (row == 0 && len > 3 && (uint8_t)value[0] == 0xEF && (uint8_t)value[1] == 0xBB && (uint8_t)value[2] == 0xBF) {
				// This is a UTF-8 BOM
				len -= 3;
				memmove(value, &value[3], len);
			}

			// strtok function is used to split the string into tokens
			char *token = strtok(value, ",");
			int col = 0;

			while (token != NULL && col < NUM_AXES) {
				if (strlen(token) > 0) {
					printf("Row %d, Col %d: %f\n", row, col, strtod(token, NULL));
					mixer(row, col) =  strtof(token, NULL);
				}

				token = strtok(NULL, ",");
				col++;
			}

			if (col > 0) { //row protection
				row++;
			}
		}
	}

	// Close the file
	fclose(file);
	return 1;

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
