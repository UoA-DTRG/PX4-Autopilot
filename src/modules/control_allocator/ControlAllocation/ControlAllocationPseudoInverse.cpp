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
		matrix::geninv(_effectiveness, _mix);

		if (_normalization_needs_update && !_had_actuator_failure) {
			updateControlAllocationMatrixScale();
			_normalization_needs_update = false;
		}

		normalizeControlAllocationMatrix();
		_mix_update_needed = false;
	}
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

	// //jspa778: implement chirp mixer
	// //roll
	// _mix(0,0) = -0.25222f;
	// _mix(1,0) = 0.25525f;
	// _mix(2,0) = 0.2537f;
	// _mix(3,0) = -0.27558f;
	// _mix(4,0) = 0.45252f;
	// _mix(5,0) = -0.45564f;
	// _mix(6,0) = -0.44812f;
	// _mix(7,0) = 0.43539f;
	// //pitch
	// _mix(0,1) = 0.49465f;
	// _mix(1,1) = 0.48587f;
	// _mix(2,1) = -0.47741f;
	// _mix(3,1) = -0.49259f;
	// _mix(4,1) = 0.22445f;
	// _mix(5,1) = 0.2242f;
	// _mix(6,1) = -0.20542f;
	// _mix(7,1) = -0.22384f;
	// //yaw
	// _mix(0,2) = 0.9144f;
	// _mix(1,2) = -0.90835f;
	// _mix(2,2) = 1.1137f;
	// _mix(3,2) = -1.0857f;
	// _mix(4,2) = 1.048f;
	// _mix(5,2) = -0.96098f;
	// _mix(6,2) = 0.95769f;
	// _mix(7,2) = -1.0112f;
	// //x
	// _mix(0,3) = -0.89532f;
	// _mix(1,3) = -0.92996f;
	// _mix(2,3) = 0.97146f;
	// _mix(3,3) = 1.0357f;
	// _mix(4,3) = 1.0788f;
	// _mix(5,3) = 1.0948f;
	// _mix(6,3) = -1.0037f;
	// _mix(7,3) = -0.9902f;
	// //y
	// _mix(0,4) = 1.0952f;
	// _mix(1,4) = -1.162f;
	// _mix(2,4) = -1.1055f;
	// _mix(3,4) = 1.1519f;
	// _mix(4,4) = 0.87168f;
	// _mix(5,4) = -0.87041f;
	// _mix(6,4) = -0.88265f;
	// _mix(7,4) = 0.86065f;
	// //z
	// _mix(0,5) = -0.98817f;
	// _mix(1,5) = -1.0066f;
	// _mix(2,5) = -0.97947f;
	// _mix(3,5) = -1.0256f;
	// _mix(4,5) = -0.93157f;
	// _mix(5,5) = -0.98996f;
	// _mix(6,5) = -1.0363f;
	// _mix(7,5) = -1.0423f;

	//-------------------------------------------------------

	//jspa778: implement random SP mixer
	//roll
	// _mix(0,0) = -0.28859f;
	// _mix(1,0) = 0.31047f;
	// _mix(2,0) = 0.28712f;
	// _mix(3,0) = -0.29858f;
	// _mix(4,0) = 0.41506f;
	// _mix(5,0) = -0.39502f;
	// _mix(6,0) = -0.41471f;
	// _mix(7,0) = 0.41889f;
	// pitch
	// _mix(0,1) = 0.42128f;
	// _mix(1,1) = 0.41901f;
	// _mix(2,1) = -0.40344f;
	// _mix(3,1) = -0.39879f;
	// _mix(4,1) = 0.30357f;
	// _mix(5,1) = 0.32042f;
	// _mix(6,1) = -0.27229f;
	// _mix(7,1) = -0.28963f;
	// yaw
	// _mix(0,2) = 1.0326f;
	// _mix(1,2) = -1.0952f;
	// _mix(2,2) = 0.88681f;
	// _mix(3,2) = -0.91446f;
	// _mix(4,2) = 0.99959f;
	// _mix(5,2) = -1.0617f;
	// _mix(6,2) = 0.98442f;
	// _mix(7,2) = -1.0252f;
	// x
	// _mix(0,3) = -0.94905f;
	// _mix(1,3) = -0.89274f;
	// _mix(2,3) = 0.95466f;
	// _mix(3,3) = 0.96883f;
	// _mix(4,3) = 1.1201f;
	// _mix(5,3) = 1.0377f;
	// _mix(6,3) = -1.1117f;
	// _mix(7,3) = -0.96512f;
	// y
	// _mix(0,4) = 1.066f;
	// _mix(1,4) = -1.1386f;
	// _mix(2,4) = -1.1036f;
	// _mix(3,4) = 1.1071f;
	// _mix(4,4) = 0.92531f;
	// _mix(5,4) = -0.85677f;
	// _mix(6,4) = -0.965f;
	// _mix(7,4) = 0.83754f;
	// z
	// _mix(0,5) = -0.97246f;
	// _mix(1,5) = -0.99584f;
	// _mix(2,5) = -1.0027f;
	// _mix(3,5) = -1.0292f;
	// _mix(4,5) = -0.95419f;
	// _mix(5,5) = -1.0133f;
	// _mix(6,5) = -1.021f;
	// _mix(7,5) = -1.0114f;


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
