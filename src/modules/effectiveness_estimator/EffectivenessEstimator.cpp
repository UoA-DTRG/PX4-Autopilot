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
 * @file EffectivenessEstimator.cpp
 *
 * RLS-based effectiveness matrix and mixer estimator
 */

#include "EffectivenessEstimator.hpp"

#include <mathlib/mathlib.h>

using namespace time_literals;
using namespace matrix;

EffectivenessEstimator::EffectivenessEstimator() :
	ModuleParams(nullptr),
	ScheduledWorkItem("effectiveness_estimator", px4::wq_configurations::nav_and_controllers)
{
	updateParams();
	reset();
}

EffectivenessEstimator::~EffectivenessEstimator()
{
	perf_free(_cycle_perf);
}

bool EffectivenessEstimator::init()
{
	if (!_param_eff_est_enable.get()) {
		PX4_INFO("Effectiveness estimator disabled");
		return false;
	}

	// Schedule at the configured update rate (convert Hz to microseconds interval)
	const uint32_t interval_us = static_cast<uint32_t>(1000000.0f / _param_eff_est_update_rate.get());
	ScheduleOnInterval(interval_us);
	return true;
}

void EffectivenessEstimator::reset()
{
	// Initialize covariance matrix
	_P.setZero();
	const float p_init = _param_eff_est_p_init.get();

	for (size_t i = 0; i < MAX_ROTORS * DOF; i++) {
		_P(i, i) = p_init;
	}

	// Initialize parameter vector (effectiveness matrix flattened)
	_theta.setZero();

	// Initialize effectiveness and mixer matrices
	_effectiveness.setZero();
	_mixer.setZero();

	_estimation_valid = false;
	_mixer_valid = false;
}

void EffectivenessEstimator::updateParams()
{
	ModuleParams::updateParams();

	_lambda = _param_eff_est_lambda.get();
	_num_rotors = math::constrain(static_cast<uint8_t>(_param_eff_est_num_rotors.get()), 1, MAX_ROTORS);

	// Update rate is in Hz, need to convert to interval in microseconds
	const uint32_t interval_us = static_cast<uint32_t>(1000000.0f / _param_eff_est_update_rate.get());
	if (ScheduleOnInterval(interval_us) != PX4_OK) {
		PX4_ERR("Failed to update scheduling interval");
	}
}

void EffectivenessEstimator::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);

	// Check for parameter updates
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
	}

	// Update vehicle status
	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status;
		_vehicle_status_sub.copy(&vehicle_status);
		_armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
	}

	// Only estimate when armed
	if (!_armed || !_param_eff_est_enable.get()) {
		perf_end(_cycle_perf);
		return;
	}

	// Update sensor data
	bool data_updated = false;

	if (_actuator_outputs_sub.updated()) {
		actuator_outputs_s actuator_outputs;
		_actuator_outputs_sub.copy(&actuator_outputs);

		for (uint8_t i = 0; i < _num_rotors && i < MAX_ROTORS; i++) {
			_actuator_outputs[i] = actuator_outputs.output[i];
		}

		data_updated = true;
	}

	if (_vehicle_angular_velocity_sub.updated()) {
		vehicle_angular_velocity_s angular_velocity;
		_vehicle_angular_velocity_sub.copy(&angular_velocity);
		
		// Store previous values for differentiation
		_angular_velocity_prev = _angular_velocity;
		_angular_velocity = Vector3f(angular_velocity.xyz);
		
		// Compute angular acceleration via finite difference
		if (_angular_velocity_timestamp_prev > 0) {
			const float dt = (angular_velocity.timestamp - _angular_velocity_timestamp_prev) * 1e-6f;
			if (dt > 0.0f && dt < 1.0f) { // Sanity check
				_angular_acceleration = (_angular_velocity - _angular_velocity_prev) / dt;
			}
		}
		_angular_velocity_timestamp_prev = angular_velocity.timestamp;
		
		data_updated = true;
	}

	if (_vehicle_attitude_sub.updated()) {
		vehicle_attitude_s attitude;
		_vehicle_attitude_sub.copy(&attitude);
		_attitude = Quatf(attitude.q);
		data_updated = true;
	}

	if (data_updated) {
		updateEstimation();
		const hrt_abstime now = hrt_absolute_time();
		publishEstimates(now);
		_timestamp_last = now;
	}

	perf_end(_cycle_perf);
}

void EffectivenessEstimator::updateEstimation()
{
	// RLS update for effectiveness estimation
	// Estimates the effectiveness matrix B where:
	// [Mx, My, Mz]^T = B * u (where u is the actuator output vector)
	
	// For moment estimation, we use:
	// y = measured_moments (from IMU angular acceleration * inertia)
	// phi = actuator outputs (regressor)
	// theta = effectiveness parameters (one row of B per moment axis)
	
	const Vector3f inertia(_param_eff_est_ixx.get(), 
						   _param_eff_est_iyy.get(), 
						   _param_eff_est_izz.get());
	
	// Measured moments in body frame (angular_acceleration * inertia)
	Vector3f measured_moments;
	measured_moments(0) = _angular_acceleration(0) * inertia(0);
	measured_moments(1) = _angular_acceleration(1) * inertia(1);
	measured_moments(2) = _angular_acceleration(2) * inertia(2);
	
	// Construct regressor vector from actuator outputs (only active rotors)
	matrix::Vector<float, MAX_ROTORS> phi;
	phi.setZero();
	
	for (uint8_t i = 0; i < _num_rotors && i < MAX_ROTORS; i++) {
		phi(i) = _actuator_outputs[i];
	}
	
	// Check if we have enough excitation (actuators not at zero)
	float actuator_norm = phi.norm();
	const float min_excitation = _param_eff_est_min_excite.get();
	if (actuator_norm < min_excitation) {
		// Not enough excitation, skip this sample
		return;
	}
	
	// Update RLS for each moment axis (Mx, My, Mz)
	// For simplicity, we update only the moment axes (indices 3, 4, 5 in DOF)
	// Full implementation would also estimate force axes (0, 1, 2)
	for (size_t moment_idx = 3; moment_idx < DOF; moment_idx++) {
		size_t axis_idx = moment_idx - 3; // 0=Mx, 1=My, 2=Mz
		
		// Extract parameter vector for this axis
		matrix::Vector<float, MAX_ROTORS> theta_axis;
		for (size_t j = 0; j < MAX_ROTORS; j++) {
			size_t param_idx = moment_idx * MAX_ROTORS + j;
			theta_axis(j) = _theta(param_idx);
		}
		
		// Extract covariance sub-matrix for this axis
		matrix::Matrix<float, MAX_ROTORS, MAX_ROTORS> P_axis;
		for (size_t i = 0; i < MAX_ROTORS; i++) {
			for (size_t j = 0; j < MAX_ROTORS; j++) {
				size_t row = moment_idx * MAX_ROTORS + i;
				size_t col = moment_idx * MAX_ROTORS + j;
				P_axis(i, j) = _P(row, col);
			}
		}
		
		// RLS update equations:
		// K = P * phi / (lambda + phi' * P * phi)
		// theta_new = theta + K * (y - phi' * theta)
		// P_new = (P - K * phi' * P) / lambda
		
		const matrix::Vector<float, MAX_ROTORS> P_phi = P_axis * phi;
		const float phi_P_phi = (phi.transpose() * P_phi)(0, 0);
		const float denominator = _lambda + phi_P_phi;
		
		if (fabsf(denominator) < RLS_NUMERICAL_EPSILON) {
			continue; // Avoid division by zero
		}
		
		const matrix::Vector<float, MAX_ROTORS> K = P_phi / denominator;
		
		// Innovation: y - phi' * theta
		const float prediction = (phi.transpose() * theta_axis)(0, 0);
		const float innovation = measured_moments(axis_idx) - prediction;
		_innovations(axis_idx) = innovation; // Store innovation per axis
		
		// Update parameters
		theta_axis = theta_axis + K * innovation;
		
		// Update covariance
		P_axis = (P_axis - K * phi.transpose() * P_axis) / _lambda;
		
		// Store updated parameters back
		for (size_t j = 0; j < MAX_ROTORS; j++) {
			size_t param_idx = moment_idx * MAX_ROTORS + j;
			_theta(param_idx) = theta_axis(j);
		}
		
		// Store updated covariance back
		for (size_t i = 0; i < MAX_ROTORS; i++) {
			for (size_t j = 0; j < MAX_ROTORS; j++) {
				size_t row = moment_idx * MAX_ROTORS + i;
				size_t col = moment_idx * MAX_ROTORS + j;
				_P(row, col) = P_axis(i, j);
			}
		}
	}
	
	_sample_count++;
	
	// Update effectiveness matrix from theta (only moment rows to avoid stale data)
	// Force rows (0-2) remain zero as we only estimate moments
	for (size_t i = 3; i < DOF; i++) { // Only update moment axes
		for (size_t j = 0; j < _num_rotors && j < MAX_ROTORS; j++) {
			size_t param_idx = i * MAX_ROTORS + j;
			_effectiveness(i, j) = _theta(param_idx);
		}
	}
	
	// Check convergence: sufficient samples and low innovation variance
	if (_sample_count >= MIN_SAMPLES_FOR_CONVERGENCE) {
		// Compute variance of P diagonal for active parameters
		float avg_variance = 0.0f;
		uint32_t param_count = 0;
		
		for (size_t i = 3; i < DOF; i++) { // Only check moment axes
			for (size_t j = 0; j < _num_rotors && j < MAX_ROTORS; j++) {
				size_t param_idx = i * MAX_ROTORS + j;
				avg_variance += _P(param_idx, param_idx);
				param_count++;
			}
		}
		
		if (param_count > 0) {
			avg_variance /= param_count;
			
			// Compute RMS innovation across all moment axes
			float rms_innovation = _innovations.norm() / sqrtf(3.0f);
			
			// Consider converged if average variance and innovation are low enough
			const float variance_threshold = _param_eff_est_conv_var.get();
			const float innovation_threshold = _param_eff_est_conv_innov.get();
			
			_estimation_valid = (avg_variance < variance_threshold) && (rms_innovation < innovation_threshold);
		}
	}
}

bool EffectivenessEstimator::computeMixer()
{
	// Compute pseudo-inverse of effectiveness matrix to get mixer
	// mixer = pinv(effectiveness)
	
	// Extract the active portion of the effectiveness matrix (only moment axes)
	// For a quadcopter: 3 moment axes x N rotors
	static constexpr size_t MOMENT_AXES = 3;
	matrix::Matrix<float, MOMENT_AXES, MAX_ROTORS> B_moments;
	B_moments.setZero(); // Initialize to zero to avoid uninitialized memory
	
	for (size_t i = 0; i < MOMENT_AXES; i++) {
		for (size_t j = 0; j < _num_rotors && j < MAX_ROTORS; j++) {
			B_moments(i, j) = _effectiveness(i + 3, j); // Rows 3,4,5 are Mx,My,Mz
		}
	}
	
	// Compute pseudo-inverse using geninv from matrix library
	matrix::Matrix<float, MAX_ROTORS, MOMENT_AXES> mixer_moments;
	
	if (!matrix::geninv(B_moments, mixer_moments)) {
		_mixer_valid = false;
		return false;
	}
	
	// Store the mixer (expand to full DOF x rotors, zero-fill force rows)
	_mixer.setZero();
	
	for (size_t i = 0; i < _num_rotors && i < MAX_ROTORS; i++) {
		for (size_t j = 0; j < MOMENT_AXES; j++) {
			_mixer(i, j + 3) = mixer_moments(i, j); // Columns 3,4,5 are Mx,My,Mz
		}
	}
	
	_mixer_valid = true;
	return _mixer_valid;
}

void EffectivenessEstimator::publishEstimates(const hrt_abstime &timestamp)
{
	// Publish effectiveness estimate
	effectiveness_estimate_s eff_est{};
	eff_est.timestamp = timestamp;
	eff_est.num_rotors = _num_rotors;

	// Copy effectiveness matrix (6 x num_rotors, row-major storage)
	// Storage order: [rotor0_fx, rotor1_fx, ..., rotor15_fx, rotor0_fy, ...]
	for (size_t i = 0; i < DOF; i++) {
		for (size_t j = 0; j < _num_rotors && j < MAX_ROTORS; j++) {
			eff_est.effectiveness_matrix[i * MAX_ROTORS + j] = _effectiveness(i, j);
		}
	}

	eff_est.estimation_variance = 0.0f; // Compute average variance from P
	eff_est.innovation = _innovations.norm() / sqrtf(3.0f); // RMS innovation
	eff_est.estimation_valid = _estimation_valid;
	
	// Compute average variance from covariance matrix diagonal
	float variance_sum = 0.0f;
	uint32_t variance_count = 0;
	for (size_t i = 3; i < DOF; i++) { // Only moment axes
		for (size_t j = 0; j < _num_rotors && j < MAX_ROTORS; j++) {
			size_t param_idx = i * MAX_ROTORS + j;
			variance_sum += _P(param_idx, param_idx);
			variance_count++;
		}
	}
	if (variance_count > 0) {
		eff_est.estimation_variance = variance_sum / variance_count;
	}

	eff_est.mass = _param_eff_est_mass.get();
	eff_est.inertia_diagonal[0] = _param_eff_est_ixx.get();
	eff_est.inertia_diagonal[1] = _param_eff_est_iyy.get();
	eff_est.inertia_diagonal[2] = _param_eff_est_izz.get();

	_effectiveness_estimate_pub.publish(eff_est);

	// Publish mixer estimate if valid
	if (computeMixer()) {
		mixer_estimate_s mixer_est{};
		mixer_est.timestamp = timestamp;
		mixer_est.num_rotors = _num_rotors;

		// Copy mixer matrix (num_rotors x 6, row-major format)
		// Row i represents how rotor i responds to [Fx, Fy, Fz, Mx, My, Mz]
		// Storage: [fx_to_rotor0, fy_to_rotor0, ..., mz_to_rotor0, fx_to_rotor1, ...]
		for (size_t i = 0; i < _num_rotors && i < MAX_ROTORS; i++) {
			for (size_t j = 0; j < DOF; j++) {
				mixer_est.mixer_matrix[i * DOF + j] = _mixer(i, j);
			}
		}

		mixer_est.mixer_valid = _mixer_valid;
		_mixer_estimate_pub.publish(mixer_est);
	}
}

int EffectivenessEstimator::task_spawn(int argc, char *argv[])
{
	EffectivenessEstimator *instance = new EffectivenessEstimator();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int EffectivenessEstimator::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int EffectivenessEstimator::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
RLS-based effectiveness matrix and mixer estimator for multicopter vehicles.

This module estimates the actuator effectiveness matrix using Recursive Least Squares (RLS)
and computes the corresponding mixer matrix. The estimates are logged for validation but
not actively used in the control allocation.

### Implementation
The estimator uses vehicle angular acceleration and actuator outputs to identify the
effectiveness matrix that maps actuator commands to vehicle moments and forces.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("effectiveness_estimator", "estimator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int EffectivenessEstimator::print_status()
{
	PX4_INFO("Running");
	PX4_INFO("Armed: %s", _armed ? "yes" : "no");
	PX4_INFO("Estimation valid: %s", _estimation_valid ? "yes" : "no");
	PX4_INFO("Mixer valid: %s", _mixer_valid ? "yes" : "no");
	PX4_INFO("Number of rotors: %d", _num_rotors);
	PX4_INFO("Forgetting factor: %.4f", (double)_lambda);

	perf_print_counter(_cycle_perf);

	return 0;
}

extern "C" __EXPORT int effectiveness_estimator_main(int argc, char *argv[])
{
	return EffectivenessEstimator::main(argc, argv);
}
