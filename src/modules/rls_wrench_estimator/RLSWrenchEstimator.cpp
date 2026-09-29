/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file RLSWrenchEstimator.cpp
 * @brief RLS parameter identification and external wrench estimator
 *
 * @author Pedro Mendes <pmen817@aucklanduni.ac.nz>
 */

#include "RLSWrenchEstimator.hpp"

using matrix::Vector3f;

namespace
{
// Revolutions per minute to rad/s (2*pi/60 = pi/30)
static constexpr float RPM_TO_RADS = 0.104719755f;
// Maximum age of an esc_status message for its RPM to be trusted
static constexpr hrt_abstime ESC_STATUS_TIMEOUT = 100_ms;
}

RLSWrenchEstimator::RLSWrenchEstimator() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	_valid_hysteresis.set_hysteresis_time_from(false, 2_s);
	initRotorParamHandles();
	updateParams();
}

RLSWrenchEstimator::~RLSWrenchEstimator()
{
	perf_free(_cycle_perf);
}

bool RLSWrenchEstimator::init()
{
	// execute Run() on every vehicle_acceleration publication
	if (!_vehicle_acceleration_sub.registerCallback()) {
		PX4_ERR("vehicle_acceleration callback registration failed");
		return false;
	}

	return true;
}

void RLSWrenchEstimator::initRotorParamHandles()
{
	_param_handle_rotor_count = param_find("CA_ROTOR_COUNT");

	char buffer[17];

	for (int i = 0; i < MAX_ROTORS; i++) {
		snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_PX", i);
		_rotor_handles[i].position_x = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_PY", i);
		_rotor_handles[i].position_y = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_PZ", i);
		_rotor_handles[i].position_z = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_AX", i);
		_rotor_handles[i].axis_x = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_AY", i);
		_rotor_handles[i].axis_y = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_AZ", i);
		_rotor_handles[i].axis_z = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_CT", i);
		_rotor_handles[i].thrust_coef = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_KM", i);
		_rotor_handles[i].moment_ratio = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "RLS_ROTOR%u_GRP", i);
		_rotor_handles[i].group = param_find(buffer);
	}
}

void RLSWrenchEstimator::updateGeometry(VehicleParameters &params)
{
	params.mass = _param_rls_mass.get();
	params.lpf_motor_tau = _param_rls_lpf_motor.get();
	params.n_groups = math::constrain((int)_param_rls_n_grp.get(), 1, RLS_MAX_GROUPS);

	int32_t rotor_count = 0;

	if (_param_handle_rotor_count != PARAM_INVALID) {
		param_get(_param_handle_rotor_count, &rotor_count);
	}

	params.num_rotors = math::constrain((int)rotor_count, 0, MAX_ROTORS);

	for (int i = 0; i < MAX_ROTORS; i++) {
		Vector3f position(0.f, 0.f, 0.f);
		Vector3f axis(0.f, 0.f, -1.f);
		float thrust_coef = 1.f;
		float moment_ratio = 0.05f;
		int32_t group = 0;

		const RotorParamHandles &h = _rotor_handles[i];

		if (h.position_x != PARAM_INVALID) { param_get(h.position_x, &position(0)); }

		if (h.position_y != PARAM_INVALID) { param_get(h.position_y, &position(1)); }

		if (h.position_z != PARAM_INVALID) { param_get(h.position_z, &position(2)); }

		if (h.axis_x != PARAM_INVALID) { param_get(h.axis_x, &axis(0)); }

		if (h.axis_y != PARAM_INVALID) { param_get(h.axis_y, &axis(1)); }

		if (h.axis_z != PARAM_INVALID) { param_get(h.axis_z, &axis(2)); }

		if (h.thrust_coef != PARAM_INVALID) { param_get(h.thrust_coef, &thrust_coef); }

		if (h.moment_ratio != PARAM_INVALID) { param_get(h.moment_ratio, &moment_ratio); }

		if (h.group != PARAM_INVALID) { param_get(h.group, &group); }

		// Normalize the thrust axis
		const float axis_norm = axis.norm();

		if (axis_norm > 1e-6f) {
			axis /= axis_norm;

		} else {
			axis = Vector3f(0.f, 0.f, -1.f);
		}

		params.position[i] = position;
		params.axis[i] = axis;
		params.thrust_coef[i] = thrust_coef;
		params.moment_ratio[i] = moment_ratio;
		params.motor_group[i] = math::constrain((int)group, 0, params.n_groups - 1);
	}
}

void RLSWrenchEstimator::updateParams()
{
	ModuleParams::updateParams();

	if (!_in_air) {
		VehicleParameters vehicle_params{};
		updateGeometry(vehicle_params);

		_num_rotors = vehicle_params.num_rotors;
		_n_groups = vehicle_params.n_groups;

		for (int i = 0; i < MAX_ROTORS; i++) {
			_motor_group[i] = vehicle_params.motor_group[i];
		}

		const float initial_guess[3] = {
			_param_rls_kf_init.get(),
			_param_rls_xo_init.get() * 1000.f,
			_param_rls_yo_init.get() * 1000.f
		};

		const float initial_confidence[3] = {
			_param_rls_kf_conf.get(),
			_param_rls_xo_conf.get(),
			_param_rls_yo_conf.get()
		};

		const float R_diag[5] = {
			_param_rls_xy_noise.get(),
			_param_rls_xy_noise.get(),
			_param_rls_z_noise.get(),
			_param_rls_f_noise.get(),
			_param_rls_f_noise.get()
		};

		const Vector3f inertia_diag = {
			_param_rls_inertia_x.get() * (1E3f),
			_param_rls_inertia_y.get() * (1E3f),
			_param_rls_inertia_z.get() * (1E3f)
		};

		_identification.initialize(initial_guess, initial_confidence, R_diag, vehicle_params);
		_wrench_estimator.initialize(_param_rls_lpf_force.get(), _param_rls_lpf_moment.get(), inertia_diag);
	}
}

void RLSWrenchEstimator::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	if (_vehicle_land_detected_sub.updated()) {
		vehicle_land_detected_s vehicle_land_detected;

		if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
			_landed = vehicle_land_detected.landed;

			if (_landed) {
				_in_air = false;
			}
		}
	}

	if (!_vehicle_acceleration_sub.updated()) {
		return;
	}

	if (_vehicle_local_position_sub.updated()) {
		vehicle_local_position_s local_pos{};

		if (_vehicle_local_position_sub.copy(&local_pos)) {
			if (!_landed) {
				if (local_pos.dist_bottom > 0.3f) {
					_in_air = true;
				}
			}
		}
	}

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams(); // update module parameters (in DEFINE_PARAMETERS)
	}

	perf_begin(_cycle_perf);

	vehicle_acceleration_s accel;
	actuator_outputs_s actuator_outputs;
	vehicle_attitude_s v_att;
	vehicle_angular_velocity_s v_ang_vel;
	battery_status_s batt_stat;

	_finite = copyAndCheckAllFinite(accel, actuator_outputs, v_att, v_ang_vel, batt_stat);

	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status;

		if (_vehicle_status_sub.copy(&vehicle_status)) {
			_armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
		}
	}

	const float dt = (accel.timestamp - _timestamp_last) * 1e-6f;
	_timestamp_last = accel.timestamp;

	if (_debug_vect_sub.updated()) {
		debug_vect_s flags_vect;
		_debug_vect_sub.copy(&flags_vect);
		_interaction_flag = (flags_vect.x > 0.5f);

		_debug_timestamp_last = hrt_absolute_time();
	}

	if (hrt_elapsed_time(&_debug_timestamp_last) > 1_s) {
		_interaction_flag = false; //timeout case external link lost
	}

// Guard against too small (< 0.2ms) and too large (> 20ms) dt's.
	if (_finite && _armed && _in_air && (dt > 0.0002f) && (dt < 0.02f)) {

		const Vector3f acc = Vector3f(accel.xyz[0], accel.xyz[1], accel.xyz[2]);

		_voltage = math::constrain(batt_stat.voltage_v, _param_n_cells.get() * 3.0f, _param_n_cells.get() * 4.2f);

		// Motor speed [rad/s]: measured ESC RPM when selected/available, otherwise the PWM model.
		const bool use_esc = (_param_rls_spd_src.get() == 1);
		const bool apply_lpf = !use_esc; // measured ESC speed needs no motor-dynamics filter

		esc_status_s esc_status{};
		bool esc_fresh = false;

		if (use_esc && _esc_status_sub.copy(&esc_status)) {
			esc_fresh = (hrt_elapsed_time(&esc_status.timestamp) < ESC_STATUS_TIMEOUT);
		}

		const float voltage_correction = sqrtf(powf((_voltage / _param_rls_speed_v1.get()), _param_rls_speed_v2.get()));

		float speed[RLS_MAX_ROTORS] = {};

		for (int i = 0; i < _num_rotors; i++) {
			// PWM model (also the per-motor fall-back in ESC mode)
			float s = ((actuator_outputs.output[i] * _param_rls_speed_p1.get()) - _param_rls_speed_p2.get())
				  * voltage_correction;

			if (use_esc && esc_fresh && (i < esc_status.esc_count) && (i < esc_status_s::CONNECTED_ESC_MAX)) {
				const bool online = (esc_status.esc_online_flags & (1 << i));
				const int32_t rpm = esc_status.esc[i].esc_rpm;

				if (online && (rpm != 0)) {
					s = fabsf((float)rpm) * RPM_TO_RADS;
				}
			}

			speed[i] = s;
		}

		const matrix::Vector<float, RLS_MAX_ROTORS> output = matrix::Vector<float, RLS_MAX_ROTORS>(speed);

		//RLS Thrust
		_identification.updateThrust(acc, output, dt, _interaction_flag, apply_lpf);

		const Vector3f p_error_t = _identification.getPredictionErrorThrust();
		const matrix::Quatf q{v_att.q};

		//Wrench Estimator Thrust
		_wrench_estimator.updateForce(p_error_t, dt, _interaction_flag);
		// --------------------------------------------------------- //

		//RLS Offset
		_identification.updateOffset(q, _interaction_flag);

		Vector3f p_error_o = _identification.getPredictionErrorOffset();

		//Wrench Estimator Moment
		_wrench_estimator.updateMoment(p_error_o, Vector3f(v_ang_vel.xyz), dt, _interaction_flag);

		// Check validity of results - IMPROVE
		bool valid = true;

		_valid_hysteresis.set_state_and_update(valid, accel.timestamp);
		_valid = _valid_hysteresis.get_state();

		publishStatus();

	} else {
		_valid_hysteresis.set_state_and_update(false, hrt_absolute_time());

		publishInvalidStatus();

		if (_valid) {
			updateParams(); //reset RLS when landing
			_valid = false;
		}
	}

	perf_end(_cycle_perf);
}

void RLSWrenchEstimator::publishStatus()
{
	rls_wrench_estimator_s status_msg{};
	Vector3f Fe = _wrench_estimator.getExternalForce();
	Vector3f Me = _wrench_estimator.getExternalMoment();
	matrix::Vector<float, RLS_MAX_GROUPS> kf_groups = _identification.getEstimationThrust();
	Vector3f params_offset = _identification.getEstimationOffset();
	Vector3f Fi = _identification.getActuatorForceVector();
	Vector3f Mi = _identification.getActuatorMomentVector();

	status_msg.timestamp = hrt_absolute_time();

	status_msg.fe[0] = Fe(0);
	status_msg.fe[1] = Fe(1);
	status_msg.fe[2] = Fe(2);

	status_msg.me[0] = Me(0);
	status_msg.me[1] = Me(1);
	status_msg.me[2] = Me(2);

	status_msg.fi[0] = Fi(0);
	status_msg.fi[1] = Fi(1);
	status_msg.fi[2] = Fi(2);

	status_msg.mi[0] = Mi(0);
	status_msg.mi[1] = Mi(1);
	status_msg.mi[2] = Mi(2);

	// Expand the per-group thrust constant to each motor
	for (int i = 0; i < (int)(sizeof(status_msg.k_f) / sizeof(status_msg.k_f[0])); i++) {
		if (i < _num_rotors) {
			const int g = _motor_group[i];
			status_msg.k_f[i] = kf_groups(g);
			status_msg.motor_group[i] = (uint8_t)g;

		} else {
			status_msg.k_f[i] = NAN;
			status_msg.motor_group[i] = 0;
		}
	}

	status_msg.n_groups = (uint8_t)_n_groups;

	status_msg.x_offset[0] = params_offset(0);
	status_msg.x_offset[1] = params_offset(1);
	status_msg.x_offset[2] = params_offset(2);

	status_msg.interaction_flag = _interaction_flag;
	status_msg.valid = _valid;

	_rls_wrench_estimator_pub.publish(status_msg);
}

void RLSWrenchEstimator::publishInvalidStatus()
{
	rls_wrench_estimator_s status_msg{};

	status_msg.timestamp = hrt_absolute_time();

	status_msg.fe[0] = 0.f;
	status_msg.fe[1] = 0.f;
	status_msg.fe[2] = 0.f;

	status_msg.me[0] = 0.f;
	status_msg.me[1] = 0.f;
	status_msg.me[2] = 0.f;

	status_msg.fi[0] = NAN;
	status_msg.fi[1] = NAN;
	status_msg.fi[2] = NAN;

	status_msg.mi[0] = NAN;
	status_msg.mi[1] = NAN;
	status_msg.mi[2] = NAN;

	for (int i = 0; i < (int)(sizeof(status_msg.k_f) / sizeof(status_msg.k_f[0])); i++) {
		status_msg.k_f[i] = NAN;
		status_msg.motor_group[i] = 0;
	}

	status_msg.n_groups = 0;

	status_msg.x_offset[0] = NAN;
	status_msg.x_offset[1] = NAN;
	status_msg.x_offset[2] = NAN;

	status_msg.interaction_flag = _interaction_flag;
	status_msg.valid = false;

	_rls_wrench_estimator_pub.publish(status_msg);
}

bool RLSWrenchEstimator::copyAndCheckAllFinite(vehicle_acceleration_s &accel, actuator_outputs_s &actuator_outputs,
		vehicle_attitude_s &v_att, vehicle_angular_velocity_s &v_ang_vel, battery_status_s &batt_stat)
{
	_vehicle_acceleration_sub.copy(&accel);

	if (!(PX4_ISFINITE(accel.xyz[0]) && PX4_ISFINITE(accel.xyz[1]) && PX4_ISFINITE(accel.xyz[2]))) {

		return false;
	}


	_vehicle_attitude_sub.copy(&v_att);

	if (!(PX4_ISFINITE(v_att.q[0]) && PX4_ISFINITE(v_att.q[1]) && PX4_ISFINITE(v_att.q[2]) && PX4_ISFINITE(v_att.q[3]))) {

		return false;
	}

	_vehicle_angular_velocity_sub.copy(&v_ang_vel);

	if (!(PX4_ISFINITE(v_ang_vel.xyz[0]) && PX4_ISFINITE(v_ang_vel.xyz[1]) && PX4_ISFINITE(v_ang_vel.xyz[2]))) {

		return false;
	}

	_battery_status_sub.copy(&batt_stat);

	if (!(PX4_ISFINITE(batt_stat.voltage_v))) {

		_voltage = 11.7f;
	}


	_actuator_outputs_sub.copy(&actuator_outputs);

	for (int i = 0; i < _num_rotors; i++) {
		if (!PX4_ISFINITE(actuator_outputs.output[i])) {return false;}
	}

	return true;
}

int RLSWrenchEstimator::task_spawn(int argc, char *argv[])
{
	RLSWrenchEstimator *instance = new RLSWrenchEstimator();

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

int RLSWrenchEstimator::print_status()
{
	perf_print_counter(_cycle_perf);
	return 0;
}

int RLSWrenchEstimator::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int RLSWrenchEstimator::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
RLS parameter identification and external wrench estimator.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rls_wrench_estimator", "estimator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int rls_wrench_estimator_main(int argc, char *argv[])
{
	return RLSWrenchEstimator::main(argc, argv);
}
