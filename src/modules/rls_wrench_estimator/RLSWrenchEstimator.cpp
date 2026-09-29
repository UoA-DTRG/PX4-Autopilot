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

#include <string.h>

using matrix::Vector3f;

namespace
{
// Revolutions per minute to rad/s (2*pi/60 = pi/30)
static constexpr float RPM_TO_RADS = 0.104719755f;
// Maximum age of an esc_status message for its RPM to be trusted
static constexpr hrt_abstime ESC_STATUS_TIMEOUT = 100_ms;
// Maximum age of an actuator_outputs message for it to be used
static constexpr hrt_abstime ACTUATOR_OUTPUTS_TIMEOUT = 100_ms;
// How often the actuator_outputs instances are probed while none is selected
static constexpr hrt_abstime ACTUATOR_OUTPUTS_SCAN_INTERVAL = 1_s;
}

RLSWrenchEstimator::RLSWrenchEstimator() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::wrench_est)
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
	// execute Run() on vehicle_acceleration publications, limited to 100 Hz
	if (!_vehicle_acceleration_sub.registerCallback()) {
		PX4_ERR("vehicle_acceleration callback registration failed");
		return false;
	}

	// vehicle_acceleration is published at the IMU rate, far faster than this
	// estimator needs. Running every publication burns CPU that lower-priority work
	// such as mavlink needs, so the update is limited to 100 Hz.
	_vehicle_acceleration_sub.set_interval_us(10_ms);

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

	// Rotation from the frame the IMU and attitude are reported in to the frame the
	// rotor geometry (CA_ROTOR*) is defined in. A few degrees here turn a large
	// vertical thrust into a lateral force the vehicle never feels, so this is
	// applied to the measurements before the residual that becomes the wrench
	// estimate is formed. Safe to update in flight: it changes no RLS state.
	_q_align = matrix::Quatf(matrix::Eulerf(math::radians(_param_rls_align_roll.get()),
						math::radians(_param_rls_align_pitch.get()), 0.f));
	_R_align = matrix::Dcmf(_q_align);

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

		// RLS_EST_I** are given in g m^2 (SI inertia x 1e3), while the moment vector
		// they are combined with is in N m, so they have to be scaled back to SI.
		const Vector3f inertia_diag = {
			_param_rls_inertia_x.get() * (1E-3f),
			_param_rls_inertia_y.get() * (1E-3f),
			_param_rls_inertia_z.get() * (1E-3f)
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

				// An alignment estimate belongs to the flight it was taken in. The
				// vehicle gets refitted between flights and the misalignment moves with
				// it, but the coverage gate only knows how far the vehicle turned - a
				// stale estimate would still pass it and be saved without any warning.
				resetAlignment();
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
		_interaction_flag_external = (flags_vect.x > 0.5f);

		_debug_timestamp_last = hrt_absolute_time();
	}

	if (hrt_elapsed_time(&_debug_timestamp_last) > 1_s) {
		_interaction_flag_external = false; //timeout case external link lost
	}

	// A console stage ("rls_wrench_estimator stage ...") wins over the external flag
	// until it is set back to auto.
	if (_alignment_reset_request.load()) {
		_alignment_reset_request.store(false);
		resetAlignment();
	}

	const Stage stage_prev = _stage;
	const int32_t stage_override = getStageOverride();

	if (stage_override >= 0) {
		_stage = (Stage)stage_override;

	} else {
		_stage = _interaction_flag_external ? Stage::Interact : Stage::Identify;
	}

	if ((_stage == Stage::Align) && (stage_prev != Stage::Align)) {
		resetAlignment(); // each alignment run starts from the parameters, not from the last run
	}

	_interaction_flag = (_stage == Stage::Interact);

	// The identification is frozen for anything past Identify: from Align onwards the
	// residual is the measurement, so letting k_f keep adapting would consume it.
	const bool freeze_identification = (_stage != Stage::Identify);

// Guard against too small (< 0.2ms) and too large (> 20ms) dt's.
	if (_finite && _armed && _in_air && (dt > 0.0002f) && (dt < 0.02f)) {

		// Measurements are rotated into the rotor frame, so that the residual against
		// the actuator model is formed between two vectors in the same frame.
		const Vector3f acc = _R_align * Vector3f(accel.xyz[0], accel.xyz[1], accel.xyz[2]);

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
			// PWM model (also the per-motor fall-back in ESC mode). The affine model
			// crosses zero at P2/P1 (~918 us here), so it has to be clamped: the speed
			// is squared downstream, and an unclamped low/zero PWM would otherwise
			// model *more* thrust than full throttle.
			float s = math::max(((actuator_outputs.output[i] * _param_rls_speed_p1.get()) - _param_rls_speed_p2.get())
					    * voltage_correction, 0.f);

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
		_identification.updateThrust(acc, output, dt, freeze_identification, apply_lpf);

		const Vector3f p_error_t = _identification.getPredictionErrorThrust();
		// Attitude of the rotor frame rather than of the IMU frame, so the gravity
		// direction the offset RLS regresses against is in the rotor frame too.
		const matrix::Quatf q = matrix::Quatf(v_att.q) * _q_align.inversed();

		//Wrench Estimator Thrust
		_wrench_estimator.updateForce(p_error_t, dt, freeze_identification);
		// --------------------------------------------------------- //

		//RLS Offset
		_identification.updateOffset(q, freeze_identification);

		Vector3f p_error_o = _identification.getPredictionErrorOffset();

		//Wrench Estimator Moment
		_wrench_estimator.updateMoment(p_error_o, _R_align * Vector3f(v_ang_vel.xyz), dt, freeze_identification);

		// Sensor alignment. Fed the raw thrust prediction error rather than the
		// filtered fe, and run after updateOffset() because that is what computes the
		// actuator force vector this regresses against.
		if (_stage == Stage::Align) {
			_alignment.update(p_error_t, _identification.getActuatorForceVector(), q);
		}

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

	status_msg.stage = (uint8_t)_stage;
	const matrix::Vector2f misalignment = _alignment.getMisalignment();
	const matrix::Vector2f align_force = _alignment.getExternalForce();
	status_msg.alignment[0] = misalignment(0);
	status_msg.alignment[1] = misalignment(1);
	status_msg.align_force[0] = align_force(0);
	status_msg.align_force[1] = align_force(1);
	status_msg.align_coverage = _alignment.getCoverageDeg();

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

	status_msg.stage = (uint8_t)_stage;
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


	// No usable actuator data means there is nothing to identify against. Bailing
	// out here marks the sample invalid instead of running the RLS on a stale or
	// empty struct, which silently produces a plausible-looking but meaningless fit.
	if (!selectActuatorOutputs(actuator_outputs)) {
		return false;
	}

	for (int i = 0; i < _num_rotors; i++) {
		if (!PX4_ISFINITE(actuator_outputs.output[i])) {return false;}
	}

	return true;
}

bool RLSWrenchEstimator::actuatorOutputsDriveRotors(const actuator_outputs_s &outputs, int num_rotors)
{
	if ((num_rotors <= 0) || (outputs.noutputs < (uint32_t)num_rotors)) {
		return false;
	}

	bool all_equal = true;

	for (int i = 0; i < num_rotors; i++) {
		// An output bank that is not driving these motors publishes either zeros
		// (unassigned functions) or a constant disarmed value on every channel.
		if (!PX4_ISFINITE(outputs.output[i]) || (outputs.output[i] <= 0.f)) {
			return false;
		}

		if (fabsf(outputs.output[i] - outputs.output[0]) > FLT_EPSILON) {
			all_equal = false;
		}
	}

	// Bit-identical commands across every rotor do not occur while flying, and are
	// the signature of the wrong bank: with equal speeds the tilted-rotor geometry
	// cancels, so the identified force loses its lateral components entirely.
	return !all_equal;
}

bool RLSWrenchEstimator::selectActuatorOutputs(actuator_outputs_s &actuator_outputs)
{
	// actuator_outputs is a multi-instance topic, one instance per output driver,
	// numbered in advertise order. Which one carries the motors therefore depends
	// on the boot order of the output drivers, so it has to be found at run time.
	if (_actuator_outputs_instance >= 0) {
		if (_actuator_outputs_sub.copy(&actuator_outputs)
		    && (hrt_elapsed_time(&actuator_outputs.timestamp) < ACTUATOR_OUTPUTS_TIMEOUT)
		    && (actuator_outputs.noutputs >= (uint32_t)_num_rotors)) {
			// Only freshness is re-checked here: the stricter content test below is a
			// selection criterion, not an invariant (a hovering vehicle may briefly
			// command identical values).
			return true;
		}

		PX4_WARN("actuator_outputs instance %d lost, rescanning", _actuator_outputs_instance);
		_actuator_outputs_instance = -1;
	}

	if (hrt_elapsed_time(&_actuator_outputs_scan_last) < ACTUATOR_OUTPUTS_SCAN_INTERVAL) {
		return false;
	}

	_actuator_outputs_scan_last = hrt_absolute_time();

	for (uint8_t i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
		if (!_actuator_outputs_sub.ChangeInstance(i)) {
			continue; // instance does not exist
		}

		// Probed into the caller's struct rather than a local copy: a rejected candidate
		// is discarded by the caller anyway because this returns false, so a second
		// actuator_outputs_s on the stack would buy nothing.
		if (!_actuator_outputs_sub.copy(&actuator_outputs)
		    || (hrt_elapsed_time(&actuator_outputs.timestamp) >= ACTUATOR_OUTPUTS_TIMEOUT)
		    || !actuatorOutputsDriveRotors(actuator_outputs, _num_rotors)) {
			continue;
		}

		PX4_INFO("using actuator_outputs instance %d (%d outputs)", i, (int)actuator_outputs.noutputs);
		_actuator_outputs_instance = i;
		return true;
	}

	return false;
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

void RLSWrenchEstimator::resetAlignment()
{
	_alignment.reset();
	_alignment_saved.store(false);
}

matrix::Vector2f RLSWrenchEstimator::getAlignmentResult() const
{
	// The estimator sees residuals that already have the current alignment applied,
	// so what it produces is an increment on the parameters, not a replacement.
	const matrix::Vector2f correction = _alignment.getAlignmentCorrection();
	return matrix::Vector2f(_param_rls_align_roll.get() + math::degrees(correction(0)),
				_param_rls_align_pitch.get() + math::degrees(correction(1)));
}

int RLSWrenchEstimator::print_status()
{
	static const char *const stage_names[] = {"identify", "align", "interact"};
	const int stage_idx = math::constrain((int)_stage, 0, 2);
	const int32_t override_val = getStageOverride();

	PX4_INFO("stage: %s [%s, DEBUG_VECT: %s]", stage_names[stage_idx],
		 (override_val < 0) ? "auto" : "forced from console",
		 _interaction_flag_external ? "on" : "off");

	PX4_INFO("alignment applied: roll %.3f deg, pitch %.3f deg",
		 (double)_param_rls_align_roll.get(), (double)_param_rls_align_pitch.get());

	if (_alignment.getSampleCount() > 0) {
		const matrix::Vector2f result = getAlignmentResult();
		const matrix::Vector2f wind = _alignment.getExternalForce();
		PX4_INFO("alignment estimate: roll %.3f deg, pitch %.3f deg (+/- %.3f deg, %d samples)",
			 (double)result(0), (double)result(1),
			 (double)math::degrees(_alignment.getMisalignmentStdDev()), _alignment.getSampleCount());
		PX4_INFO("  world-fixed force separated out: N %.3f E %.3f N", (double)wind(0), (double)wind(1));
		PX4_INFO("  heading coverage %.0f deg -> %s", (double)_alignment.getCoverageDeg(),
			 _alignment_saved.load() ? "already saved"
			 : (_alignment.isValid() ? "usable, saved on 'stage interact'"
			    : (_alignment.hasEnoughRotation() ? "still converging" : "NOT usable, yaw the vehicle further")));
	}

	if (_actuator_outputs_instance < 0) {
		PX4_WARN("actuator_outputs: no instance driving the %d rotors found", _num_rotors);

	} else {
		PX4_INFO("actuator_outputs: instance %d", _actuator_outputs_instance);
	}

	perf_print_counter(_cycle_perf);
	return 0;
}

namespace
{
/**
 * Write the pending alignment correction to RLS_EST_ALN_R/P.
 *
 * The estimate is an increment on the parameters already in use, so it must be
 * applied exactly once - hence the saved flag rather than a plain idempotent write.
 */
int saveAlignment(RLSWrenchEstimator *instance, bool forced)
{
	const AlignmentEstimator &align = instance->getAlignmentEstimator();

	if (instance->isAlignmentSaved()) {
		PX4_WARN("this alignment estimate was already saved; run 'stage align' again for a new one");
		return 0;
	}

	if (!align.isValid() && !forced) {
		PX4_ERR("alignment estimate not usable: %.0f deg of heading covered (need %.0f), +/- %.2f deg",
			(double)align.getCoverageDeg(), (double)AlignmentEstimator::MIN_COVERAGE_DEG,
			(double)math::degrees(align.getMisalignmentStdDev()));
		PX4_ERR("without rotation a misalignment and a real lateral force are the same thing; use -f to override");
		return 1;
	}

	const matrix::Vector2f result = instance->getAlignmentResult();
	float roll = result(0);
	float pitch = result(1);

	if (param_set(param_find("RLS_EST_ALN_R"), &roll) != PX4_OK
	    || param_set(param_find("RLS_EST_ALN_P"), &pitch) != PX4_OK) {
		PX4_ERR("failed to write alignment parameters");
		return 1;
	}

	instance->markAlignmentSaved();
	PX4_INFO("saved RLS_EST_ALN_R %.3f deg, RLS_EST_ALN_P %.3f deg", (double)roll, (double)pitch);
	return 0;
}
}

int RLSWrenchEstimator::custom_command(int argc, char *argv[])
{
	if (argc < 1) {
		return print_usage("missing command");
	}

	const bool is_stage = (strcmp(argv[0], "stage") == 0);
	const bool is_interaction = (strcmp(argv[0], "interaction") == 0);
	const bool is_align = (strcmp(argv[0], "align") == 0);

	if (!is_stage && !is_interaction && !is_align) {
		return print_usage("unknown command");
	}

	RLSWrenchEstimator *instance = get_instance();

	if (!is_running() || (instance == nullptr)) {
		PX4_ERR("module not running");
		return 1;
	}

	if (argc < 2) {
		return instance->print_status();
	}

	if (is_stage || is_interaction) {
		// "interaction on|off|auto" is kept as an alias of the stage machine so the
		// existing DEBUG_VECT-driven workflow and scripts keep working.
		int32_t stage = STAGE_AUTO;

		if ((strcmp(argv[1], "identify") == 0) || (strcmp(argv[1], "off") == 0)) {
			stage = (int32_t)Stage::Identify;

		} else if (strcmp(argv[1], "align") == 0) {
			stage = (int32_t)Stage::Align;

		} else if ((strcmp(argv[1], "interact") == 0) || (strcmp(argv[1], "on") == 0)) {
			stage = (int32_t)Stage::Interact;

		} else if (strcmp(argv[1], "auto") != 0) {
			// Deliberately not print_usage(): that dumps the whole module description,
			// and this is normally typed into a MAVLink shell sharing a link with the
			// vehicle's position aiding.
			PX4_ERR("unknown stage '%s' (expected identify|align|interact|auto)", argv[1]);
			return 1;
		}

		const bool finishing_align = (stage == (int32_t)Stage::Interact) && (instance->getStage() == Stage::Align);

		instance->setStageOverride(stage);

		if (finishing_align) {
			// Let the work queue observe the stage change before the estimate is read,
			// so it cannot still be updating while the console snapshots it.
			px4_usleep(30000);

			if (instance->getAlignmentEstimator().isValid()) {
				saveAlignment(instance, false);

			} else {
				PX4_WARN("alignment run was not usable (%.0f deg of heading covered); nothing saved",
					 (double)instance->getAlignmentEstimator().getCoverageDeg());
			}
		}

		if (stage == STAGE_AUTO) {
			PX4_INFO("stage following MAVLink DEBUG_VECT");

		} else {
			static const char *const stage_names[] = {"identify", "align", "interact"};
			PX4_INFO("stage forced to %s", stage_names[stage]);

			if (stage == (int32_t)Stage::Align) {
				PX4_INFO("yaw the vehicle through at least %.0f deg, then 'align save'",
					 (double)AlignmentEstimator::MIN_COVERAGE_DEG);
			}
		}

		return 0;
	}

	// align save | align reset
	if (strcmp(argv[1], "reset") == 0) {
		instance->requestAlignmentReset();
		PX4_INFO("alignment estimate reset");
		return 0;
	}

	if (strcmp(argv[1], "save") != 0) {
		PX4_ERR("unknown align argument '%s' (expected save|reset)", argv[1]);
		return 1;
	}

	return saveAlignment(instance, (argc >= 3) && (strcmp(argv[2], "-f") == 0));
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

Identification runs in stages, advanced from the console (e.g. over the MAVLink shell).
Each stage freezes what the previous one identified, so that nothing is still adapting
when the external wrench becomes the measurement:

  identify   k_f and the CoM offset adapt. Free flight in still air only - the RLS
             cannot tell a steady external force from a parameter error.
  align      identification frozen, the sensor-to-rotor misalignment is estimated.
             The vehicle must yaw: at a fixed heading a misalignment and a real
             lateral force are the same two degrees of freedom.
  interact   everything frozen, fe/me are the measurement.

$ rls_wrench_estimator stage align
  ... yaw the vehicle through at least 90 deg, ideally a full turn ...
$ rls_wrench_estimator stage interact

Leaving align for interact saves the alignment to RLS_EST_ALN_R/P automatically, if
the run passed the rotation gate; if it did not, nothing is written and it says so.
'align save' does the same by hand. Either way the correction is applied exactly
once - it is an increment on the parameters, not an absolute value. To advance
without keeping a run, 'align reset' first.

With no stage forced, the stage follows the MAVLink DEBUG_VECT flag as before.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rls_wrench_estimator", "estimator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("stage", "Set the identification stage, or print the current state");
	PRINT_MODULE_USAGE_ARG("identify|align|interact|auto", "Stage to force, or follow the DEBUG_VECT flag", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("align", "Manage the sensor alignment estimate (saved automatically on align -> interact)");
	PRINT_MODULE_USAGE_ARG("save|reset", "Write the estimate to RLS_EST_ALN_R/P (-f to skip the rotation check), or discard it", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("interaction", "Alias of 'stage': on -> interact, off -> identify");
	PRINT_MODULE_USAGE_ARG("on|off|auto", "Force interact, force identify, or follow the DEBUG_VECT flag", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int rls_wrench_estimator_main(int argc, char *argv[])
{
	return RLSWrenchEstimator::main(argc, argv);
}
