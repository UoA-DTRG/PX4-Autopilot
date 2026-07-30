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
 * @file AdmittanceControlModule.cpp
 * @brief Admittance controller.
 *
 * @author Pedro Mendes <pmen817@aucklanduni.ac.nz>
 */

#include "AdmittanceControlModule.hpp"

#include <string.h>

using namespace matrix;

AdmittanceControlModule::AdmittanceControlModule() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	updateParams();
}

AdmittanceControlModule::~AdmittanceControlModule()
{
	perf_free(_cycle_perf);
}

bool AdmittanceControlModule::init()
{
	// Run on every rls_wrench_estimator publication, limited to 100 Hz.
	if (!_rls_wrench_estimator_sub.registerCallback()) {
		PX4_ERR("rls_wrench_estimator callback registration failed");
		return false;
	}

	_rls_wrench_estimator_sub.set_interval_us(10_ms);

	// The estimator may never publish (it is gated on RLS_EST_EN). Schedule a
	// watchdog as well so status keeps flowing and `stop` can terminate.
	ScheduleDelayed(kWatchdogInterval);

	return true;
}

void AdmittanceControlModule::updateParams()
{
	ModuleParams::updateParams();

	BellParameters params_bell{};

	params_bell.A[0] = _param_adm_ctr_ax.get();
	params_bell.A[1] = _param_adm_ctr_ay.get();
	params_bell.A[2] = _param_adm_ctr_az.get();
	params_bell.A[3] = _param_adm_ctr_ayaw.get();

	params_bell.B1[0] = _param_adm_ctr_b1x.get();
	params_bell.B1[1] = _param_adm_ctr_b1y.get();
	params_bell.B1[2] = _param_adm_ctr_b1z.get();
	params_bell.B1[3] = _param_adm_ctr_b1yaw.get();

	params_bell.B2[0] = _param_adm_ctr_b2x.get();
	params_bell.B2[1] = _param_adm_ctr_b2y.get();
	params_bell.B2[2] = _param_adm_ctr_b2z.get();
	params_bell.B2[3] = _param_adm_ctr_b2yaw.get();

	params_bell.B3[0] = _param_adm_ctr_b3x.get();
	params_bell.B3[1] = _param_adm_ctr_b3y.get();
	params_bell.B3[2] = _param_adm_ctr_b3z.get();
	params_bell.B3[3] = _param_adm_ctr_b3yaw.get();

	params_bell.M_min[0] = _param_adm_ctr_mminx.get();
	params_bell.M_min[1] = _param_adm_ctr_mminy.get();
	params_bell.M_min[2] = _param_adm_ctr_mminz.get();
	params_bell.M_min[3] = _param_adm_ctr_mminyaw.get();

	params_bell.K_min[0] = _param_adm_ctr_kminx.get();
	params_bell.K_min[1] = _param_adm_ctr_kminy.get();
	params_bell.K_min[2] = _param_adm_ctr_kminz.get();
	params_bell.K_min[3] = _param_adm_ctr_kminyaw.get();

	params_bell.M_max[0] = _param_adm_ctr_mmaxx.get();
	params_bell.M_max[1] = _param_adm_ctr_mmaxy.get();
	params_bell.M_max[2] = _param_adm_ctr_mmaxz.get();
	params_bell.M_max[3] = _param_adm_ctr_mmaxyaw.get();

	params_bell.K_max[0] = _param_adm_ctr_kmaxx.get();
	params_bell.K_max[1] = _param_adm_ctr_kmaxy.get();
	params_bell.K_max[2] = _param_adm_ctr_kmaxz.get();
	params_bell.K_max[3] = _param_adm_ctr_kmaxyaw.get();

	params_bell.lpf_sat_factor = _param_adm_ctr_lpf.get();

	// Only the schedule parameters are replaced. The integrator state is left
	// alone so a parameter write in flight cannot step the setpoint.
	_control.setParams(params_bell);
	_control.setLimits(_param_adm_ctr_dev_max.get(), _param_adm_ctr_devr_max.get());
}

bool AdmittanceControlModule::navStateAllowed(uint8_t nav_state) const
{
	const int32_t mask = _param_adm_ctr_nav.get();

	switch (nav_state) {
	case vehicle_status_s::NAVIGATION_STATE_POSCTL:
	case vehicle_status_s::NAVIGATION_STATE_POSITION_SLOW:
		return (mask & kNavBitPosition) != 0;

	case vehicle_status_s::NAVIGATION_STATE_OFFBOARD:
		return (mask & kNavBitOffboard) != 0;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:
		return (mask & kNavBitAuto) != 0;

	default:
		return false;
	}
}

bool AdmittanceControlModule::rcSwitchAllowsEngage()
{
	const int32_t channel = _param_adm_ctr_rc_ch.get();

	if (channel <= 0) {
		return true; // switch not configured
	}

	rc_channels_s rc_channels;

	if (!_rc_channels_sub.copy(&rc_channels)) {
		return false;
	}

	const int32_t num_channels = static_cast<int32_t>(sizeof(rc_channels.channels) / sizeof(rc_channels.channels[0]));

	if ((channel > num_channels) || (hrt_elapsed_time(&rc_channels.timestamp) > kInputTimeout)) {
		return false;
	}

	const float value = rc_channels.channels[channel - 1];

	return PX4_ISFINITE(value) && (value > 0.25f);
}

bool AdmittanceControlModule::rawSaturation(float &raw_sat)
{
	actuator_motors_s motors;

	if (!_actuator_motors_sub.copy(&motors) || (hrt_elapsed_time(&motors.timestamp) > kInputTimeout)) {
		return false;
	}

	// Normalised motor commands sit in [0, 1] for a multirotor (or [-1, 1] when
	// reversible). Distance from mid-range is the saturation measure: 0 at the
	// middle, 1 at either rail. Disarmed/unassigned outputs are NaN and skipped,
	// which is what removes the rotor-count bookkeeping the old code needed.
	bool any = false;
	float max_sat = 0.f;

	for (int i = 0; i < actuator_motors_s::NUM_CONTROLS; i++) {
		const float c = motors.control[i];

		if (!PX4_ISFINITE(c)) {
			continue;
		}

		any = true;
		max_sat = math::max(max_sat, fabsf(2.f * (c - 0.5f)));
	}

	if (!any) {
		return false;
	}

	raw_sat = math::min(max_sat, 1.f);
	return true;
}

Vector<float, 4> AdmittanceControlModule::conditionWrench(const rls_wrench_estimator_s &wrench, const Quatf &q_att,
		const Quatf &q_yaw) const
{
	const float deadzone[4] = {
		_param_adm_ctr_dzx.get(), _param_adm_ctr_dzy.get(),
		_param_adm_ctr_dzz.get(), _param_adm_ctr_dzw.get()
	};
	const float saturation[4] = {
		_param_adm_ctr_sax.get(), _param_adm_ctr_say.get(),
		_param_adm_ctr_saz.get(), _param_adm_ctr_saw.get()
	};

	// The estimate is in body frame. Take it to NED, then into the yaw-aligned
	// heading frame the per-axis gains are tuned for, so tilt does not bleed the
	// horizontal channels into the vertical one.
	const Vector3f force_ned = q_att.rotateVector(Vector3f(wrench.fe));
	const Vector3f force_heading = q_yaw.rotateVectorInverse(force_ned);

	// Yaw moment about body z is taken directly; for the small tilts this
	// controller operates at, body z and NED down are close enough.
	const float moment_yaw = wrench.me[2];

	const float raw[4] = {force_heading(0), force_heading(1), force_heading(2), moment_yaw};

	Vector<float, 4> We;

	for (int i = 0; i < 4; i++) {
		const float v = (fabsf(raw[i]) > deadzone[i]) ? raw[i] : 0.f;
		We(i) = math::constrain(v, -saturation[i], saturation[i]);
	}

	return We;
}

void AdmittanceControlModule::updateReference(const trajectory_setpoint_s &sp, float dt)
{
	vehicle_local_position_s local_pos{};
	const bool have_local_pos = _vehicle_local_position_sub.copy(&local_pos)
				    && (hrt_elapsed_time(&local_pos.timestamp) < kInputTimeout);

	const float measured[3] = {local_pos.x, local_pos.y, local_pos.z};
	const bool measured_valid[3] = {
		have_local_pos && local_pos.xy_valid && PX4_ISFINITE(local_pos.x),
		have_local_pos && local_pos.xy_valid && PX4_ISFINITE(local_pos.y),
		have_local_pos && local_pos.z_valid && PX4_ISFINITE(local_pos.z)
	};

	const float ref_max = math::max(_param_adm_ctr_ref_max.get(), 0.f);

	for (int i = 0; i < 3; i++) {
		if (PX4_ISFINITE(sp.position[i])) {
			// The setpoint controls position directly, just follow it.
			_ref_position(i) = sp.position[i];
			_ref_integrating[i] = false;
			continue;
		}

		// Velocity-only on this axis. In Position mode this is what the flight task
		// publishes whenever the stick is deflected.
		if (!_ref_integrating[i] || !_ref_valid) {
			// Seed on the transition so there is no step.
			_ref_position(i) = measured_valid[i] ? measured[i] : _ref_position(i);
			_ref_integrating[i] = true;
		}

		const float v = PX4_ISFINITE(sp.velocity[i]) ? sp.velocity[i] : 0.f;
		_ref_position(i) += v * dt;

		// Rubber-band the reference to the measured position. Without this the
		// integrated reference drifts by the tracking error and the vehicle
		// overshoots when the pilot re-centres.
		if (measured_valid[i]) {
			_ref_position(i) = math::constrain(_ref_position(i), measured[i] - ref_max, measured[i] + ref_max);
		}

		if (!PX4_ISFINITE(_ref_position(i))) {
			_ref_position(i) = measured_valid[i] ? measured[i] : 0.f;
		}
	}

	_ref_valid = true;
}

void AdmittanceControlModule::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	// Re-arm the watchdog first so every exit path below keeps the module alive.
	ScheduleDelayed(kWatchdogInterval);

	perf_begin(_cycle_perf);

	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
	}

	const Mode mode = static_cast<Mode>(_param_adm_ctr_en.get());

	// --- inputs -------------------------------------------------------------
	const bool wrench_updated = _rls_wrench_estimator_sub.update(&_wrench);

	float dt = 0.f;

	if (wrench_updated) {
		dt = static_cast<float>(_wrench.timestamp - _timestamp_last) * 1e-6f;
		_timestamp_last = _wrench.timestamp;
	}

	// Target distance arrives over MAVLink DEBUG_VECT. Check the name so an
	// unrelated publisher cannot drive this controller.
	debug_vect_s flags_vect;

	if (_debug_vect_sub.update(&flags_vect)
	    && (strncmp(flags_vect.name, kDebugVectName, strlen(kDebugVectName)) == 0)) {
		_target_dist = PX4_ISFINITE(flags_vect.z) ? flags_vect.z : kDefaultTargetDist;
		_debug_timestamp_last = hrt_absolute_time();
	}

	if (hrt_elapsed_time(&_debug_timestamp_last) > 1_s) {
		_target_dist = kDefaultTargetDist; // external link lost
	}

	actuator_armed_s armed{};
	_actuator_armed_sub.copy(&armed);

	vehicle_status_s vehicle_status{};
	_vehicle_status_sub.copy(&vehicle_status);

	trajectory_setpoint_s sp{};
	const bool have_setpoint = _trajectory_setpoint_sub.copy(&sp)
				   && (hrt_elapsed_time(&sp.timestamp) < kInputTimeout);

	if (!have_setpoint) {
		// A zero-initialised struct would read as "hold the origin", which is a
		// dangerous thing to publish even on a topic nobody is following yet.
		sp = trajectory_setpoint_s{};
		sp.position[0] = sp.position[1] = sp.position[2] = NAN;
		sp.velocity[0] = sp.velocity[1] = sp.velocity[2] = NAN;
		sp.acceleration[0] = sp.acceleration[1] = sp.acceleration[2] = NAN;
		sp.jerk[0] = sp.jerk[1] = sp.jerk[2] = NAN;
		sp.yaw = NAN;
		sp.yawspeed = NAN;
	}

	vehicle_attitude_s att{};
	const bool have_attitude = _vehicle_attitude_sub.copy(&att)
				   && (hrt_elapsed_time(&att.timestamp) < kInputTimeout);

	// Without a valid attitude fall back to identity, so the rotations below stay
	// finite and the status topic keeps logging instead of filling with NaN.
	Quatf q_att(att.q);

	if (!have_attitude || !q_att.isAllFinite() || (q_att.norm() < FLT_EPSILON)) {
		q_att = Quatf();
	}

	q_att.normalize();
	const Quatf q_yaw(Eulerf(0.f, 0.f, Eulerf(q_att).psi()));

	// --- gating -------------------------------------------------------------
	const bool wrench_fresh = wrench_updated || (hrt_elapsed_time(&_wrench.timestamp) < kInputTimeout);
	const bool wrench_finite = Vector3f(_wrench.fe).isAllFinite() && Vector3f(_wrench.me).isAllFinite();
	const bool dt_valid = wrench_updated && (dt > 0.001f) && (dt < 1.f);

	uint8_t bypass_reason = admittance_status_s::BYPASS_NONE;

	if (!armed.armed) {
		bypass_reason = admittance_status_s::BYPASS_DISARMED;

	} else if (!navStateAllowed(vehicle_status.nav_state)) {
		bypass_reason = admittance_status_s::BYPASS_NAV_STATE;

	} else if (!_wrench.valid || !wrench_fresh || !wrench_finite) {
		bypass_reason = admittance_status_s::BYPASS_WRENCH_INVALID;

	} else if (!have_setpoint || !have_attitude) {
		// q_att was forced to identity above if it was missing or degenerate, so
		// have_attitude is the only check still worth making here.
		bypass_reason = admittance_status_s::BYPASS_INPUT_INVALID;
	}

	const bool safety_ok = (bypass_reason == admittance_status_s::BYPASS_NONE);
	const bool rc_ok = rcSwitchAllowsEngage();

	if (safety_ok) {
		if (mode == Mode::Disabled) {
			bypass_reason = admittance_status_s::BYPASS_DISABLED;

		} else if ((mode == Mode::Active) && !rc_ok) {
			bypass_reason = admittance_status_s::BYPASS_RC_SWITCH;
		}
	}

	const bool engage_ok = safety_ok && (mode == Mode::Active) && rc_ok;

	// In Bypass the deviation is still ramped in, so admittance_setpoint carries
	// the setpoint that *would* have been commanded. Nothing consumes it because
	// `engaged` stays false.
	const bool ramp_target_up = safety_ok && ((mode == Mode::Bypass) || engage_ok);

	// --- integrate ----------------------------------------------------------
	if (!armed.armed) {
		// On the ground there is nothing to comply with, and the reference must not
		// carry stale state into the next flight.
		_control.reset();
		_ref_valid = false;
		_ref_integrating[0] = _ref_integrating[1] = _ref_integrating[2] = false;
	}

	Vector<float, 4> We{};

	if (safety_ok && dt_valid) {
		float raw_sat = 0.f;

		if (rawSaturation(raw_sat)) {
			_control.updateSaturation(dt, raw_sat);
		}

		We = conditionWrench(_wrench, q_att, q_yaw);
		_control.update(dt, We, _target_dist);

		updateReference(sp, dt);

		if (!_control.isFinite()) {
			_control.reset();
			bypass_reason = admittance_status_s::BYPASS_OUTPUT_INVALID;
		}
	}

	// Ramp uses the same dt when integrating, otherwise the watchdog period, so a
	// silent estimator still fades the deviation out rather than dropping it.
	const float ramp_dt = dt_valid ? dt : (static_cast<float>(kWatchdogInterval) * 1e-6f);
	const float ramp_step = ramp_dt / math::max(_param_adm_ctr_ramp.get(), 0.01f);
	_ramp = math::constrain(_ramp + (ramp_target_up ? ramp_step : -ramp_step), 0.f, 1.f);

	// --- output -------------------------------------------------------------
	const Vector3f deviation_ned = q_yaw.rotateVector(_control.getDeviation());
	const Vector3f deviation_rate_ned = q_yaw.rotateVector(_control.getDeviationRate());
	const Vector3f deviation_accel_ned = q_yaw.rotateVector(_control.getDeviationAccel());

	trajectory_setpoint_s out = sp;
	out.timestamp = hrt_absolute_time();

	bool output_finite = true;

	if (have_setpoint && _ref_valid) {
		for (int i = 0; i < 3; i++) {
			out.position[i] = _ref_position(i) + (_ramp * deviation_ned(i));

			const float v = PX4_ISFINITE(sp.velocity[i]) ? sp.velocity[i] : 0.f;
			out.velocity[i] = v + (_ramp * deviation_rate_ned(i));

			const float a = PX4_ISFINITE(sp.acceleration[i]) ? sp.acceleration[i] : 0.f;
			out.acceleration[i] = a + (_ramp * deviation_accel_ned(i));

			output_finite = output_finite && PX4_ISFINITE(out.position[i]) && PX4_ISFINITE(out.velocity[i])
					&& PX4_ISFINITE(out.acceleration[i]);
		}

		if (PX4_ISFINITE(sp.yaw)) {
			out.yaw = wrap_pi(sp.yaw + (_ramp * _control.getYawDeviation()));
			output_finite = output_finite && PX4_ISFINITE(out.yaw);
		}

		if (PX4_ISFINITE(sp.yawspeed)) {
			out.yawspeed = sp.yawspeed + (_ramp * _control.getYawDeviationRate());
			output_finite = output_finite && PX4_ISFINITE(out.yawspeed);
		}

	} else {
		output_finite = false;
	}

	if (!output_finite) {
		// Fall back to a clean passthrough so a consumer that is mid-transition
		// never sees a partially written setpoint.
		out = sp;
		out.timestamp = hrt_absolute_time();

		if (bypass_reason == admittance_status_s::BYPASS_NONE) {
			bypass_reason = admittance_status_s::BYPASS_OUTPUT_INVALID;
		}
	}

	_engaged = engage_ok && output_finite && (_ramp > FLT_EPSILON);

	if (_engaged) {
		bypass_reason = admittance_status_s::BYPASS_NONE;
	}

	_bypass_reason = bypass_reason;

	_admittance_setpoint_pub.publish(out);

	// --- status -------------------------------------------------------------
	const AdmittanceParameters params = _control.getAdmittanceParameters();

	admittance_status_s status{};
	status.engaged = _engaged;
	status.valid = safety_ok && dt_valid;
	status.bypass_reason = _bypass_reason;
	status.ramp = _ramp;

	for (int i = 0; i < 4; i++) {
		status.wrench_used[i] = We(i);
		status.mass[i] = params.M[i];
		status.damping[i] = params.C[i];
		status.stiffness[i] = params.K[i];
	}

	deviation_ned.copyTo(status.deviation);
	deviation_rate_ned.copyTo(status.deviation_rate);
	status.yaw_deviation = _control.getYawDeviation();
	_ref_position.copyTo(status.reference_position);
	status.reference_integrating = _ref_integrating[0] || _ref_integrating[1] || _ref_integrating[2];
	status.sat_factor = _control.getSaturationFactor();
	status.target_dist = _target_dist;
	status.timestamp = hrt_absolute_time();

	_admittance_status_pub.publish(status);

	perf_end(_cycle_perf);
}

int AdmittanceControlModule::task_spawn(int argc, char *argv[])
{
	AdmittanceControlModule *instance = new AdmittanceControlModule();

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

int AdmittanceControlModule::print_status()
{
	static const char *bypass_str[] = {
		"none", "disabled", "rc switch", "nav state", "disarmed",
		"wrench invalid", "input invalid", "output invalid"
	};

	const int32_t mode = _param_adm_ctr_en.get();
	const AdmittanceParameters params = _control.getAdmittanceParameters();

	PX4_INFO("mode: %" PRId32 " (%s)", mode,
		 (mode == 0) ? "disabled" : ((mode == 1) ? "bypass, logging only" : "active"));
	PX4_INFO("engaged: %s, ramp: %.2f, reason: %s", _engaged ? "yes" : "no", (double)_ramp,
		 (_bypass_reason < (sizeof(bypass_str) / sizeof(bypass_str[0]))) ? bypass_str[_bypass_reason] : "?");
	PX4_INFO("saturation factor: %.3f, target distance: %.2f m",
		 (double)_control.getSaturationFactor(), (double)_target_dist);

	const Vector3f deviation = _control.getDeviation();
	PX4_INFO("deviation (heading frame): %.3f %.3f %.3f m, yaw %.3f rad",
		 (double)deviation(0), (double)deviation(1), (double)deviation(2), (double)_control.getYawDeviation());
	PX4_INFO("reference: %.2f %.2f %.2f m (%s)",
		 (double)_ref_position(0), (double)_ref_position(1), (double)_ref_position(2),
		 (_ref_integrating[0] || _ref_integrating[1] || _ref_integrating[2]) ? "integrating" : "tracking");
	PX4_INFO("M: %.2f %.2f %.2f %.2f", (double)params.M[0], (double)params.M[1], (double)params.M[2],
		 (double)params.M[3]);
	PX4_INFO("K: %.2f %.2f %.2f %.2f", (double)params.K[0], (double)params.K[1], (double)params.K[2],
		 (double)params.K[3]);

	perf_print_counter(_cycle_perf);
	return 0;
}

int AdmittanceControlModule::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int AdmittanceControlModule::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Admittance controller.

Models the vehicle as a virtual mass-spring-damper driven by the external wrench
estimated by `rls_wrench_estimator`, and offsets the trajectory setpoint by the
resulting deviation so the vehicle complies with contact.

The offset is published on `admittance_setpoint` (a `trajectory_setpoint`) and
`mc_pos_control` follows it whenever `admittance_status.engaged` is set. With
`ADM_CTR_EN` set to 1 the controller runs and logs what it would have commanded
without affecting flight; set it to 2 to apply the output.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("admittance_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int admittance_control_main(int argc, char *argv[])
{
	return AdmittanceControlModule::main(argc, argv);
}
