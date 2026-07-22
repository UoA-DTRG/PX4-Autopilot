/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include "BenchTest.hpp"

#include <mathlib/mathlib.h>

BenchTest::BenchTest() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
}

BenchTest::~BenchTest()
{
	perf_free(_loop_perf);
}

bool BenchTest::init()
{
	ScheduleOnInterval(kLoopIntervalUs);
	return true;
}

void BenchTest::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
	}

	vehicle_status_s vehicle_status{};
	_vehicle_status_sub.copy(&vehicle_status);

	const hrt_abstime now = hrt_absolute_time();

	// Only act while our nav_state is active. Reset test state on exit so re-entry starts fresh.
	if (vehicle_status.nav_state != vehicle_status_s::NAVIGATION_STATE_BENCH_TEST) {
		_test_start_time = 0;
		_output_start_time = 0;
		_ramp_frozen = false;
		_ramp_value = 0.f;
		perf_end(_loop_perf);
		return;
	}

	const bool armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
	const bool arm_gate_ok = (_param_bt_arm_enable.get() == 1);

	if (!armed || !arm_gate_ok) {
		// Outputs inactive: restart the throttle spin-up on the next activation.
		_output_start_time = 0;
		publishZero(now);

		if (now - _last_warn_time > 2_s) {
			_last_warn_time = now;

			if (!arm_gate_ok) {
				PX4_WARN("bench_test: outputs suppressed - set BT_ARM_ENABLE=1 to enable");
			}
		}

		perf_end(_loop_perf);
		return;
	}

	// Gate the step/ramp profile on the start switch. While it is low we hold
	// the hover baseline and keep the clock reset so the next flip runs fresh.
	const bool start_requested = startRequested();

	if (!start_requested) {
		_test_start_time = 0;
		_ramp_frozen = false;
		_ramp_value = 0.f;
	} else if (_test_start_time == 0) {
		_test_start_time = now;
		_ramp_frozen = false;
		_ramp_value = 0.f;
	}

	float axis_output = 0.f;

	if (start_requested) {
		const float dt_since_start = static_cast<float>(now - _test_start_time) * 1e-6f;
		axis_output = computeAxisOutput(dt_since_start, motorSaturated());
	}

	// Soft-start: ramp the hover baseline up from zero over BT_SPINUP_T from the
	// moment the outputs became active, so the motors spin up gradually.
	if (_output_start_time == 0) {
		_output_start_time = now;
	}

	float spinup = 1.f;
	const float spinup_t = _param_bt_spinup_t.get();

	if (spinup_t > 0.f) {
		const float since_output = static_cast<float>(now - _output_start_time) * 1e-6f;
		spinup = math::constrain(since_output / spinup_t, 0.f, 1.f);
	}

	// Hover baseline: only Z body thrust (NED: -Z is up). Horizontal thrust is 0 at hover.
	float thrust_x = 0.f;
	float thrust_y = 0.f;
	float thrust_z = -_param_bt_hover_thr.get() * spinup;
	float torque_x = 0.f;
	float torque_y = 0.f;
	float torque_z = 0.f;

	switch (static_cast<Axis>(_param_bt_axis.get())) {
	case Axis::ThrustX:
		thrust_x = axis_output;
		break;

	case Axis::ThrustY:
		thrust_y = axis_output;
		break;

	case Axis::ThrustZ:
		thrust_z -= axis_output; // additional thrust in -Z (up)
		break;

	case Axis::Roll:
		torque_x = axis_output;
		break;

	case Axis::Pitch:
		torque_y = axis_output;
		break;

	case Axis::Yaw:
		torque_z = axis_output;
		break;
	}

	publishOutputs(now, thrust_x, thrust_y, thrust_z, torque_x, torque_y, torque_z);

	perf_end(_loop_perf);
}

float BenchTest::computeAxisOutput(float dt_since_start, bool motor_saturated)
{
	const float sign = (_param_bt_sign.get() >= 0) ? 1.f : -1.f;

	switch (static_cast<Mode>(_param_bt_mode.get())) {
	case Mode::Hover:
		return 0.f;

	case Mode::Step: {
			const float delay = _param_bt_step_delay.get();
			const float dur = _param_bt_step_dur.get();

			if (dt_since_start < delay) {
				return 0.f;
			}

			if (dt_since_start < delay + dur) {
				return sign * _param_bt_step_mag.get();
			}

			return 0.f;
		}

	case Mode::Ramp: {
			const float max_val = _param_bt_max_val.get();

			// Keep increasing the ramp until a motor saturates (upper or lower) or
			// the safety clamp is reached, then freeze and hold that value.
			if (!_ramp_frozen) {
				const float ramped = sign * _param_bt_ramp_rate.get() * dt_since_start;
				_ramp_value = math::constrain(ramped, -max_val, max_val);

				if (motor_saturated || fabsf(_ramp_value) >= max_val) {
					_ramp_frozen = true;
				}
			}

			return _ramp_value;
		}
	}

	return 0.f;
}

bool BenchTest::startRequested()
{
	const int32_t sw = _param_bt_start_sw.get();

	// No switch configured: start the profile immediately on mode entry.
	if (sw <= 0) {
		return true;
	}

	input_rc_s input_rc{};

	if (!_input_rc_sub.copy(&input_rc)) {
		return false;
	}

	// Ignore stale / lost RC so the profile stops if the link drops.
	if (input_rc.rc_lost || input_rc.rc_failsafe) {
		return false;
	}

	const int channel_index = sw - 1; // BT_START_SW is 1-based (e.g. 16 -> values[15])

	if (channel_index < 0 || channel_index >= input_rc.channel_count
	    || channel_index >= input_rc_s::RC_INPUT_MAX_CHANNELS) {
		return false;
	}

	return input_rc.values[channel_index] > kSwitchThresholdUs;
}

bool BenchTest::motorSaturated()
{
	actuator_motors_s motors{};

	if (!_actuator_motors_sub.copy(&motors)) {
		return false;
	}

	// Treat a motor as saturated once its normalised output comes within the
	// configured margin of its limit, rather than only at the hard 0 / 1 stops.
	const float margin = math::constrain(_param_bt_sat_margin.get(), 0.f, 0.5f);
	const float upper = 1.f - margin;

	// Only inspect the connected motors so unused slots do not freeze the ramp.
	const int n = math::min(numMotors(), static_cast<int>(actuator_motors_s::NUM_CONTROLS));

	for (int i = 0; i < n; i++) {
		const float c = motors.control[i];

		if (!PX4_ISFINITE(c)) {
			continue;
		}

		// Reversible motors span [-1, 1]; standard motors span [0, 1].
		const bool reversible = (motors.reversible_flags & (1u << i)) != 0;
		const float lower = reversible ? (-1.f + margin) : margin;

		if (c >= upper || c <= lower) {
			return true;
		}
	}

	return false;
}

int BenchTest::numMotors()
{
	const int32_t configured = _param_bt_num_motors.get();

	if (configured > 0) {
		return math::min(configured, static_cast<int32_t>(actuator_motors_s::NUM_CONTROLS));
	}

	// Auto-detect: count the finite motor outputs. Unused motor slots are NaN.
	actuator_motors_s motors{};

	if (!_actuator_motors_sub.copy(&motors)) {
		return 0;
	}

	int count = 0;

	for (int i = 0; i < actuator_motors_s::NUM_CONTROLS; i++) {
		if (PX4_ISFINITE(motors.control[i])) {
			count++;
		}
	}

	return count;
}

void BenchTest::publishZero(const hrt_abstime &now)
{
	publishOutputs(now, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
}

void BenchTest::publishOutputs(const hrt_abstime &now,
			       float thrust_x, float thrust_y, float thrust_z,
			       float torque_x, float torque_y, float torque_z)
{
	vehicle_thrust_setpoint_s thrust_sp{};
	thrust_sp.timestamp_sample = now;
	thrust_sp.xyz[0] = math::constrain(thrust_x, -1.f, 1.f);
	thrust_sp.xyz[1] = math::constrain(thrust_y, -1.f, 1.f);
	thrust_sp.xyz[2] = math::constrain(thrust_z, -1.f, 1.f);
	thrust_sp.timestamp = hrt_absolute_time();
	_vehicle_thrust_setpoint_pub.publish(thrust_sp);

	vehicle_torque_setpoint_s torque_sp{};
	torque_sp.timestamp_sample = now;
	torque_sp.xyz[0] = math::constrain(torque_x, -1.f, 1.f);
	torque_sp.xyz[1] = math::constrain(torque_y, -1.f, 1.f);
	torque_sp.xyz[2] = math::constrain(torque_z, -1.f, 1.f);
	torque_sp.timestamp = hrt_absolute_time();
	_vehicle_torque_setpoint_pub.publish(torque_sp);
}

int BenchTest::task_spawn(int argc, char *argv[])
{
	BenchTest *instance = new BenchTest();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	if (!instance->init()) {
		delete instance;
		_object.store(nullptr);
		_task_id = -1;
		return PX4_ERROR;
	}

	return PX4_OK;
}

int BenchTest::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int BenchTest::print_status()
{
	PX4_INFO("Running");
	PX4_INFO("mode: %d, axis: %d, sign: %d, arm_enable: %d, start_sw: %d",
		 (int)_param_bt_mode.get(),
		 (int)_param_bt_axis.get(),
		 (int)_param_bt_sign.get(),
		 (int)_param_bt_arm_enable.get(),
		 (int)_param_bt_start_sw.get());
	PX4_INFO("ramp_frozen: %d, ramp_value: %.3f",
		 (int)_ramp_frozen,
		 (double)_ramp_value);
	perf_print_counter(_loop_perf);
	return 0;
}

int BenchTest::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Bench test flight mode.

Publishes vehicle_thrust_setpoint and vehicle_torque_setpoint directly for
system identification on a rigidly mounted vehicle. Sub-mode (hover / step /
ramp), axis, and magnitudes are controlled via BT_* parameters. Only outputs
non-zero commands when the vehicle is armed AND BT_ARM_ENABLE is 1.

The step and ramp profiles are started by a raw RC channel (input_rc)
selected with BT_START_SW, e.g. 16 for CH16 (0 disables the gate and starts
on mode entry). While the switch is low only the hover baseline is commanded.
The ramp increases until a motor saturates (upper or lower) or BT_MAX_VAL is
reached, then holds.

When the outputs first become active the hover baseline is ramped up from zero
over BT_SPINUP_T (throttle spin-up). Disarming is permitted in this mode even
though the vehicle reports as airborne on the bench.

Active only while vehicle_status.nav_state == NAVIGATION_STATE_BENCH_TEST.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("bench_test", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int bench_test_main(int argc, char *argv[])
{
	return BenchTest::main(argc, argv);
}
