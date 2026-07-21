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

	// Only act while our nav_state is active. Reset test clock on exit so re-entry starts fresh.
	if (vehicle_status.nav_state != vehicle_status_s::NAVIGATION_STATE_BENCH_TEST) {
		_test_start_time = 0;
		perf_end(_loop_perf);
		return;
	}

	const bool armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
	const bool arm_gate_ok = (_param_bt_arm_enable.get() == 1);

	if (!armed || !arm_gate_ok) {
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

	if (_test_start_time == 0) {
		_test_start_time = now;
	}

	const float dt_since_start = static_cast<float>(now - _test_start_time) * 1e-6f;
	const float axis_output = computeAxisOutput(dt_since_start);

	// Hover baseline: only Z body thrust (NED: -Z is up). Horizontal thrust is 0 at hover.
	float thrust_x = 0.f;
	float thrust_y = 0.f;
	float thrust_z = -_param_bt_hover_thr.get();
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

float BenchTest::computeAxisOutput(float dt_since_start) const
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
			const float ramped = sign * _param_bt_ramp_rate.get() * dt_since_start;
			return math::constrain(ramped, -max_val, max_val);
		}
	}

	return 0.f;
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
	PX4_INFO("mode: %d, axis: %d, sign: %d, arm_enable: %d",
		 (int)_param_bt_mode.get(),
		 (int)_param_bt_axis.get(),
		 (int)_param_bt_sign.get(),
		 (int)_param_bt_arm_enable.get());
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
