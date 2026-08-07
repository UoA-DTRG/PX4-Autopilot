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

#pragma once

#include "bench_test_switch.h"

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>

using namespace time_literals;

class BenchTest : public ModuleBase<BenchTest>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	BenchTest();
	~BenchTest() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:
	enum class Mode : int32_t {
		Hover = 0,
		Step  = 1,
		Ramp  = 2,
	};

	enum class Axis : int32_t {
		ThrustX = 0,
		ThrustY = 1,
		ThrustZ = 2,
		Roll    = 3,
		Pitch   = 4,
		Yaw     = 5,
	};

	static constexpr uint32_t kLoopIntervalUs = 2500; // 400 Hz

	void Run() override;

	void publishZero(const hrt_abstime &now);
	void publishOutputs(const hrt_abstime &now,
			    float thrust_x, float thrust_y, float thrust_z,
			    float torque_x, float torque_y, float torque_z);
	float computeAxisOutput(float sign, float dt_since_start, bool motor_saturated);

	// Direction of the excitation, taken from the 3-position RC switch selected by
	// BT_SIGN_SW: +1 (switch up), -1 (switch down), 0 (centre / no excitation).
	// Off-centre is what starts the profile; there is no separate start switch.
	float signFromSwitch();

	// The parameters that define the excitation profile. A change to any of them
	// restarts the profile so the new settings run from t = 0.
	struct ProfileParams {
		int32_t mode;
		int32_t axis;
		float step_mag;
		float step_delay;
		float step_dur;
		float ramp_rate;
		float max_val;
	};

	ProfileParams currentProfileParams();

	// Compares the profile parameters against the stored snapshot and updates it.
	bool profileParamsChanged();

	// True when any connected motor has reached its upper or lower saturation limit.
	bool motorSaturated();

	// Number of motors to consider for saturation. Uses BT_NUM_MOTORS if set,
	// otherwise auto-detects from the actuator_motors output.
	int numMotors();

	uORB::Publication<vehicle_thrust_setpoint_s> _vehicle_thrust_setpoint_pub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Publication<vehicle_torque_setpoint_s> _vehicle_torque_setpoint_pub{ORB_ID(vehicle_torque_setpoint)};

	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _input_rc_sub{ORB_ID(input_rc)};
	uORB::Subscription _actuator_motors_sub{ORB_ID(actuator_motors)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	hrt_abstime _test_start_time{0};
	hrt_abstime _output_start_time{0};
	hrt_abstime _last_warn_time{0};

	// Ramp freeze state: once a motor saturates (or BT_MAX_VAL is hit) the
	// ramp holds its value until the profile is restarted.
	bool  _ramp_frozen{false};
	float _ramp_value{0.f};

	// Sign the currently running profile was started with. A change (including
	// a return to centre) restarts the profile.
	float _active_sign{0.f};

	ProfileParams _profile_params{};

	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::BT_MODE>)        _param_bt_mode,
		(ParamInt<px4::params::BT_AXIS>)        _param_bt_axis,
		(ParamInt<px4::params::BT_SIGN_SW>)     _param_BT_SIGN_SW,
		(ParamFloat<px4::params::BT_HOVER_THR>) _param_bt_hover_thr,
		(ParamFloat<px4::params::BT_STEP_MAG>)  _param_bt_step_mag,
		(ParamFloat<px4::params::BT_STEP_DELAY>)_param_bt_step_delay,
		(ParamFloat<px4::params::BT_STEP_DUR>)  _param_bt_step_dur,
		(ParamFloat<px4::params::BT_RAMP_RATE>) _param_bt_ramp_rate,
		(ParamFloat<px4::params::BT_MAX_VAL>)   _param_bt_max_val,
		(ParamInt<px4::params::BT_ARM_ENABLE>)  _param_bt_arm_enable,
		(ParamInt<px4::params::BT_NUM_MOTORS>)  _param_bt_num_motors,
		(ParamFloat<px4::params::BT_SAT_MARGIN>) _param_bt_sat_margin,
		(ParamFloat<px4::params::BT_SPINUP_T>)  _param_bt_spinup_t
	)
};
