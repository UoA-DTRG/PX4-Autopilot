/****************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
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
 * Planetary Hex Sun Scale Module.
 */

#pragma once


#include <drivers/drv_hrt.h>
#include <lib/controllib/blocks.hpp>
#include <lib/hysteresis/hysteresis.h>
#include <lib/perf/perf_counter.h>
#include <lib/slew_rate/SlewRateYaw.hpp>
#include <lib/systemlib/mavlink_log.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>


#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/control_sun_scale.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/error_rms.h>
#include <uORB/topics/debug_vect.h>



using namespace time_literals;



class SunScaleController : public ModuleBase<SunScaleController>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	SunScaleController();
	~SunScaleController() override;



	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);


	bool init();


private:

	void updateParams() override;



	// Performance (perf) counter
	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle time")};

	systemlib::Hysteresis _valid_hysteresis{false};


	uORB::Subscription _rc_channels_sub{ORB_ID(rc_channels)};
	uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _local_pos_sp_sub{ORB_ID(vehicle_local_position_setpoint)};
	uORB::Subscription _debug_vect_sub{ORB_ID(debug_vect)};




	uORB::Publication<control_sun_scale_s>	_control_sun_scale_pub{ORB_ID(control_sun_scale)};
	uORB::Publication<error_rms_s>	_error_rms_pub{ORB_ID(error_rms)};



	uORB::SubscriptionCallbackWorkItem _rc_channels1_sub{this,ORB_ID(rc_channels)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};


	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MIXER_SUN_SCALE>) _param_mixer_sun_scale,
		(ParamBool<px4::params::MIXER_SUN_CONST>) _param_mixer_sun_const,
		(ParamInt<px4::params::MIXER_SUN_STICK>) _param_mixer_sun_stick
	);


	perf_counter_t	_loop_perf;			/**< loop duration performance counter */


	struct rc_channels_s _rc_channels{};
	struct vehicle_local_position_setpoint_s local_pos_sp;
	struct vehicle_local_position_s local_pos;


	/** @see ModuleBase::run() */
	void Run() override;

	float sunScale=1;
	bool sunConst=true;
	int sunStick=5;


	int counter=0;
	int end=1;

	matrix::Vector<float, 100> error;
	matrix::Vector<uint, 100> timestamps;
};
