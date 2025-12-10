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
 * @file multisine_excitation.hpp
 *
 * Multisine Excitation Module for System Identification
 *
 * This module generates orthogonal multisine excitation signals for RLS
 * (Recursive Least Squares) system identification. It publishes the
 * excitation signals via uORB for the ControlAllocator to consume.
 *
 * @author Jaap Peetsma <jaap.peetsma@gmail.com>
 */

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <lib/system_identification/multisine_excitation.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/multisine_excitation_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;

class MultisineExcitationModule : public ModuleBase<MultisineExcitationModule>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	MultisineExcitationModule();
	~MultisineExcitationModule() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:
	/**
	 * Main Run loop
	 */
	void Run() override;

	/**
	 * Check for parameter changes and update them if needed
	 */
	void parameters_updated();

	/**
	 * Configure the multisine generator with current parameters
	 */
	bool configureGenerator();

	/**
	 * Publish the current excitation status
	 */
	void publishStatus(const float excitation[multisine::MAX_MOTORS]);

	/**
	 * Get the RC aux channel value based on the configured channel
	 * @return RC aux value in range [-1, 1]
	 */
	float getRcAuxValue() const;

	/**
	 * Check if the RC trigger switch is activated
	 * @return true if switch is in active position (> 0.5)
	 */
	bool isRcTriggered() const;

	// Subscriptions
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};

	// Publications
	uORB::Publication<multisine_excitation_status_s> _status_pub{ORB_ID(multisine_excitation_status)};

	// Performance counters
	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	// Multisine generator
	multisine::MultisineExcitation _generator;

	// State
	bool _armed{false};
	bool _rc_triggered_prev{false};
	bool _manual_trigger{false};  // For debug command triggering
	hrt_abstime _last_run{0};
	manual_control_setpoint_s _manual_control_setpoint{};

	// Excitation phase enum
	enum class Phase : uint8_t {
		IDLE = 0,
		PRE_SETTLE = 1,
		EXCITING = 2,
		POST_SETTLE = 3
	};

	Phase _phase{Phase::IDLE};
	float _settle_time{0.0f};
	static constexpr float SETTLE_DURATION_S = 0.5f;  // Time to settle before/after excitation

	// Parameters
	DEFINE_PARAMETERS(
		(ParamBool<px4::params::DTRG_MSINE_EN>) _param_enable,
		(ParamFloat<px4::params::DTRG_MSINE_AMP>) _param_amplitude,
		(ParamFloat<px4::params::DTRG_MSINE_T>) _param_period,
		(ParamFloat<px4::params::DTRG_MSINE_FMIN>) _param_freq_min,
		(ParamFloat<px4::params::DTRG_MSINE_FMAX>) _param_freq_max,
		(ParamInt<px4::params::DTRG_MSINE_AUX>) _param_aux_channel,
		(ParamInt<px4::params::DTRG_MSINE_NMOT>) _param_num_motors,
		(ParamBool<px4::params::DTRG_MSINE_SEQ>) _param_sequential
	)
};
