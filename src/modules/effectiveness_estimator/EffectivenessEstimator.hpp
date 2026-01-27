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
 * @file EffectivenessEstimator.hpp
 * @brief RLS-based effectiveness matrix and mixer estimator for multicopters
 *
 * This module uses Recursive Least Squares (RLS) to estimate the actuator
 * effectiveness matrix and derive the mixer matrix for multicopter vehicles.
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <lib/matrix/matrix/math.hpp>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/effectiveness_estimate.h>
#include <uORB/topics/mixer_estimate.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;

class EffectivenessEstimator : public ModuleBase<EffectivenessEstimator>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	EffectivenessEstimator();
	~EffectivenessEstimator() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:
	void Run() override;
	void updateParams() override;

	void reset();
	void updateEstimation();
	void publishEstimates(const hrt_abstime &timestamp);
	bool computeMixer();

	static constexpr uint8_t MAX_ROTORS = 16;
	static constexpr uint8_t DOF = 6; // 3 forces + 3 moments

	// RLS estimation state
	matrix::Matrix<float, MAX_ROTORS * DOF, MAX_ROTORS * DOF> _P; // Covariance matrix
	matrix::Vector<float, MAX_ROTORS * DOF> _theta;                // Parameter vector (effectiveness)
	float _lambda{0.99f};                                           // Forgetting factor
	matrix::Vector3f _innovations;                                  // Innovation per moment axis (Mx, My, Mz)
	uint32_t _sample_count{0};                                      // Number of samples processed
	static constexpr uint32_t MIN_SAMPLES_FOR_CONVERGENCE = 100;   // Minimum samples before considering convergence
	static constexpr float RLS_NUMERICAL_EPSILON = 1e-6f;          // Numerical stability threshold

	// Temporary matrices/vectors for RLS update (avoids stack overflow)
	// These are reused across iterations to minimize stack usage
	matrix::Vector<float, MAX_ROTORS> _phi_temp;
	matrix::Vector3f _measured_moments_temp;
	matrix::Vector3f _inertia_temp;

	// Effectiveness and mixer matrices
	matrix::Matrix<float, DOF, MAX_ROTORS> _effectiveness;
	matrix::Matrix<float, MAX_ROTORS, DOF> _mixer;
	bool _mixer_valid{false};

	// Vehicle state
	matrix::Vector3f _angular_velocity;
	matrix::Vector3f _angular_velocity_prev;
	matrix::Vector3f _angular_acceleration;
	matrix::Quatf _attitude;
	float _actuator_outputs[MAX_ROTORS] {};
	uint8_t _num_rotors{4};
	hrt_abstime _angular_velocity_timestamp_prev{0};

	bool _armed{false};
	bool _estimation_valid{false};

	// Subscriptions
	uORB::Subscription _actuator_outputs_sub{ORB_ID(actuator_outputs)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

	// Publications
	uORB::Publication<effectiveness_estimate_s> _effectiveness_estimate_pub{ORB_ID(effectiveness_estimate)};
	uORB::Publication<mixer_estimate_s> _mixer_estimate_pub{ORB_ID(mixer_estimate)};

	hrt_abstime _timestamp_last{0};

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, "effectiveness_estimator: cycle time")};

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::EFF_EST_ACT>) _param_eff_est_enable,
		(ParamFloat<px4::params::EFF_EST_LAMBDA>) _param_eff_est_lambda,
		(ParamFloat<px4::params::EFF_EST_P_INIT>) _param_eff_est_p_init,
		(ParamInt<px4::params::EFF_EST_N_ROTORS>) _param_eff_est_num_rotors,
		(ParamFloat<px4::params::EFF_EST_MASS>) _param_eff_est_mass,
		(ParamFloat<px4::params::EFF_EST_IXX>) _param_eff_est_ixx,
		(ParamFloat<px4::params::EFF_EST_IYY>) _param_eff_est_iyy,
		(ParamFloat<px4::params::EFF_EST_IZZ>) _param_eff_est_izz,
		(ParamFloat<px4::params::EFF_EST_UPD_RATE>) _param_eff_est_update_rate,
		(ParamFloat<px4::params::EFF_EST_CONV_VAR>) _param_eff_est_conv_var,
		(ParamFloat<px4::params::EFF_EST_C_INNOV>) _param_eff_est_conv_innov,
		(ParamFloat<px4::params::EFF_EST_MIN_EXC>) _param_eff_est_min_excite
	)
};
