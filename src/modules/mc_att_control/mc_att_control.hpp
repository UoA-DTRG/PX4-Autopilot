/****************************************************************************
 *
 *   Copyright (c) 2013-2025 PX4 Development Team. All rights reserved.
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

#include <matrix/matrix/math.hpp>
#include <perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/autotune_attitude_control_status.h>
#include <uORB/topics/hover_thrust_estimate.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <lib/mathlib/mathlib.h>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <lib/slew_rate/SlewRate.hpp>
#include <lib/stick_yaw/StickYaw.hpp>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/horizontal_thrust_limit.h>

#include <AttitudeControl.hpp>

using namespace time_literals;

class MulticopterAttitudeControl : public ModuleBase<MulticopterAttitudeControl>, public ModuleParams,
	public px4::WorkItem
{
public:
	MulticopterAttitudeControl(bool vtol = false);
	~MulticopterAttitudeControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	/**
	 * initialize some vectors/matrices from parameters
	 */
	void parameters_updated();

	float throttle_curve(float throttle_stick_input);

	/**
	 * Generate & publish an attitude setpoint from stick inputs
	 */
	void generate_attitude_setpoint(const matrix::Quatf &q, float dt);

	AttitudeControl _attitude_control; /**< class for attitude control calculations */
	StickYaw _stick_yaw{this};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _hover_thrust_estimate_sub{ORB_ID(hover_thrust_estimate)};
	uORB::Subscription _vehicle_attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Subscription _autotune_attitude_control_status_sub{ORB_ID(autotune_attitude_control_status)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	//for DTRG horitontal thrust
	uORB::Subscription _rc_channels_sub{ORB_ID(rc_channels)};
	struct rc_channels_s _rc_channels{};
	uORB::Publication<horizontal_thrust_limit_s>	     _horizontal_thrust_limit_pub{ORB_ID(horizontal_thrust_limit)};

	uORB::SubscriptionCallbackWorkItem _vehicle_attitude_sub{this, ORB_ID(vehicle_attitude)};

	uORB::Publication<vehicle_rates_setpoint_s>     _vehicle_rates_setpoint_pub{ORB_ID(vehicle_rates_setpoint)};    /**< rate setpoint publication */
	uORB::Publication<vehicle_attitude_setpoint_s>  _vehicle_attitude_setpoint_pub;

	manual_control_setpoint_s       _manual_control_setpoint {};    /**< manual control setpoint */
	vehicle_control_mode_s          _vehicle_control_mode {};       /**< vehicle control mode */

	perf_counter_t  _loop_perf;             /**< loop duration performance counter */

	matrix::Vector3f _thrust_setpoint_body; /**< body frame 3D thrust vector */

	float _hover_thrust_estimate{NAN};
	SlewRate<float> _hover_thrust_slew_rate{.5f};

	float _yaw_setpoint_stabilized{0.f};
	bool _heading_good_for_control{true}; // initialized true to have heading lock when local position never published
	float _unaided_heading{NAN}; // initialized NAN to not distract heading lock when local position never published
	float _man_tilt_max{0.f};			/**< maximum tilt allowed for manual flight [rad] */
	float _ht_gain{0.f};                   	/**< DTRG horizontal thrust rate */
	int _ht_en{0};                       	/**< DTRG horizontal thrust enable */
	int _ht_rc_en_add{-1};				/**< DTRF HT RC enable channel */
	// 0-based RC channel indices, -1 when the channel parameter is 0 (input disabled)
	int _ht_r_add{-1};                     	/**< DTRG horizontal thrust Roll channel */
	int _ht_p_add{-1}; 	       		/**< DTRG horizontal thrust Pitch channel */
	float _ht_limit = 0.5f; 		/**< DTRG horizontal horizontal thrust limit */
	float _ht_r_limit{math::radians(10.f)};	/**< DTRG horizontal thrust Roll angle limit [rad] */
	float _ht_p_limit{math::radians(10.f)};	/**< DTRG horizontal thrust Pitch angle limit [rad] */
	int _dtrg_ht_mask{0};

	/**
	 * Tilt setpoint from a DTRG-assigned RC channel.
	 *
	 * @param channel_index 0-based RC channel, or -1 when DTRG_HT_R/DTRG_HT_P is 0
	 * @param limit         magnitude limit [rad], from DTRG_HT_R_MAX / DTRG_HT_P_MAX
	 * @return              0 when the input is disabled, out of range or inside the deadzone
	 */
	float dtrgAuxTiltSetpoint(int channel_index, float limit) const;

	/** True when horizontal thrust is enabled and its RC enable switch is held high. */
	bool htSwitchActive() const;


	SlewRate<float> _manual_throttle_minimum{0.f}; ///< 0 when landed and ramped to MPC_MANTHR_MIN in air
	SlewRate<float> _manual_throttle_maximum{0.f}; ///< 0 when disarmed ramped to 1 when spooled up
	AlphaFilter<float> _man_roll_input_filter;
	AlphaFilter<float> _man_pitch_input_filter;

	hrt_abstime _last_run{0};
	hrt_abstime _last_attitude_setpoint{0};

	bool _spooled_up{false}; ///< used to make sure the vehicle cannot take off during the spoolup time
	bool _landed{true};
	bool _vehicle_type_rotary_wing{true};
	bool _vtol{false};
	bool _vtol_tailsitter{false};
	bool _vtol_in_transition_mode{false};

	uint8_t _quat_reset_counter{0};


	//TODO implement param for horiztonal thrust switch
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::MC_AIRMODE>)         _param_mc_airmode,
		(ParamFloat<px4::params::MC_MAN_TILT_TAU>)  _param_mc_man_tilt_tau,

		(ParamFloat<px4::params::MC_ROLL_P>)        _param_mc_roll_p,
		(ParamFloat<px4::params::MC_PITCH_P>)       _param_mc_pitch_p,
		(ParamFloat<px4::params::MC_YAW_P>)         _param_mc_yaw_p,
		(ParamFloat<px4::params::MC_YAW_WEIGHT>)    _param_mc_yaw_weight,

		(ParamFloat<px4::params::MC_ROLLRATE_MAX>)  _param_mc_rollrate_max,
		(ParamFloat<px4::params::MC_PITCHRATE_MAX>) _param_mc_pitchrate_max,
		(ParamFloat<px4::params::MC_YAWRATE_MAX>)   _param_mc_yawrate_max,

		/* Stabilized mode params */
		(ParamFloat<px4::params::MPC_HOLD_DZ>) _param_mpc_hold_dz,
		(ParamFloat<px4::params::MPC_MAN_TILT_MAX>) _param_mpc_man_tilt_max,
		(ParamFloat<px4::params::MPC_MANTHR_MIN>) _param_mpc_manthr_min,
		(ParamFloat<px4::params::MPC_THR_MAX>) _param_mpc_thr_max,
		(ParamFloat<px4::params::MPC_THR_HOVER>) _param_mpc_thr_hover,
		(ParamInt<px4::params::MPC_THR_CURVE>) _param_mpc_thr_curve,
		(ParamFloat<px4::params::MPC_YAW_EXPO>) _param_mpc_yaw_expo,

		(ParamFloat<px4::params::COM_SPOOLUP_TIME>) _param_com_spoolup_time,

		(ParamInt<px4::params::DTRG_HT_EN>)         _param_dtrg_ht_en,		/**< horizontal thrust feature */
		(ParamInt<px4::params::DTRG_HT_RC_EN>)      _param_dtrg_ht_rc_en,	/**< horizontal thrust enable RC channel*/
		(ParamInt<px4::params::DTRG_HT_R>)  	    _param_dtrg_h_t_R,		/**< horizontal thrust Roll channel */
		(ParamInt<px4::params::DTRG_HT_P>)  	    _param_dtrg_h_t_P,		/**< horizontal thrust Pitch channel */
		(ParamFloat<px4::params::DTRG_HT_MAX>)      _param_dtrg_ht_max,		/**< horizontal thrust Limit */
		(ParamFloat<px4::params::DTRG_HT_R_MAX>)    _param_dtrg_ht_r_max,	/**< horizontal thrust roll angle Limit */
		(ParamFloat<px4::params::DTRG_HT_P_MAX>)    _param_dtrg_ht_p_max,	/**< horizontal thrust pitch angle Limit */
		(ParamInt<px4::params::DTRG_HT_MASK>)       _param_dtrg_ht_mask		/**< HT gmask for pitching and rolling using HT thrust*/

	)
};
