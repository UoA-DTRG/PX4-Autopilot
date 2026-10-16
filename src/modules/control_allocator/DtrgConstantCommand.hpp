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

/**
 * @file DtrgConstantCommand.hpp
 *
 * DTRG constant command allocation (planetaryHex): with the switch RC_MAP_CC_MODE on, the
 * rotors in DTRG_CC_MOTORS are held at DTRG_CC_M_CMD and the other rotors are allocated
 * the rest of the demand.
 *
 * The normal allocation is left untouched. A second allocation of the same method
 * (the "outer allocation") is built from the normal effectiveness with the centre rotor
 * columns removed (the motors in DTRG_CC_MOTORS), and is normalised with the normal allocation's scale, so a control
 * setpoint means the same in both. With the centre rotors at u_cc the outer allocation
 * is given
 *
 *   c_outer = c - scale .* (B_cc * u_cc)
 *
 * and the published motor commands are its result with u_cc in place of the held rotors.
 * Thrust and torque setpoints therefore keep their meaning, and the controllers, the
 * flight mode, the hover thrust and control_allocator_status are unaffected.
 *
 * At least MIN_FREE_MOTORS motors must stay allocated, so at most (number of motors - 6)
 * may be held; with a selection outside that the mode is unavailable. The selection is
 * latched while the rotors are held or ramping, and applies once released.
 *
 * Switching ramps the centre rotors between their normally allocated commands and the
 * held ones over DTRG_CC_RAMP, with the outer rotors compensating throughout.
 */

#pragma once

#include <ControlAllocation.hpp>

#include <drivers/drv_hrt.h>
#include <px4_platform_common/module_params.h>
#include <uORB/uORB.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/dtrg_constant_command.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/vehicle_land_detected.h>

class DtrgConstantCommand : public ModuleParams
{
public:
	/// motors that must stay allocated: at most (number of motors - MIN_FREE_MOTORS) can be held
	static constexpr int MIN_FREE_MOTORS = 6;

	/// most motors that can be held (DTRG_CC_MOTORS covers 12 motors)
	static constexpr int MAX_COMMAND_MOTORS = 12 - MIN_FREE_MOTORS;

	explicit DtrgConstantCommand(ModuleParams *parent);
	~DtrgConstantCommand() override;

	/**
	 * Allocation method of matrix 0. The outer allocation uses the same method, and is
	 * recreated when it changes.
	 */
	void setAllocationMethod(AllocationMethod method);

	/**
	 * The effectiveness, actuator limits or normalisation of matrix 0 may have changed.
	 * The outer allocation is rebuilt before its next use.
	 */
	void effectivenessUpdated() { _outer_update_needed = true; }

	/**
	 * Allocate the control setpoint of @p allocation (matrix 0, setpoint already set).
	 * Calls allocation.allocate() as normal unless the mode is engaged or ramping.
	 *
	 * @param allocation  allocation of matrix 0
	 * @param available   the configuration supports the mode (see dtrg_constant_command_s::available);
	 *                    when false the normal allocation is used at once, without ramping
	 * @param num_motors  number of configured motors, which limits DTRG_CC_MOTORS
	 * @param armed       vehicle armed; disarming resets the mode at once
	 * @param dt          time since the last allocation [s]
	 */
	void allocate(ControlAllocation &allocation, bool available, int num_motors, bool armed, float dt);

	/// @return whether the centre rotors are currently held or ramping
	bool active() const { return _blend > 0.f; }

	void printStatus() const;

private:
	void updateParams() override
	{
		ModuleParams::updateParams();
		parametersUpdated();
	}

	void parametersUpdated();

	/// @return whether the switch asks for the mode: enabled, switch high, RC present, flying
	bool switchRequested();

	/// take DTRG_CC_MOTORS as the held motors (only while released)
	void latchMotors();

	/// @return whether the latched motors are a valid selection for @p num_motors
	bool motorsValid(int num_motors) const;

	/// create the outer allocation for _allocation_method, if enabled and not created yet
	void createOuterAllocation();

	/// rebuild the outer allocation from the normal one (effectiveness, limits, scale)
	void updateOuterAllocation(ControlAllocation &allocation);

	void publishStatus(bool requested, bool available, const float cc_cmd[MAX_COMMAND_MOTORS]);

	/// tell the pilot when the mode engages and releases, or cannot engage
	void reportTransitions(bool requested, bool available);

	ControlAllocation *_outer_allocation{nullptr};
	AllocationMethod _allocation_method{AllocationMethod::NONE};
	bool _outer_update_needed{true};

	float _blend{0.f}; ///< 0: normal allocation, 1: centre rotors held

	uint32_t _motors_mask{0}; ///< latched DTRG_CC_MOTORS
	int _num_cc_motors{0}; ///< bits set in _motors_mask
	int _cc_motors[MAX_COMMAND_MOTORS] {}; ///< motor indices of the held rotors, first min(_num_cc_motors, MAX_COMMAND_MOTORS)
	bool _motors_valid{false};

	int _switch_index{-1};
	rc_channels_s _rc_channels{};
	bool _landed{true};

	bool _reported_requested{false};
	bool _reported_engaged{false};
	hrt_abstime _last_status_pub{0};
	orb_advert_t _mavlink_log_pub{nullptr};
	dtrg_constant_command_s _status{};

	uORB::Subscription _rc_channels_sub{ORB_ID(rc_channels)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Publication<dtrg_constant_command_s> _dtrg_constant_command_pub{ORB_ID(dtrg_constant_command)};

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::DTRG_CC_EN>) _param_dtrg_cc_en,
		(ParamInt<px4::params::RC_MAP_CC_MODE>) _param_rc_map_cc_mode,
		(ParamFloat<px4::params::DTRG_CC_M_CMD>) _param_dtrg_cc_m_cmd,
		(ParamFloat<px4::params::DTRG_CC_RAMP>) _param_dtrg_cc_ramp,
		(ParamInt<px4::params::DTRG_CC_MOTORS>) _param_dtrg_cc_motors
	)
};
