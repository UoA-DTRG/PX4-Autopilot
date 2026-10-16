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
 * @file DtrgConstantCommand.cpp
 */

#include "DtrgConstantCommand.hpp"

#include <ControlAllocationPseudoInverse.hpp>
#include <ControlAllocationSequentialDesaturation.hpp>

#include <drivers/drv_hrt.h>
#include <lib/dtrg_horizontal_thrust/dtrg_horizontal_thrust.hpp>
#include <mathlib/math/Limits.hpp>
#include <systemlib/mavlink_log.h>

using namespace matrix;
using namespace time_literals;

/// the switch reads off when rc_channels has not been published for this long
static constexpr hrt_abstime kRcTimeout = 500_ms;

DtrgConstantCommand::DtrgConstantCommand(ModuleParams *parent) :
	ModuleParams(parent)
{
	_dtrg_constant_command_pub.advertise();
	parametersUpdated();
	latchMotors();
}

DtrgConstantCommand::~DtrgConstantCommand()
{
	delete _outer_allocation;
}

void DtrgConstantCommand::parametersUpdated()
{
	// RC_MAP_CC_MODE of 0 means unassigned, which maps to index -1
	_switch_index = dtrg_ht::channelIndex(_param_rc_map_cc_mode.get());

	if (_outer_allocation != nullptr) {
		_outer_allocation->updateParameters();

	} else {
		createOuterAllocation();
	}

	_outer_update_needed = true;
}

void DtrgConstantCommand::setAllocationMethod(AllocationMethod method)
{
	if (method != _allocation_method) {
		_allocation_method = method;
		delete _outer_allocation;
		_outer_allocation = nullptr;
	}

	createOuterAllocation();
}

void DtrgConstantCommand::createOuterAllocation()
{
	// Only once enabled, so that other vehicles do not carry a second allocation. It is
	// kept when disabled again, so that disabling in flight ramps out like the switch.
	if ((_outer_allocation != nullptr) || !_param_dtrg_cc_en.get()) {
		return;
	}

	switch (_allocation_method) {
	case AllocationMethod::PSEUDO_INVERSE:
		_outer_allocation = new ControlAllocationPseudoInverse();
		break;

	case AllocationMethod::SEQUENTIAL_DESATURATION:
		_outer_allocation = new ControlAllocationSequentialDesaturation();
		break;

	default:
		return;
	}

	if (_outer_allocation == nullptr) {
		PX4_ERR("constant command: alloc failed");
		return;
	}

	_outer_allocation->updateParameters();
	_outer_update_needed = true;
}

void DtrgConstantCommand::latchMotors()
{
	const uint32_t mask = static_cast<uint32_t>(_param_dtrg_cc_motors.get());

	if (mask == _motors_mask) {
		return;
	}

	_motors_mask = mask;
	_num_cc_motors = 0;

	for (int motor = 0; motor < 32; ++motor) {
		if (mask & (1u << motor)) {
			if (_num_cc_motors < MAX_COMMAND_MOTORS) {
				_cc_motors[_num_cc_motors] = motor;
			}

			++_num_cc_motors;
		}
	}

	_outer_update_needed = true;
}

bool DtrgConstantCommand::motorsValid(int num_motors) const
{
	// at least one motor, all of them configured, and MIN_FREE_MOTORS left to the allocation
	return (_num_cc_motors > 0) && (_num_cc_motors <= MAX_COMMAND_MOTORS)
	       && (num_motors >= 0) && (num_motors < 32) && ((_motors_mask >> num_motors) == 0)
	       && (_num_cc_motors <= num_motors - MIN_FREE_MOTORS);
}

bool DtrgConstantCommand::switchRequested()
{
	_rc_channels_sub.update(&_rc_channels);

	vehicle_land_detected_s vehicle_land_detected;

	if (_vehicle_land_detected_sub.update(&vehicle_land_detected)) {
		_landed = vehicle_land_detected.landed;
	}

	const bool rc_present = (_rc_channels.timestamp != 0) && !_rc_channels.signal_lost
				&& (hrt_elapsed_time(&_rc_channels.timestamp) < kRcTimeout);

	return rc_present && !_landed
	       && dtrg_ht::switchActive(_rc_channels, _param_dtrg_cc_en.get(), _switch_index);
}

void DtrgConstantCommand::updateOuterAllocation(ControlAllocation &allocation)
{
	// Brings the normal allocation's normalisation scale up to date, which it otherwise
	// only does on its next allocate()
	Matrix<float, ControlAllocation::NUM_ACTUATORS, ControlAllocation::NUM_AXES> mixer;
	allocation.getMixer(mixer);

	ActuatorEffectiveness::EffectivenessMatrix outer = allocation.getEffectivenessMatrix();

	for (int i = 0; i < _num_cc_motors; ++i) {
		outer.col(_cc_motors[i]) = 0.f;
	}

	// Limits first: setEffectivenessMatrix() clips the trim to them
	_outer_allocation->setActuatorMin(allocation.getActuatorMin());
	_outer_allocation->setActuatorMax(allocation.getActuatorMax());

	// _actuator_trim already includes the linearization point
	_outer_allocation->setEffectivenessMatrix(outer, allocation._actuator_trim, ActuatorEffectiveness::ActuatorVector{},
			allocation.numConfiguredActuators(), false);

	// Normalise with the normal allocation's scale (update_normalization_scale is false
	// above, so the outer allocation keeps it), so that a control setpoint means the same
	_outer_allocation->_control_allocation_scale = allocation._control_allocation_scale;

	_outer_update_needed = false;
}

void DtrgConstantCommand::allocate(ControlAllocation &allocation, bool available, int num_motors, bool armed, float dt)
{
	if (!_param_dtrg_cc_en.get() && (_blend <= 0.f)) {
		// disabled and released: the normal allocation, untouched
		_reported_requested = false;
		allocation.allocate();
		return;
	}

	const bool requested = switchRequested();

	if (_blend <= 0.f) {
		// a new selection only applies while released
		latchMotors();
	}

	_motors_valid = motorsValid(num_motors);
	available = available && _motors_valid && (_outer_allocation != nullptr);

	reportTransitions(requested && armed, available);

	float cc_cmd[MAX_COMMAND_MOTORS];

	for (float &cmd : cc_cmd) {
		cmd = NAN;
	}

	if (!available || !armed) {
		// nothing to ramp from: the outer allocation cannot be used, or the outputs are disarmed
		_blend = 0.f;

	} else {
		const float ramp = _param_dtrg_cc_ramp.get();
		const float step = (ramp > FLT_EPSILON) ? (dt / ramp) : 1.f;
		_blend = math::constrain(_blend + (requested ? step : -step), 0.f, 1.f);
	}

	if (_blend <= 0.f) {
		allocation.allocate();
		publishStatus(requested, available, cc_cmd);
		return;
	}

	if (_outer_update_needed) {
		updateOuterAllocation(allocation);
	}

	const float held = math::constrain(_param_dtrg_cc_m_cmd.get(), 0.f, 1.f);

	if (_blend < 1.f) {
		// Ramping: blend from the commands the normal allocation gives the held rotors now
		allocation.allocate();

		for (int i = 0; i < _num_cc_motors; ++i) {
			const float normal = allocation._actuator_sp(_cc_motors[i]);
			cc_cmd[i] = normal + _blend * (held - normal);
		}

	} else {
		// allocate() is skipped, keep the previous setpoint for the slew rate limit
		allocation._prev_actuator_sp = allocation._actuator_sp;

		for (int i = 0; i < _num_cc_motors; ++i) {
			cc_cmd[i] = held;
		}
	}

	// What the centre rotors produce, in the normalised units of the control setpoint
	const ActuatorEffectiveness::EffectivenessMatrix &effectiveness = allocation.getEffectivenessMatrix();
	Vector<float, ControlAllocation::NUM_AXES> cc_control{};

	for (int i = 0; i < _num_cc_motors; ++i) {
		const int motor = _cc_motors[i];
		cc_control += Vector<float, ControlAllocation::NUM_AXES>(effectiveness.col(motor))
			      * (cc_cmd[i] - allocation._actuator_trim(motor));
	}

	_outer_allocation->setControlSetpoint(allocation.getControlSetpoint()
					      - cc_control.emult(allocation._control_allocation_scale));
	_outer_allocation->allocate();

	allocation._actuator_sp = _outer_allocation->getActuatorSetpoint();

	for (int i = 0; i < _num_cc_motors; ++i) {
		allocation._actuator_sp(_cc_motors[i]) = cc_cmd[i];
	}

	publishStatus(requested, available, cc_cmd);
}

void DtrgConstantCommand::reportTransitions(bool requested, bool available)
{
	// No trailing tab: there is no matching event, so the ground station shows the text.
	// Keep within MAVLINK_LOG_MAXLEN (50).
	if (requested != _reported_requested) {
		_reported_requested = requested;

		if (requested && !_motors_valid) {
			mavlink_log_critical(&_mavlink_log_pub, "Constant command: invalid DTRG_CC_MOTORS");

		} else if (requested && !available) {
			mavlink_log_critical(&_mavlink_log_pub, "Constant command unavailable, check CA config");
		}
	}

	const bool engaged = requested && available;

	if (engaged != _reported_engaged) {
		_reported_engaged = engaged;

		if (engaged) {
			mavlink_log_info(&_mavlink_log_pub, "Constant command engaged");

		} else {
			mavlink_log_info(&_mavlink_log_pub, "Constant command released");
		}
	}
}

void DtrgConstantCommand::publishStatus(bool requested, bool available, const float cc_cmd[MAX_COMMAND_MOTORS])
{
	const hrt_abstime now = hrt_absolute_time();

	const bool changed = (requested != _status.requested) || (available != _status.available)
			     || (fabsf(_blend - _status.blend) > FLT_EPSILON);

	// every allocation while held or ramping, otherwise on change and at 1 Hz
	if (!changed && !active() && (now - _last_status_pub < 1_s)) {
		return;
	}

	_status.timestamp = now;
	_status.requested = requested;
	_status.available = available;
	_status.blend = _blend;
	_status.motors = static_cast<uint16_t>(_motors_mask);

	for (int i = 0; i < MAX_COMMAND_MOTORS; ++i) {
		_status.cc_cmd[i] = cc_cmd[i];
	}

	_dtrg_constant_command_pub.publish(_status);
	_last_status_pub = now;
}

void DtrgConstantCommand::printStatus() const
{
	PX4_INFO("DTRG constant command: %s, switch %s, blend %.2f, motors 0x%" PRIx32 " (%s) at %.3f",
		 _param_dtrg_cc_en.get() ? "enabled" : "disabled",
		 _status.requested ? "on" : "off", (double)_blend,
		 _motors_mask, _motors_valid ? "valid" : "invalid", (double)_param_dtrg_cc_m_cmd.get());
}
