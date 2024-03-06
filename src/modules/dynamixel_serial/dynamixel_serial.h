/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file dynamixel_serial.h
 * @brief Module for sending Dynamixel commands using serial port
 *
 * @author Pedro Mendes <pmen817@aucklanduni.ac.nz>
 *
 */

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/cli.h>

#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/debug_vect.h>
#include <uORB/topics/rc_channels.h>

#include <termios.h>
#include "DynamixelProtocol/DynamixelProtocol.hpp"

#define DEFAULT_DEVICE_NAME     "/dev/ttyS3"

using namespace time_literals;

extern "C" __EXPORT int dynamixel_serial_main(int argc, char *argv[]);

const char *_device_name{DEFAULT_DEVICE_NAME};
struct termios _uart_config_original; //save original uart config
static unsigned long int sentPackets = 0;

int _val_cmd = 0;  //custom command
unsigned short int _led_cmd = 1;    //custom command
unsigned short int _mode_cmd = 4; //custom command #JOAO set to 4, extended position control
unsigned short int _servo_id_cmd = 1;  //custom command

class DynamixelSerial : public ModuleBase<DynamixelSerial>, public ModuleParams
{
public:
	DynamixelSerial(int uart, int baud) : ModuleParams(nullptr), _uart(uart), _baudrate(baud)
	{
	};

	virtual ~DynamixelSerial() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static DynamixelSerial *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:

	/* Default values for arguments */
	int _uart {};
	int _baudrate {};
	bool _comm_state = false;

	float _ext_setpoint{0};  //Need to receive value from mavlink
	bool _ext_flag = false;
	hrt_abstime _debug_timestamp_last{};

	/**
	 * Check for parameter changes and update them if needed.
	 * @param parameter_update_sub uorb subscription to parameter_update
	 * @param force for a parameter update
	 */
	void parameters_update(bool force = false);
	bool constrain_input(int val, unsigned short mode);


	DEFINE_PARAMETERS(
		(ParamInt<px4::params::DYN_SER_MODE>)   _param_dyn_mode,
		(ParamInt<px4::params::DYN_SER_POS_MIN>) _param_dyn_posmin,
		(ParamInt<px4::params::DYN_SER_POS_MAX>) _param_dyn_posmax,
		(ParamInt<px4::params::DYN_SER_VEL_MAX>) _param_dyn_velmax,
		(ParamInt<px4::params::DYN_SER_CUR_MAX>) _param_dyn_curmax,
		(ParamInt<px4::params::DYN_SER_EXT_MAX>) _param_dyn_extmax
	)

	// Subscriptions
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _debug_vect_sub{ORB_ID(debug_vect)};
	uORB::Subscription _rc_channels_sub{ORB_ID(rc_channels)};
};

