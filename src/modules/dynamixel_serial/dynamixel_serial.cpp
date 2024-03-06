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
 * @file dynamixel_serial.cpp
 * @brief Module for sending Dynamixel commands using serial port
 *
 * @author Pedro Mendes <pmen817@aucklanduni.ac.nz>
 *
 */

#include "dynamixel_serial.h"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <poll.h>
#include <fcntl.h>
#include <unistd.h>

#include <systemlib/err.h>
#include <drivers/drv_hrt.h>
#include <math.h>

int32_t dyn_extpos_rad2raw(float rad)
{
	//dynamixel XM430 can go up to 256 turns for each side
	float rad_min = 256 * (-3.141593) * 2; //minimum range of motion in radians
	float rad_max = 256 * (3.141593) * 2; //maximum range of motion in radians
	int32_t max_raw = 1048575;
	int32_t min_raw = -1048575;

	int32_t raw = (int32_t)(rad*((max_raw - min_raw)/(rad_max - rad_min)));

	return raw;
}

/**
 * Opens the UART device and sets all required serial parameters.
 */
int dynamixel_open_uart(const int baud, const char *uart_name, struct termios *uart_config,
			struct termios *uart_config_original)
{
	/* Set baud rate */
	// Dynamixel works on 9600 19200 57600(default) 115200 1M 2M 3M

#ifndef B1000000
#define B1000000 1000000
#endif

	unsigned int speed = B57600; //default

	switch (baud) {
	case 9600:   speed = B9600;   break;

	case 19200:  speed = B19200;  break;

	case 57600:  speed = B57600;  break;

	case 115200: speed = B115200; break;

	case 1000000: speed = B1000000; break;

#ifdef B2000000

	case 2000000: speed = B2000000; break;
#endif

#ifdef B3000000

	case 3000000: speed = B3000000; break;
#endif

	default:
		PX4_ERR("Unsupported baudrate: %d\n\tsupported examples:\n\t9600, 19200, (default) 57600\t\n115200\n1000000\n",
			baud);
		return -EINVAL;
	}

	/* Open UART */
	const int uart = open(uart_name, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (uart < 0) {
		PX4_ERR("Error opening port: %s (%i)", uart_name, errno);
		return -1;
	}

	/* Back up the original UART configuration to restore it after exit */
	int termios_state;

	/* Initialize the uart config */
	if ((termios_state = tcgetattr(uart, uart_config_original)) < 0) {
		PX4_ERR("ERR GET CONF %s: %d\n", uart_name, termios_state);
		close(uart);
		return -1;
	}

	/* Fill the struct for the new configuration */
	tcgetattr(uart, uart_config);

	/* Disable output post-processing */
	uart_config->c_oflag &= ~OPOST;

	uart_config->c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
	uart_config->c_cflag &= ~CSIZE;
	uart_config->c_cflag |= CS8;         /* 8-bit characters */
	uart_config->c_cflag &= ~PARENB;     /* no parity bit */
	uart_config->c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
	uart_config->c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

	/* setup for non-canonical mode */
	uart_config->c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	uart_config->c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

	/* Set baud rate */
	if (cfsetispeed(uart_config, speed) < 0 || cfsetospeed(uart_config, speed) < 0) {
		PX4_ERR("ERR SET BAUD %s: %d\n", uart_name, termios_state);
		close(uart);
		return -1;
	}

	if ((termios_state = tcsetattr(uart, TCSANOW, uart_config)) < 0) {
		PX4_WARN("ERR SET CONF %s\n", uart_name);
		close(uart);
		return -1;
	}

	return uart;

}

int set_uart_speed(int uart, struct termios *uart_config, unsigned int speed)
{

	if (cfsetispeed(uart_config, speed) < 0) {
		return -1;
	}

	if (tcsetattr(uart, TCSANOW, uart_config) < 0) {
		return -1;
	}

	return uart;
}

void set_uart_single_wire(int uart, bool single_wire)
{
	if (ioctl(uart, TIOCSSINGLEWIRE, single_wire ? (SER_SINGLEWIRE_ENABLED | SER_SINGLEWIRE_PUSHPULL |
			SER_SINGLEWIRE_PULLDOWN) : 0) < 0) {
		PX4_WARN("setting TIOCSSINGLEWIRE failed");
	}
}

void set_uart_invert(int uart, bool invert)
{
	// Not all architectures support this. That's ok as it will just re-test the non-inverted case
	ioctl(uart, TIOCSINVERT, invert ? (SER_INVERT_ENABLED_RX | SER_INVERT_ENABLED_TX) : 0);
}


int DynamixelSerial::print_status()
{
	PX4_INFO("Running");
	PX4_INFO("Device: %s", (char const *) _device_name);
	PX4_INFO("Baudrate: %i", _baudrate);
	PX4_INFO("Packets sent: %lu", sentPackets);

	return 0;
}

bool DynamixelSerial::constrain_input(int val, unsigned short mode)
{
	bool constrain = false;

	//return true if input is constrained
	if (mode == 3) { // Position mode

		if (val > _param_dyn_posmax.get()) {
			_val_cmd = _param_dyn_posmax.get();
			constrain = true;

		} else if (val < _param_dyn_posmin.get()) {
			_val_cmd = _param_dyn_posmin.get();
			constrain = true;

		} else {
			_val_cmd = val;
			constrain = false;
		}

	} else if (mode == 4) { // Extended Position mode

		if (val > _param_dyn_extmax.get()) {
			_val_cmd = _param_dyn_extmax.get();
			constrain = true;

		} else if (val < -_param_dyn_extmax.get()) {
			_val_cmd = -_param_dyn_extmax.get();
			constrain = true;

		} else {
			_val_cmd = val;
			constrain = false;
		}

	} else if (mode == 1) { // Velocity mode

		if (val > _param_dyn_velmax.get()) {
			_val_cmd = _param_dyn_velmax.get();
			constrain = true;

		} else if (val < -_param_dyn_velmax.get()) {
			_val_cmd = -_param_dyn_velmax.get();
			constrain = true;

		} else {
			_val_cmd = val;
			constrain = false;
		}

	} else if (mode == 0) { // Current mode

		if (val > _param_dyn_curmax.get()) {
			_val_cmd = _param_dyn_curmax.get();
			constrain = true;

		} else if (val < -_param_dyn_curmax.get()) {
			_val_cmd = -_param_dyn_curmax.get();
			constrain = true;

		} else {
			_val_cmd = val;
			constrain = false;
		}

	}

	return constrain;
}

int DynamixelSerial::custom_command(int argc, char *argv[])
{

	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	if (!strcmp(argv[0], "send")) {

		if (argc > 4) {
			_servo_id_cmd = atof(argv[2]);
			_val_cmd = atof(argv[3]);
			_led_cmd = atof(argv[4]);

			if (!strcmp(argv[1], "position")) {
				_mode_cmd = 3;

			} else if (!strcmp(argv[1], "extposition")) {
				_mode_cmd = 4;

			} else if (!strcmp(argv[1], "velocity")) {
				_mode_cmd = 1;

			} else if (!strcmp(argv[1], "current")) {
				_mode_cmd = 0;

			} else {
				PX4_WARN("Invalid Mode '%s'...\n Setting to Position Control", argv[1]);
				_mode_cmd = 3;
			}

			PX4_INFO("Mode is: %u - %s", _mode_cmd, argv[1]);

			if (_servo_id_cmd < 1 || (_servo_id_cmd > 16 && _servo_id_cmd != 254)) {
				PX4_WARN("Invalid ID '%s'...\n Setting ID to 1. Valid range: 1-16 or 254 for broadcast", argv[2]);
				_servo_id_cmd = 1;
			}

			if (_led_cmd > 1) {
				PX4_WARN("Invalid LED switch '%s'...\n Setting to 1.", argv[4]);
				_led_cmd = 1;
			}

			return 0;

		} else {
			PX4_ERR("missing argument");
			return 0;
		}
	}

	return print_usage("unknown command");
}


int DynamixelSerial::task_spawn(int argc, char *argv[])
{

	_task_id = px4_task_spawn_cmd("dynamixel_serial",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT + 4,
				      1400,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

DynamixelSerial *DynamixelSerial::instantiate(int argc, char *argv[])
{
	const char *device_name = DEFAULT_DEVICE_NAME; /* default device*/;
	int baud = 57600;  			/* default baudrate */;
	sentPackets = 0;

	bool error_flag = false;
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:b:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {

		case 'd':
			device_name = myoptarg;

			PX4_INFO("Device %s selected", device_name);

			if (access(device_name, F_OK) == -1) {
				PX4_ERR("Device %s does not exist", device_name);
				error_flag = true;
			}

			break;

		case 'b':
			if (px4_get_parameter_value(myoptarg, baud) != 0) {
				PX4_ERR("baudrate parsing failed");
				error_flag = true;
			}

			if (baud  < 9600 || baud > 3000000) {
				PX4_ERR("invalid baud rate '%s'", myoptarg);
				error_flag = true;
			}

			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return nullptr;
	}

	//make a copy of device_name to be used in status command
	_device_name = device_name;

	/* Open UART */
	struct termios uart_config;
	const int uart = dynamixel_open_uart(baud, device_name, &uart_config, &_uart_config_original);

	if (uart < 0) {
		device_name = NULL;
		PX4_ERR("failed to open UART");
		return nullptr;

	} else {
		DynamixelSerial *instance = new DynamixelSerial(uart, baud);

		if (instance == nullptr) {
			PX4_ERR("alloc failed");

		} else {
			PX4_INFO("alloc success");
			PX4_INFO("setting baud rate to %d (single wire)", baud);
			set_uart_speed(uart, &uart_config, baud);
			// switch to single-wire (half-duplex) mode, because S.Port uses only a single wire
			set_uart_single_wire(uart, true);
			set_uart_invert(uart, false);     //Only works if set to false (no need to invert)
		}

		return instance;
	}

}

void DynamixelSerial::run()
{
	DynamixelProtocol dynamixel;

	dynamixel.init(_uart, _baudrate);
	_comm_state = false;
	_ext_flag = false;

	px4_pollfd_struct_t fds[1];
	fds[0].fd = dynamixel.get_uart();
	fds[0].events = POLLIN;

	// uint8_t sbuf[64];
	// const hrt_abstime start_time = hrt_absolute_time();

	PX4_INFO("_uart %i", _uart);
	PX4_INFO("_baudrate %i", _baudrate);

	// initialize parameters
	parameters_update(true);

	debug_vect_s flags_vect;
	rc_channels_s _rc_channels;

	while (!should_exit()) {

		int status = px4_poll(fds, sizeof(fds) / sizeof(fds[0]), 50);

		if ((status < 1) && _comm_state) { continue; }

		if (_debug_vect_sub.updated()) {

			_debug_vect_sub.copy(&flags_vect);
		}
		if (_rc_channels_sub.updated()) {

			_rc_channels_sub.copy(&_rc_channels);
		}

		if (constrain_input(_val_cmd, _mode_cmd)) {
			PX4_WARN("Setpoint outside limits...\n Setting to %i.", _val_cmd);
		}

		// dynamixel.set_setpoints(_servo_id_cmd, _val_cmd, _led_cmd, _mode_cmd); //Uncomment this if sending commands through console terminal

		int32_t dynx_2_raw = dyn_extpos_rad2raw(flags_vect.y);
		int32_t dynx_1_raw = dyn_extpos_rad2raw(flags_vect.x);

		//saturate to 20 degrees
		float rad20deg = 20*3.141593/180.0;
		int32_t max20deg = dyn_extpos_rad2raw(rad20deg);

		//For dynx 1
		if (dynx_1_raw > max20deg)
		{
			dynx_1_raw = max20deg;
		}
		else if (dynx_1_raw < -max20deg)
		{
			dynx_1_raw = -max20deg;
		}
		//For dynx 2
		if (dynx_2_raw > max20deg)
		{
			dynx_2_raw = max20deg;
		}
		else if (dynx_2_raw < -max20deg)
		{
			dynx_2_raw = -max20deg;
		}




		//Apparently it has to be sent with the ids in order
		// dynamixel.set_setpoints(1, dynx_1_raw, 1, OPMODE_EXT_POS_CONTROL);
		// dynamixel.set_setpoints(2, dynx_2_raw, 0, OPMODE_POS_CONTROL);

		//Handle offset of dynamixel 1
		dynx_1_raw = dynx_1_raw + 2488;

		//Controll the gripper (id = 3)
		if (_rc_channels.channels[5] > (float)-0.1)
		{
			dynamixel.set_setpoints(1, dynx_1_raw, 0, OPMODE_EXT_POS_CONTROL);
			dynamixel.set_setpoints(2, dynx_2_raw, 0, OPMODE_EXT_POS_CONTROL);
			// PX4_INFO("dynx_1_raw: %ld", dynx_1_raw);
			// PX4_INFO("dynx_2_raw: %ld", dynx_2_raw);
		}
		else
		{
			dynamixel.set_setpoints(1, dynx_1_raw, 0, OPMODE_EXT_POS_CONTROL);
			dynamixel.set_setpoints(2, dynx_2_raw, 0, OPMODE_EXT_POS_CONTROL);
		}



		// read(dynamixel.get_uart(), &sbuf[0], sizeof(sbuf));

		if (dynamixel.update()) {
			_comm_state = true;
			sentPackets++;
			// PX4_INFO("Passed on update dynamixel out true");
		}

		parameters_update();
	}

	_comm_state = false;

	/* Reset the UART flags to original state */
	tcsetattr(dynamixel.get_uart(), TCSANOW, &_uart_config_original);

	PX4_INFO("closing uart");
	close(dynamixel.get_uart());

}

void DynamixelSerial::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		// update parameters from storage
		updateParams();

		_mode_cmd = _param_dyn_mode.get();
	}
}

int DynamixelSerial::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module implements the usage of Dynamixel commands using a serial port.

### Examples
Start dynamixel communication on ttyS3 serial with baudrate 57600
$ dynamixel_serial start -d /dev/ttyS3 -b 57600

### Send Dynamixel Write and Switch LED:
Available modes: (position,extposition,velocity,current)

Position Control Mode = position
Servo_ID = 1 (214 for brodcast all)
Setpoint = 512
Turn LED On = 1

$ dynamixel_serial send position 1 512 1

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("dynamixel_serial", "communication");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS3", "<file:dev>", "Select Serial Device", true);
	PRINT_MODULE_USAGE_PARAM_INT('b', 57600, 9600, 3000000, "Baudrate", true);

	PRINT_MODULE_USAGE_COMMAND("send");
	PRINT_MODULE_USAGE_ARG("MODE, ID, SETPOINT, LED", "Mode:(position,extposition,velocity,current), Servo ID, Setpoint, Switch LED", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("MODE|ID|SETPOINT|LED", "Send Dynamixel Write");

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int dynamixel_serial_main(int argc, char *argv[])
{
	return DynamixelSerial::main(argc, argv);
}
