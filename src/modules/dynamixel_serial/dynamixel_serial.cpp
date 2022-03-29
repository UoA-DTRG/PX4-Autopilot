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
 * @brief Module for sending Dynamixel commands using serial port (based on FrSky telemetry)
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

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>


/**
 * Opens the UART device and sets all required serial parameters.
 */
int dynamixel_open_uart(const int baud, const char *uart_name, struct termios *uart_config,
		struct termios *uart_config_original)
{
	/* Set baud rate */
	// Dynamixel works on 9600 57600(default) 115200 1M 2M 3M 4M 4.5M

#ifndef B1000000
#define B1000000 1000000
#endif

	unsigned int speed = B57600;

	switch (baud) {
	case 9600:   speed = B9600;   break;

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
		PX4_ERR("Unsupported baudrate: %d\n\tsupported examples:\n\t9600, 57600\t\n115200\n1000000\n",
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

	if (cfsetispeed(uart_config, speed) < 0 || cfsetospeed(uart_config, speed) < 0) {
		PX4_ERR("%s: %d (cfsetispeed, cfsetospeed)\n", uart_name, termios_state);
		close(uart);
		return -1;
	}

	if ((termios_state = tcsetattr(uart, TCSANOW, uart_config)) < 0) {
		PX4_ERR("%s (tcsetattr)\n", uart_name);
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
	PX4_INFO("Single Wire");
}

void set_uart_invert(int uart, bool invert)
{
	// Not all architectures support this. That's ok as it will just re-test the non-inverted case
	ioctl(uart, TIOCSINVERT, invert ? (SER_INVERT_ENABLED_RX | SER_INVERT_ENABLED_TX) : 0);
	PX4_INFO("Inverted UART");
}


int DynamixelSerial::print_status()
{
	PX4_INFO("Running");
	PX4_INFO("Baudrate: %i", _baudrate);
	PX4_INFO("Packets sent: %lu", sentPackets);

	return 0;
}

int DynamixelSerial::custom_command(int argc, char *argv[])
{
	/*
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	// additional custom commands can be handled like this:
	if (!strcmp(argv[0], "do-something")) {
		get_instance()->do_something();
		return 0;
	}
	 */

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
	const char *device_name = "/dev/ttyS1"; /* default USART8 */;
	int baud = 57600;
	unsigned scanning_timeout_ms = 0;

	bool error_flag = false;
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:b:t:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device_name = myoptarg;

			if (access(device_name, F_OK) == -1) {
				PX4_ERR("Device %s does not exist", device_name);
				error_flag = true;
			}

			break;

		case 'b':
			baud = (int)strtol(myoptarg, nullptr, 10);

			if (baud < 9600 || baud > 3000000) {
				PX4_ERR("invalid baud rate '%s'", myoptarg);
				error_flag = true;
			}

			break;

		case 't':
			scanning_timeout_ms = strtoul(myoptarg, nullptr, 10) * 1000;
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

	/* Open UART */
	struct termios uart_config_original;
	struct termios uart_config;
	const int uart = dynamixel_open_uart(baud, device_name, &uart_config, &uart_config_original);

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
			set_uart_invert(uart, false);
			PX4_INFO("%u",scanning_timeout_ms);
		}

		return instance;
	}

}

// DynamixelSerial::DynamixelSerial(int uart, int baud)
// 	: ModuleParams(nullptr)
// {

// }

void DynamixelSerial::run()
{
	// Example: run the loop synchronized to the sensor_combined topic publication
	// int sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
	DynamixelProtocol dynamixel;

	dynamixel.init(_uart, _baudrate);

	//px4_pollfd_struct_t fds[1];
	//fds[0].fd = dynamixel.get_uart();
	//fds[0].events = POLLIN;

	struct pollfd fds[1];
	fds[0].fd = dynamixel.get_uart();
	fds[0].events = POLLIN;

	uint8_t sbuf[64];
	// char sbuf[20];
	// const hrt_abstime start_time = hrt_absolute_time();

	PX4_INFO("_uart %i",_uart);
	PX4_INFO("_baudrate %i",_baudrate);

	// initialize parameters
	parameters_update(true);

	while (!should_exit()) {

		// wait for up to 1000ms for data
		////int pret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 1000);
		//int status = px4_poll(fds, sizeof(fds) / sizeof(fds[0]), 100);
		int status = poll(fds, sizeof(fds) / sizeof(fds[0]), 100);
		_comm_state = true;

		//Scan for packets
		if (!_comm_state) {

			if (status == 0) {
				// Timeout: let the loop run anyway, don't do `continue` here
				PX4_INFO("status == 0");


			} else if (status < 0) {
				// this is undesirable but not much we can do
				PX4_ERR("poll error %d, %d", status, errno);
				px4_usleep(50000);
				continue;

			} else if (status > 0) {

				usleep(50_ms);
				int nbytes = read(dynamixel.get_uart(), &sbuf[0], sizeof(sbuf));
				PX4_INFO("dynamixel input: %d bytes: %x %x %x %x %x %x", nbytes, sbuf[0], sbuf[1],sbuf[2], sbuf[3],sbuf[4], sbuf[5]);


				if (nbytes > 5) {
					_comm_state = true;
					PX4_INFO("SCAN: packet found!");
					//This is only being found when Dynamixel connected to computer
					//ttsy1 second cable (next to red)
					//After that the module stops running
				}

			}

			PX4_INFO("EXIT comm_state false");

			// usleep(100_ms);
			// // flush buffer
			// read(dynamixel.get_uart(), &sbuf[0], sizeof(sbuf));

			// // check for a timeout
			// if (_scanning_timeout_ms > 0 && (hrt_absolute_time() - start_time) / 1000 > _scanning_timeout_ms) {
			// 	PX4_INFO("Scanning timeout: exiting");
			// 	break;
			// }

		} else {

			//usleep(200_ms);
			// flush buffer
			//read(dynamixel.get_uart(), &sbuf[0], sizeof(sbuf));
			//PX4_INFO("flushed buffer");

			PX4_INFO("sending Dynamixel commands");
			usleep(150_ms);
			dynamixel.update((uint32_t) status);


			int nbytes = read(dynamixel.get_uart(), &sbuf[0], sizeof(sbuf));
			PX4_INFO("dynamixel input: %d bytes: %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x", nbytes, sbuf[0], sbuf[1],sbuf[2], sbuf[3],sbuf[4], sbuf[5], sbuf[6],sbuf[7], sbuf[8], sbuf[9], sbuf[10], sbuf[11], sbuf[12], sbuf[13], sbuf[14], sbuf[15]);

			// usleep(100_ms);
			// write(dynamixel.get_uart(), &sbuf[0], sizeof(sbuf));
			// PX4_INFO("dwrote to port");
		}

		parameters_update();
	}

	/* Reset the UART flags to original state */
	// tcsetattr(dynamixel.get_uart(), TCSANOW, &uart_config_original);

	PX4_INFO("closing uart");
	close(dynamixel.get_uart());

	// orb_unsubscribe(sensor_combined_sub);
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
Start dynamixel communication on ttyS6 serial with baudrate 57600 and no timeout
$ dynamixel_serial start -d /dev/ttyS6 -b 57600 -t 0

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("dynamixel_serial", "communication");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS6", "<file:dev>", "Select Serial Device", true);
	PRINT_MODULE_USAGE_PARAM_INT('b', 57600, 9600, 115200, "Baudrate", true);
	PRINT_MODULE_USAGE_PARAM_INT('t', 0, 0, 60, "Scanning timeout [s] (default: no timeout)", true);

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int dynamixel_serial_main(int argc, char *argv[])
{
	return DynamixelSerial::main(argc, argv);
}
