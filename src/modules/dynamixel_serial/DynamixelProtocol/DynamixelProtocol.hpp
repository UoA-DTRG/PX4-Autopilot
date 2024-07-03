/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file DynamixelProtocol.hpp
 * @brief Dynamixel Protocol 2.0 definitions
 *
 * @author Pedro Mendes <pmen817@aucklanduni.ac.nz>
 *
 */

#pragma once

#include <sys/types.h>
#include <stdbool.h>
#include <drivers/drv_hrt.h>

#define BROADCAST_ID        0xFE    // 254
#define MAX_ID              0xFC    // 252

/* Macro for Control Table Value */
#define DXL_MAKEWORD(a, b)  ((uint16_t)(((uint8_t)(((uint64_t)(a)) & 0xff)) | ((uint16_t)((uint8_t)(((uint64_t)(b)) & 0xff))) << 8))
#define DXL_MAKEDWORD(a, b) ((uint32_t)(((uint16_t)(((uint64_t)(a)) & 0xffff)) | ((uint32_t)((uint16_t)(((uint64_t)(b)) & 0xffff))) << 16))
#define DXL_LOWORD(l)       ((uint16_t)(((uint64_t)(l)) & 0xffff))
#define DXL_HIWORD(l)       ((uint16_t)((((uint64_t)(l)) >> 16) & 0xffff))
#define DXL_LOBYTE(w)       ((uint8_t)(((uint64_t)(w)) & 0xff))
#define DXL_HIBYTE(w)       ((uint8_t)((((uint64_t)(w)) >> 8) & 0xff))

/* Instruction for DXL Protocol */
#define INST_PING               1
#define INST_READ               2
#define INST_WRITE              3
#define INST_REG_WRITE          4
#define INST_ACTION             5
#define INST_FACTORY_RESET      6
#define INST_SYNC_WRITE         131     // 0x83
#define INST_BULK_READ          146     // 0x92
// --- Only for 2.0 --- //
#define INST_REBOOT             8
#define INST_CLEAR              16      // 0x10
#define INST_STATUS             85      // 0x55
#define INST_SYNC_READ          130     // 0x82
#define INST_BULK_WRITE         147     // 0x93

///////////////// for Protocol 2.0 Packet /////////////////
#define PKT_HEADER0             0
#define PKT_HEADER1             1
#define PKT_HEADER2             2
#define PKT_RESERVED            3
#define PKT_ID                  4
#define PKT_LENGTH_L            5
#define PKT_LENGTH_H            6
#define PKT_INSTRUCTION         7
#define PKT_ERROR               8
#define PKT_PARAMETER0          8

#define   OPMODE_CURR_CONTROL    0
#define   OPMODE_VEL_CONTROL     1
#define   OPMODE_POS_CONTROL     3
#define   OPMODE_EXT_POS_CONTROL 4

#define   STATUS_RETURN_NONE 0
#define   STATUS_RETURN_READ 1
#define   STATUS_RETURN_ALL  2

// how many times to send servo configure msgs
#define CONFIGURE_SERVO_COUNT 4

// how many times to send servo detection
#define DETECT_SERVO_COUNT 4

enum class Reg : uint16_t {
	// EEPROM
	MODEL_NUMBER = 0,
	MODEL_INFORMATION = 2,
	VERSION_OF_FIRMWARE = 6,
	ID = 7,
	BAUDRATE = 8,
	RETURN_DELAY_TIME = 9,
	DRIVE_MODE = 10,
	OPERATING_MODE = 11,
	SECONDARY_ID = 12,
	PROTOCOL_VERSION = 13,
	HOMING_OFFSET = 20,
	MOVING_THRESHOLD = 24,
	TEMPERATURE_LIMIT = 31,
	MAX_VOLTAGE_LIMIT = 32,
	MIN_VOLTAGE_LIMIT = 34,
	PWM_LIMIT = 36,
	CURRENT_LIMIT = 38,
	ACCELERATION_LIMIT = 40,
	VELOCITY_LIMIT = 44,
	MAX_POSITION_LIMIT = 48,
	MIN_POSITION_LIMIT = 52,
	SHUTDOWN = 63,
	// RAM
	TORQUE_ENABLE = 64,
	LED = 65,
	STATUS_RETURN_LEVEL = 68,
	REGISTERED_INSTRUCTION = 69,
	HARDWARE_ERROR_STATUS = 70,
	VELOCITY_I_GAIN = 76,
	VELOCITY_P_GAIN = 78,
	POSITION_D_GAIN = 80,
	POSITION_I_GAIN = 82,
	POSITION_P_GAIN = 84,
	FEEDFORWARD_ACCELERATION_GAIN = 88,
	FEEDFORWARD_VELOCITY_GAIN = 90,
	BUS_WATCHDOG = 98,
	GOAL_PWM = 100,
	GOAL_CURRENT = 102,
	GOAL_VELOCITY = 104,
	PROFILE_ACCELERATION = 108,
	PROFILE_VELOCITY = 112,
	GOAL_POSITION = 116,
	REALTIME_TICK = 120,
	MOVING = 122,
	MOVING_STATUS = 123,
	PRESENT_PWM = 124,
	PRESENT_CURRENT = 126,
	PRESENT_VELOCITY = 128,
	PRESENT_POSITION = 132,
	VELOCITY_TRAJECTORY = 136,
	POSITION_TRAJECTORY = 140,

	PRESENT_INPUT_VOLTAGE = 144,
	PRESENT_TEMPERATURE = 146,

	EXTERNAL_PORT_DATA_1 = 152,
	EXTERNAL_PORT_DATA_2 = 154,
	EXTERNAL_PORT_DATA_3 = 156,

	INDIRECT_ADDR_1 = 168,
	INDIRECT_DATA_1 = 224,
	INDIRECT_ADDR_29 = 578,
	INDIRECT_DATA_29 = 634,

};

using namespace time_literals;

class DynamixelProtocol
{
public:

	DynamixelProtocol() = default;
	~DynamixelProtocol() = default;

	void init(const int serial_uart, uint32_t serial_baud);
	bool update();

	void set_setpoints(int i, int32_t val, uint32_t led, uint32_t mode);
	int get_uart() {return uart;}


private:

	int uart;
	uint32_t baudrate;
	uint32_t us_per_byte;
	uint32_t us_gap;

	void detect_servos();

	void add_stuffing(uint8_t *packet);
	void send_packet(uint8_t *txpacket);
	void read_bytes();
	void process_packet(const uint8_t *pkt, uint8_t length);
	void send_command(uint8_t id, Reg reg_addr, int32_t value);
	void configure_servos(void);
	uint16_t updateCRC(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size);
	uint8_t get_size(Reg reg);

	// auto-detected mask of available servos, from a broadcast ping
	uint16_t servo_mask{0};
	uint8_t detection_count{0};
	uint8_t configured_servos{0};
	bool initialised;

	uint8_t pktbuf[72];
	uint8_t pktbuf_ofs;

	uint32_t op_mode{3};

	int32_t val_sp[17] = {0}; //setpoint array
	int32_t led_sp[17] = {0}; //setpoint array

	bool broadcast;           //broadcast flag

	uint32_t last_send_us;
	uint32_t delay_time_us;

};
