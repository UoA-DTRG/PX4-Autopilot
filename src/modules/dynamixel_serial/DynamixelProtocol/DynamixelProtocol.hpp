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

// register offsets
#define REG_OPERATING_MODE 11
#define   OPMODE_CURR_CONTROL    0
#define   OPMODE_VEL_CONTROL     1
#define   OPMODE_POS_CONTROL     3
#define   OPMODE_EXT_POS_CONTROL 4

#define REG_TORQUE_ENABLE  64
#define LED_ENABLE  65

#define REG_STATUS_RETURN  68
#define   STATUS_RETURN_NONE 0
#define   STATUS_RETURN_READ 1
#define   STATUS_RETURN_ALL  2

#define REG_GOAL_POSITION 116

// how many times to send servo configure msgs
#define CONFIGURE_SERVO_COUNT 4

// how many times to send servo detection
#define DETECT_SERVO_COUNT 4


class DynamixelProtocol
{
public:

	DynamixelProtocol() = default;
	~DynamixelProtocol() = default;


	void update(uint32_t n, unsigned short int pos, unsigned short int led);
	int get_uart() {return uart;}
	void init(const int serial_uart, uint32_t serial_baud);

private:

	int uart;
	uint32_t baudrate;
	uint32_t us_per_byte;
	uint32_t us_gap;


	void detect_servos();

	void add_stuffing(uint8_t *packet);
	void send_packet(uint8_t *txpacket);
	void read_bytes(uint32_t n);
	void process_packet(const uint8_t *pkt, uint8_t length);
	void send_command(uint8_t id, uint16_t reg, uint32_t value, uint8_t len);
	void configure_servos(void);
	uint16_t updateCRC(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size);

	// auto-detected mask of available servos, from a broadcast ping
	uint16_t servo_mask;
	uint8_t detection_count{0};
	uint8_t configured_servos;
	bool initialised;

	uint8_t pktbuf[64];
	uint8_t pktbuf_ofs;

	// servo position limits
	uint32_t pos_min;
	uint32_t pos_max;

	uint32_t last_send_us;
	uint32_t delay_time_us;
};
