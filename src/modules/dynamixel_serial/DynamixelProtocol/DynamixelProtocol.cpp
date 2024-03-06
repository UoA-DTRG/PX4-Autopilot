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
 * @file DynamixelProtocol.cpp
 * @brief Dynamixel Protocol 2.0 definitions
 *
 * @author Pedro Mendes <pmen817@aucklanduni.ac.nz>
 *
 */

#include <DynamixelProtocol.hpp>
#include <px4_platform_common/log.h>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <poll.h>

#include <drivers/drv_hrt.h>


void DynamixelProtocol::init(const int serial_uart, uint32_t serial_baud)
{
	uart = serial_uart;
	baudrate = serial_baud;
	us_per_byte = 10 * 1e6 / baudrate;
	us_gap = 4 * 1e6 / baudrate;

	initialised = true;
	broadcast = false;

	detection_count = 0;
	last_send_us = 0;
	delay_time_us = 0;
	pktbuf_ofs = 0;

	op_mode = 3;
}

/*
  addStuffing() from Robotis SDK. This pads the packet as required by the protocol
*/
void DynamixelProtocol::add_stuffing(uint8_t *packet)
{
	int packet_length_in = DXL_MAKEWORD(packet[PKT_LENGTH_L], packet[PKT_LENGTH_H]);
	int packet_length_out = packet_length_in;

	if (packet_length_in < 8) { // INSTRUCTION, ADDR_L, ADDR_H, CRC16_L, CRC16_H + FF FF FD
		return;
	}

	uint8_t *packet_ptr;
	uint16_t packet_length_before_crc = packet_length_in - 2;

	for (uint16_t i = 3; i < packet_length_before_crc; i++) {
		packet_ptr = &packet[i + PKT_INSTRUCTION - 2];

		if (packet_ptr[0] == 0xFF && packet_ptr[1] == 0xFF && packet_ptr[2] == 0xFD) {
			packet_length_out++;
		}
	}

	if (packet_length_in == packet_length_out) { // no stuffing required
		return;
	}

	uint16_t out_index  = packet_length_out + 6 - 2;  // last index before crc
	uint16_t in_index   = packet_length_in + 6 - 2;   // last index before crc

	while (out_index != in_index) {
		if (packet[in_index] == 0xFD && packet[in_index - 1] == 0xFF && packet[in_index - 2] == 0xFF) {
			packet[out_index--] = 0xFD; // byte stuffing

			if (out_index != in_index) {
				packet[out_index--] = packet[in_index--]; // FD
				packet[out_index--] = packet[in_index--]; // FF
				packet[out_index--] = packet[in_index--]; // FF
			}

		} else {
			packet[out_index--] = packet[in_index--];
		}
	}

	packet[PKT_LENGTH_L] = DXL_LOBYTE(packet_length_out);
	packet[PKT_LENGTH_H] = DXL_HIBYTE(packet_length_out);

	return;
}

/*
  send a protocol 2.0 packet
 */
void DynamixelProtocol::send_packet(uint8_t *txpacket)
{
	add_stuffing(txpacket);

	// check max packet length
	uint16_t total_packet_length = DXL_MAKEWORD(txpacket[PKT_LENGTH_L], txpacket[PKT_LENGTH_H]) + 7;

	// make packet header
	txpacket[PKT_HEADER0]   = 0xFF;
	txpacket[PKT_HEADER1]   = 0xFF;
	txpacket[PKT_HEADER2]   = 0xFD;
	txpacket[PKT_RESERVED]  = 0x00;

	// add CRC16
	uint16_t crc = updateCRC(0, txpacket, total_packet_length - 2);    // 2: CRC16
	txpacket[total_packet_length - 2] = DXL_LOBYTE(crc);
	txpacket[total_packet_length - 1] = DXL_HIBYTE(crc);


	write(uart, &txpacket[0], 1);

	for (uint8_t i = 0; i < total_packet_length; i++) {
		write(uart, &txpacket[i], 1);
	}

	delay_time_us += total_packet_length * us_per_byte + us_gap;
}

/*
  use a broadcast ping to find attached servos
 */
void DynamixelProtocol::detect_servos(void)
{
	uint8_t txpacket[10] {};

	txpacket[PKT_ID] = BROADCAST_ID;
	txpacket[PKT_LENGTH_L] = 3;
	txpacket[PKT_LENGTH_H] = 0;
	txpacket[PKT_INSTRUCTION] = INST_PING;

	send_packet(txpacket);

	// give plenty of time for replies from all servos
	last_send_us = hrt_absolute_time();
	delay_time_us += 1000 * us_per_byte;

	read_bytes();
}

/*
broadcast configure all servos
 */
void DynamixelProtocol::configure_servos(void)
{
	// disable torque control
	send_command(BROADCAST_ID, Reg::TORQUE_ENABLE, 0);

	// disable replies unless we read
	send_command(BROADCAST_ID, Reg::STATUS_RETURN_LEVEL, 1);

	// set mode
	send_command(BROADCAST_ID, Reg::OPERATING_MODE, op_mode);

	//For omnirotor. Set the homing offset for core
	// send_command(2, Reg::HOMING_OFFSET, -3941);

	// enable torque control
	send_command(BROADCAST_ID, Reg::TORQUE_ENABLE, 1);

}


/*
  send a command to a single servo, changing a register value
 */
void DynamixelProtocol::send_command(uint8_t id, Reg reg_addr, int32_t value)
{
	uint8_t txpacket[16] {};

	uint16_t reg = static_cast<uint16_t>(reg_addr);
	uint8_t len = get_size(reg_addr);

	txpacket[PKT_ID] = id;
	txpacket[PKT_LENGTH_L] = 5 + len;
	txpacket[PKT_LENGTH_H] = 0;
	txpacket[PKT_INSTRUCTION] = INST_WRITE;
	txpacket[PKT_INSTRUCTION + 1] = DXL_LOBYTE(reg);
	txpacket[PKT_INSTRUCTION + 2] = DXL_HIBYTE(reg);

	memcpy(&txpacket[PKT_INSTRUCTION + 3], &value, ((len < 4) ? len : 4));

	send_packet(txpacket);
}


/*
	read response bytes
 */
void DynamixelProtocol::read_bytes()
{
	usleep(10_ms);
	uint8_t n = read(uart, &pktbuf[0], sizeof(pktbuf));

	if (n == 0 && pktbuf_ofs < PKT_INSTRUCTION) {
		return;
	}

	if (n > sizeof(pktbuf) - pktbuf_ofs) {
		n = sizeof(pktbuf) - pktbuf_ofs;
	}

	for (uint8_t i = 0; i < n; i++) {
		// read 1 byte
		read(uart, &pktbuf[i], 1);
		pktbuf_ofs++;
		usleep(1_ms);
	}


	while (pktbuf_ofs > 0) {

		//discard bad leading data. Find status
		while (pktbuf_ofs >= 4 &&
		       (pktbuf[PKT_HEADER0] != 0xFF || pktbuf[PKT_HEADER1] != 0xFF || pktbuf[PKT_HEADER2] != 0xFD ||
			pktbuf[PKT_RESERVED] != 0x00 || pktbuf[PKT_INSTRUCTION] != INST_STATUS)) {
			memmove(pktbuf, &pktbuf[1], pktbuf_ofs - 1);
			pktbuf_ofs--;
		}

		if (pktbuf_ofs < 10) {
			// not enough data yet
			return;
		}

		const uint16_t total_packet_length = DXL_MAKEWORD(pktbuf[PKT_LENGTH_L], pktbuf[PKT_LENGTH_H]) + PKT_INSTRUCTION;

		if (total_packet_length > sizeof(pktbuf)) {
			pktbuf_ofs = 0;
			return;
		}

		if (pktbuf_ofs < total_packet_length) {
			// more data needed
			return;
		}

		//  check CRC
		const uint16_t crc = DXL_MAKEWORD(pktbuf[total_packet_length - 2], pktbuf[total_packet_length - 1]);
		const uint16_t calc_crc = updateCRC(0, pktbuf, total_packet_length - 2);

		if (calc_crc != crc) {
			memmove(pktbuf, &pktbuf[total_packet_length], pktbuf_ofs - total_packet_length);
			pktbuf_ofs -= total_packet_length;
			return;
		}

		// process full packet
		process_packet(pktbuf, total_packet_length);



		memmove(pktbuf, &pktbuf[total_packet_length], pktbuf_ofs - total_packet_length);
		pktbuf_ofs -= total_packet_length;
	}
}

/*
  process a packet from a servo
 */
void DynamixelProtocol::process_packet(const uint8_t *pkt, uint8_t length)
{
	uint8_t id = pkt[PKT_ID];

	if (id > 16 || id < 1) {
		// discard packets from servos beyond max or min. Note that we
		// don't allow servo 0, to make mapping to SERVOn_* parameters
		// easier
		return;
	}

	uint16_t id_mask = (1U << (id - 1));

	if (!(id_mask & servo_mask)) {
		// mark the servo as present
		servo_mask |= id_mask;
		PX4_INFO("Dynamixel found - ID: %u", id);
	}
}

void DynamixelProtocol::set_setpoints(int i, int32_t val, uint32_t led, uint32_t mode)
{

	if (mode != op_mode) {
		op_mode = mode;
		configure_servos();

	} else {
		op_mode = mode;
	}

	if (i == 254) {
		val_sp[16] = val;
		led_sp[16] = led;
		broadcast = true;

	} else if (!(i > 16 || i < 1)) {
		val_sp[i - 1] = val;
		led_sp[i - 1] = led;
		broadcast = false;
	}
}

bool DynamixelProtocol::update()
{
	hrt_abstime now = hrt_absolute_time();

	PX4_INFO("Entered on update dynamixel");

	if (last_send_us != 0 && now - last_send_us < delay_time_us) {
		// waiting for last send to complete
		// PX4_INFO("Waiting on last update...");
		return false;

	}

	if (detection_count < DETECT_SERVO_COUNT) {
		detection_count++;
		PX4_INFO("Detected different servo count");
		detect_servos();
	}

	// if (servo_mask == 0) {
	// 	PX4_INFO("Detected mask 0");
	// 	return false;
	// }

	if (configured_servos < CONFIGURE_SERVO_COUNT) {
		configured_servos++;
		last_send_us = now;
		configure_servos();
		PX4_INFO("Configured servos");
		return false;
	}

	last_send_us = now;
	delay_time_us = 0;
	bool flag = false;

	// loop for all 16 channels
	for (uint8_t i = 0; i < 2; i++) {
		// if (((1U << i) & servo_mask) == 0) {
		// 	PX4_INFO("true for servo mask 0");
		// 	continue;
		// }

		PX4_INFO("in servo loop");

		if (broadcast) {

			switch (op_mode) {
			case OPMODE_CURR_CONTROL:
				send_command(BROADCAST_ID, Reg::GOAL_CURRENT, val_sp[16]);
				//Applying high current to the motor for long period of time might damage the motor
				break;

			case OPMODE_EXT_POS_CONTROL:
				send_command(BROADCAST_ID, Reg::GOAL_POSITION, val_sp[16]);
				break;

			case OPMODE_POS_CONTROL:
				send_command(BROADCAST_ID, Reg::GOAL_POSITION, val_sp[16]);
				break;

			case OPMODE_VEL_CONTROL:
				send_command(BROADCAST_ID, Reg::GOAL_VELOCITY, val_sp[16]);
				break;
			}

			send_command(BROADCAST_ID, Reg::LED, led_sp[16]);
			continue;
			// PX4_INFO("Command broadcast sent!");

		} else {
			switch (op_mode) {
			case OPMODE_CURR_CONTROL:
				send_command(i + 1, Reg::GOAL_CURRENT, val_sp[i]);
				//Applying high current to the motor for long period of time might damage the motor
				break;

			case OPMODE_EXT_POS_CONTROL:
				send_command(i + 1, Reg::GOAL_POSITION, val_sp[i]);
				break;

			case OPMODE_POS_CONTROL:
				send_command(i + 1, Reg::GOAL_POSITION, val_sp[i]);
				break;

			case OPMODE_VEL_CONTROL:
				send_command(i + 1, Reg::GOAL_VELOCITY, val_sp[i]);
				break;
			}

			send_command(i + 1, Reg::LED, led_sp[i]);
			// PX4_INFO("Command sent!");
		}

		flag = true;
	}

	return flag;
}

//MX series Control Table
uint8_t DynamixelProtocol::get_size(Reg reg)
{
	uint8_t size = 1;

	switch (reg) {

	case Reg::MODEL_NUMBER : size = 2; break;

	case Reg::MODEL_INFORMATION : size = 4; break;

	case Reg::VERSION_OF_FIRMWARE : size = 1; break;

	case Reg::ID : size = 1; break;

	case Reg::BAUDRATE : size = 1; break;

	case Reg::RETURN_DELAY_TIME : size = 1; break;

	case Reg::DRIVE_MODE : size = 1; break;

	case Reg::OPERATING_MODE : size = 1; break;

	case Reg::SECONDARY_ID : size = 1; break;

	case Reg::PROTOCOL_VERSION : size = 1; break;

	case Reg::HOMING_OFFSET : size = 4; break;

	case Reg::MOVING_THRESHOLD : size = 4; break;

	case Reg::TEMPERATURE_LIMIT : size = 1; break;

	case Reg::MAX_VOLTAGE_LIMIT : size = 2; break;

	case Reg::MIN_VOLTAGE_LIMIT : size = 2; break;

	case Reg::PWM_LIMIT : size = 2; break;

	case Reg::CURRENT_LIMIT : size = 2; break;

	case Reg::ACCELERATION_LIMIT : size = 4; break;

	case Reg::VELOCITY_LIMIT : size = 4; break;

	case Reg::MAX_POSITION_LIMIT : size = 4; break;

	case Reg::MIN_POSITION_LIMIT : size = 4; break;

	case Reg::SHUTDOWN : size = 4; break;

	case Reg::TORQUE_ENABLE : size = 1; break;

	case Reg::LED : size = 1; break;

	case Reg::STATUS_RETURN_LEVEL : size = 1; break;

	case Reg::REGISTERED_INSTRUCTION : size = 1; break;

	case Reg::HARDWARE_ERROR_STATUS : size = 1; break;

	case Reg::VELOCITY_I_GAIN : size = 2; break;

	case Reg::VELOCITY_P_GAIN : size = 2; break;

	case Reg::POSITION_D_GAIN : size = 2; break;

	case Reg::POSITION_I_GAIN : size = 2; break;

	case Reg::POSITION_P_GAIN : size = 2; break;

	case Reg::FEEDFORWARD_ACCELERATION_GAIN : size = 2; break;

	case Reg::FEEDFORWARD_VELOCITY_GAIN : size = 2; break;

	case Reg::BUS_WATCHDOG : size = 1; break;

	case Reg::GOAL_PWM : size = 2; break;

	case Reg::GOAL_CURRENT : size = 2; break;

	case Reg::GOAL_VELOCITY : size = 4; break;

	case Reg::PROFILE_ACCELERATION : size = 4; break;

	case Reg::PROFILE_VELOCITY : size = 4; break;

	case Reg::GOAL_POSITION : size = 4; break;

	case Reg::REALTIME_TICK : size = 2; break;

	case Reg::MOVING : size = 1; break;

	case Reg::MOVING_STATUS  : size = 1; break;

	case Reg::PRESENT_PWM : size = 2; break;

	case Reg::PRESENT_CURRENT :  size = 2; break;

	case Reg::PRESENT_VELOCITY : size = 4; break;

	case Reg::PRESENT_POSITION : size = 4; break;

	case Reg::VELOCITY_TRAJECTORY : size = 4; break;

	case Reg::POSITION_TRAJECTORY : size = 4; break;

	case Reg::PRESENT_INPUT_VOLTAGE : size = 2; break;

	case Reg::PRESENT_TEMPERATURE : size = 1; break;

	case Reg::EXTERNAL_PORT_DATA_1 : size = 2; break;

	case Reg::EXTERNAL_PORT_DATA_2 : size = 2; break;

	case Reg::EXTERNAL_PORT_DATA_3 : size = 2; break;

	case Reg::INDIRECT_ADDR_1 : size = 2; break;

	case Reg::INDIRECT_DATA_1 : size = 1; break;

	case Reg::INDIRECT_ADDR_29 : size = 2; break;

	case Reg::INDIRECT_DATA_29 : size = 1; break;

	}

	return size;
}

// CRC-16 (IBM/ANSI)
// Polynomial : x16 + x15 + x2 + 1 (polynomial representation : 0x8005)
// Initial Value : 0
uint16_t DynamixelProtocol::updateCRC(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size)
{
	uint16_t i;
	static const uint16_t crc_table[256] = {0x0000,
						0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
						0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027,
						0x0022, 0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D,
						0x8077, 0x0072, 0x0050, 0x8055, 0x805F, 0x005A, 0x804B,
						0x004E, 0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC, 0x80C9,
						0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0, 0x80F5, 0x80FF,
						0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1, 0x00A0, 0x80A5,
						0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1, 0x8093,
						0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
						0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197,
						0x0192, 0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE,
						0x01A4, 0x81A1, 0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB,
						0x01FE, 0x01F4, 0x81F1, 0x81D3, 0x01D6, 0x01DC, 0x81D9,
						0x01C8, 0x81CD, 0x81C7, 0x01C2, 0x0140, 0x8145, 0x814F,
						0x014A, 0x815B, 0x015E, 0x0154, 0x8151, 0x8173, 0x0176,
						0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162, 0x8123,
						0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
						0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104,
						0x8101, 0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D,
						0x8317, 0x0312, 0x0330, 0x8335, 0x833F, 0x033A, 0x832B,
						0x032E, 0x0324, 0x8321, 0x0360, 0x8365, 0x836F, 0x036A,
						0x837B, 0x037E, 0x0374, 0x8371, 0x8353, 0x0356, 0x035C,
						0x8359, 0x0348, 0x834D, 0x8347, 0x0342, 0x03C0, 0x83C5,
						0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1, 0x83F3,
						0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
						0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7,
						0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E,
						0x0384, 0x8381, 0x0280, 0x8285, 0x828F, 0x028A, 0x829B,
						0x029E, 0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC, 0x82B9,
						0x02A8, 0x82AD, 0x82A7, 0x02A2, 0x82E3, 0x02E6, 0x02EC,
						0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2, 0x02D0, 0x82D5,
						0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1, 0x8243,
						0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
						0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264,
						0x8261, 0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E,
						0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219, 0x0208,
						0x820D, 0x8207, 0x0202
					       };

	for (uint16_t j = 0; j < data_blk_size; j++) {
		i = ((uint16_t)(crc_accum >> 8) ^ *data_blk_ptr++) & 0xFF;
		crc_accum = (crc_accum << 8) ^ crc_table[i];
	}

	return crc_accum;
}
