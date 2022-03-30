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
	detection_count = 0;

	last_send_us = 0;
	delay_time_us = 0;
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

	write(uart, txpacket, total_packet_length);

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
}

/*
broadcast configure all servos
 */
void DynamixelProtocol::configure_servos(void)
{
	// disable torque control
	send_command(BROADCAST_ID, REG_TORQUE_ENABLE, 0, 1);

	// disable replies unless we read
	send_command(BROADCAST_ID, REG_STATUS_RETURN, STATUS_RETURN_READ, 1);

	// use position control mode
	send_command(BROADCAST_ID, REG_OPERATING_MODE, OPMODE_POS_CONTROL, 1);

	// enable torque control
	send_command(BROADCAST_ID, REG_TORQUE_ENABLE, 1, 1);
}


/*
  send a command to a single servo, changing a register value
 */
void DynamixelProtocol::send_command(uint8_t id, uint16_t reg, uint32_t value, uint8_t len)
{
	uint8_t txpacket[16] {};

	txpacket[PKT_ID] = id;
	txpacket[PKT_LENGTH_L] = 5 + len;
	txpacket[PKT_LENGTH_H] = 0;
	txpacket[PKT_INSTRUCTION] = INST_WRITE;
	txpacket[PKT_INSTRUCTION + 1] = DXL_LOBYTE(reg);
	txpacket[PKT_INSTRUCTION + 2] = DXL_HIBYTE(reg);
	//memcpy(&txpacket[PKT_INSTRUCTION + 3], &value, MIN(len, 4));
	memcpy(&txpacket[PKT_INSTRUCTION + 3], &value, ((len < 4) ? len : 4));

	send_packet(txpacket);
}

/*
	read response bytes
 */
void DynamixelProtocol::read_bytes(uint32_t n)
{

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
	}

	// discard bad leading data. This should be rare
	while (pktbuf_ofs >= 4 &&
	       (pktbuf[0] != 0xFF || pktbuf[1] != 0xFF || pktbuf[2] != 0xFD || pktbuf[3] != 0x00)) {
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

	// check CRC
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
	}
}


void DynamixelProtocol::update(uint32_t n, unsigned short int pos, unsigned short int led)
{
	// 	// disable torque control
	// send_command(BROADCAST_ID, REG_TORQUE_ENABLE, 0, 1);

	// // // disable replies unless we read
	// send_command(BROADCAST_ID, REG_STATUS_RETURN, STATUS_RETURN_READ, 1);

	// // // use position control mode
	// send_command(BROADCAST_ID, REG_OPERATING_MODE, OPMODE_POS_CONTROL, 1);

	//send_command(1, REG_TORQUE_ENABLE, 1, 1);


	// enable torque control


	// // enable torque control
	// configure_servos();
	//send_command(1, REG_GOAL_POSITION, (uint32_t) 512, 4);


	// if (!initialised) {
	// 	initialised = true;
	// 	last_send_us = hrt_absolute_time();
	// 	return;
	// }

	read_bytes(n);

	hrt_abstime now = hrt_absolute_time();

	if (last_send_us != 0 && now - last_send_us < delay_time_us) {
		// waiting for last send to complete
		return;
	}

	detect_servos();
	send_command(BROADCAST_ID, LED_ENABLE, led, 1);

	send_command(BROADCAST_ID, REG_TORQUE_ENABLE, 1, 1);

	send_command(BROADCAST_ID, REG_GOAL_POSITION, (uint32_t) pos, 4);

	// if (detection_count < DETECT_SERVO_COUNT) {
	// 	detection_count++;
	// 	detect_servos();
	// }

	// if (servo_mask == 0) {
	// 	return;
	// }

	// if (configured_servos < CONFIGURE_SERVO_COUNT) {
	// 	configured_servos++;
	// 	last_send_us = now;
	// 	configure_servos();
	// 	return;
	// }

	last_send_us = now;
	delay_time_us = 0;

	// // loop for all 16 channels
	// for (uint8_t i = 0; i < 16; i++) {
	// 	if (((1U << i) & servo_mask) == 0) {
	// 		continue;
	// 	}

	// 	// SRV_Channel *c = SRV_Channels::srv_channel(i);

	// 	// if (c == nullptr) {
	// 	// 	continue;
	// 	// }

	// 	// const uint16_t pwm = c->get_output_pwm();
	// 	// const uint16_t min = c->get_output_min();
	// 	// const uint16_t max = c->get_output_max();
	// 	// float v = float(pwm - min) / (max - min);
	// 	float v = 0.5f;
	// 	uint32_t value = pos_min + v * (pos_max - pos_min);
	// 	send_command(i + 1, REG_GOAL_POSITION, value, 4);
	// }


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
