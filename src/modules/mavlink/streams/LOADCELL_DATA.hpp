/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#ifndef WIND_COV_HPP
#define WIND_COV_HPP

#include <uORB/topics/loadcell_data.h>


class MavlinkStreamLoadcell : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamLoadcell(mavlink); }

	static constexpr const char *get_name_static() { return "LOADCELL_DATA"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_LOADCELL_DATA; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _loadcell_data_sub.advertised() ? MAVLINK_MSG_ID_LOADCELL_DATA_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamLoadcell(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _loadcell_data_sub{ORB_ID(loadcell_data)};

	bool send() override
	{
		loadcell_data_s loadcell_data;

		if (_loadcell_data_sub.update(&loadcell_data)) {
			mavlink_loadcell_data_t msg{};

			msg.time_usec = loadcell_data.timestamp;

			msg.force_x=loadcell_data.force_x
			msg.force_y=loadcell_data.force_y
			msg.force_z=loadcell_data.force_z


			msg.torque_x=loadcell_data.torque_x
			msg.torque_y=loadcell_data.torque_y
			msg.torque_z=loadcell_data.torque_z



			mavlink_msg_wind_cov_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // LOADCELL_DATA
