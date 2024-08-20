
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

/**
 * DTRG OFFBOARD MAVLink stream
 *
 * @author Jaap Skinner <jski306@aucklanduni.ac.nz>
 */

#ifndef DTRG_OFFBOARD_HPP
#define DTRG_OFFBOARD_HPP


#include <uORB/topics/dtrg_custom.h>


class MavlinkStreamDTRGOffboard : public MavlinkStream
{
public:
    static MavlinkStream *new_instance(Mavlink *mavlink)
    {
        return new MavlinkStreamBMavlinkStreamDTRGOffboard(mavlink);
    }
    const char *get_name() const
    {
        return MavlinkMavlinkStreamDTRGOffboard::get_name_static();
    }
    static const char *get_name_static()
    {
        return "DTRG_OFFBOARD";
    }
    static uint16_t get_id_static()
    {
        return MAVLINK_MSG_ID_DTRG_OFFBOARD;
    }
    uint16_t get_id()
    {
        return get_id_static();
    }
    unsigned get_size()
    {
        return MAVLINK_MSG_ID_DTRG_OFFBOARD_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
    }

private:
    uORB::Subscription _dtrg_offboard_sub{ORB_ID::dtrg_custom};

    /* do not allow top copying this class */
    MavlinkStreamDTRGOffboard(MavlinkStreamDTRGOffboard &);
    MavlinkStreamDTRGOffboard& operator = (const MavlinkStreamDTRGOffboard &);

protected:
    explicit MavlinkStreamDTRGOffboard(Mavlink *mavlink) : MavlinkStream(mavlink)
    {}

	bool send() override
	{
		bool updated = false;
			offboard_sp_s offboard_sp;

			if (_dtrg_offboard_sub.update(&offboard_sp)) {
                		// mavlink_battery_status_demo_t is the MAVLink message object
				mavlink_dtrg_offboard_t offboard_msg{};

				offboard_msg.offboard_sp = offboard_sp.offboard_sp;

				mavlink_msg_dtrg_offboard_send_struct(_mavlink->get_channel(), &offboard_msg);
				updated = true;
			}

		return updated;
	}

};

#endif // DTRG_OFFBOARD_HPP
