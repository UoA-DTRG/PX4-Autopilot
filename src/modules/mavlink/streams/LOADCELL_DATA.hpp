#ifndef LOADCELL_DATA_HPP
#define LOADCELL_DATA_HPP

#include <uORB/topics/loadcell_data.h>
#include "../mavlink_stream.h"

class MavlinkStreamLoadcellData : public MavlinkStream
{
public:
    static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamLoadcellData(mavlink); }

    static constexpr const char *get_name_static() { return "LOADCELL_DATA"; }
    static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_DEBUG_VECT; }

    const char *get_name() const override { return get_name_static(); }
    uint16_t get_id() override { return get_id_static(); }

    unsigned get_size() override
    {
        return _loadcell_data_sub.advertised() ? (MAVLINK_MSG_ID_DEBUG_VECT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) * 2 : 0;
    }

private:
    explicit MavlinkStreamLoadcellData(Mavlink *mavlink) : MavlinkStream(mavlink) {}

    uORB::Subscription _loadcell_data_sub{ORB_ID(loadcell_data)};

    bool send() override
    {
        loadcell_data_s data;

        if (_loadcell_data_sub.update(&data)) {
            // Send forces
            mavlink_debug_vect_t force_msg{};
            strncpy(force_msg.name, "FRC", sizeof(force_msg.name));
            force_msg.time_usec = data.timestamp;
            force_msg.x = data.force_x;
            force_msg.y = data.force_y;
            force_msg.z = data.force_z;
            mavlink_msg_debug_vect_send_struct(_mavlink->get_channel(), &force_msg);

            // Send torques
            mavlink_debug_vect_t torque_msg{};
            strncpy(torque_msg.name, "TRQ", sizeof(torque_msg.name));
            torque_msg.time_usec = data.timestamp;
            torque_msg.x = data.torque_x;
            torque_msg.y = data.torque_y;
            torque_msg.z = data.torque_z;
            mavlink_msg_debug_vect_send_struct(_mavlink->get_channel(), &torque_msg);

            return true;
        }

        return false;
    }
};

#endif // LOADCELL_DATA_HPP
