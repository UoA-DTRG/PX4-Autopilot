#include "loadcell.h"
#include <termios.h>
#include <string.h>
#include <unistd.h>
#include <stdio.h>
// #include <mavlink.h>

#include <parameters/param.h>
// #include <mavlink/mavlink_log.h>
// #include <mavlink/mavlink_main.h>

// PARAM_DEFINE_INT32(LOADCELL_ENABLE,1)
// Add MAVLink headers for sending data


LoadCell::LoadCell() : Device(nullptr)
{

}



LoadCell::LoadCell(const char *port) : Device(nullptr)
{
    strncpy(_port, port, sizeof(_port) - 1);
    _port[sizeof(_port) - 1] = '\0';
}

LoadCell::~LoadCell()
{
    if (_uart_fd >= 0) {
        ::close(_uart_fd);
    }
}

int LoadCell::init()
{
    // Open the UART port (GPS 2 is typically /dev/ttyS3)
    _uart_fd = ::open(_port, O_RDWR | O_NOCTTY);

    if (_uart_fd < 0) {
        PX4_ERR("Failed to open UART port %s", _port);
        return -1;
    }

    // Configure UART (baud rate, parity, etc.)
    struct termios uart_config;
    tcgetattr(_uart_fd, &uart_config);
    cfsetspeed(&uart_config, B115200); // Set baud rate to 115200
    uart_config.c_cflag &= ~PARENB;   // No parity
    uart_config.c_cflag &= ~CSTOPB;   // 1 stop bit
    uart_config.c_cflag &= ~CSIZE;
    uart_config.c_cflag |= CS8;       // 8 data bits
    tcsetattr(_uart_fd, TCSANOW, &uart_config);

    char response[100];
    send_command("AT+SFWV=?\r\n", response, sizeof(response)); // Read firmware version
    px4_usleep(100000); // Wait 100 ms for response
    PX4_INFO("Firmware version: %s", response);

    send_command("AT+SMPF=200\r\n", response, sizeof(response)); // Set sampling rate to 200 Hz
    px4_usleep(100000); // Wait 100 ms for response
    PX4_INFO("Sampling rate set: %s", response);

    send_command("AT+DCPCU=N\r\n", response, sizeof(response)); // Set calculation unit to Newtons
    px4_usleep(100000); // Wait 100 ms for response
    PX4_INFO("Calculation unit set: %s", response);

    PX4_INFO("Load cell driver initialized on port %s", _port);
    return PX4_OK;
}

bool LoadCell::parse_loadcell_data(const char *response, float &fx, float &fy, float &fz, float &tx, float &ty, float &tz)
{
    // Example: Parse data in the format "Fx=1.23,Fy=4.56,Fz=7.89,Tx=0.12,Ty=3.45,Tz=6.78"
    if (sscanf(response, "Fx=%f,Fy=%f,Fz=%f,Tx=%f,Ty=%f,Tz=%f", &fx, &fy, &fz, &tx, &ty, &tz) == 6) {
        return true;
    }
    return false;
}

// void LoadCell::send_loadcell_data_mavlink(float force_x, float force_y, float force_z, float torque_x, float torque_y, float torque_z)
// {
//     mavlink_message_t msg;
//     mavlink_msg_loadcell_data_pack(
//         mavlink_system.sysid,  // System ID
//         mavlink_system.compid, // Component ID
//         &msg,                  // MAVLink message
//         force_x,               // Force X
//         force_y,               // Force Y
//         force_z,               // Force Z
//         torque_x,              // Torque X
//         torque_y,              // Torque Y
//         torque_z               // Torque Z
//     );

//     // Send the message over the MAVLink channel
//     mavlink_send_buffer(MAVLINK_COMM_0, &msg);
// }

int LoadCell::send_command(const char *command, char *response, int response_size)
{
    // Send command
    int bytes_written = ::write(_uart_fd, command, strlen(command));
    if (bytes_written < 0) {
        PX4_ERR("Failed to send command");
        return -1;
    }

    // Read response (if needed)
    if (response && response_size > 0) {
        int bytes_read = ::read(_uart_fd, response, response_size - 1);
        if (bytes_read < 0) {
            PX4_ERR("Failed to read response");
            return -1;
        }
        response[bytes_read] = '\0'; // Null-terminate the response
        return bytes_read;
    }

    return bytes_written;
}

int LoadCell::read_data()
{
    char response[100];
    float fx, fy, fz, tx, ty, tz;

    // Send command to request data
    const char *command = "AT+GETDATA=?\r\n";
    int bytes_read = send_command(command, response, sizeof(response));

    if (bytes_read > 0) {
        // Parse the response
        if (parse_loadcell_data(response, fx, fy, fz, tx, ty, tz)) {
            // Publish data to uORB
            loadcell_data_s loadcell_data = {};
            loadcell_data.timestamp = hrt_absolute_time();
            loadcell_data.force_x = fx;
            loadcell_data.force_y = fy;
            loadcell_data.force_z = fz;
            loadcell_data.torque_x = tx;
            loadcell_data.torque_y = ty;
            loadcell_data.torque_z = tz;

            orb_advert_t loadcell_pub = orb_advertise(ORB_ID(loadcell_data), &loadcell_data);
            orb_publish(ORB_ID(loadcell_data), loadcell_pub, &loadcell_data);

            // Send data via MAVLink
            // send_loadcell_data_mavlink(fx, fy, fz, tx, ty, tz);

            // Debug prints
            PX4_INFO("Received data: %s", response);
            // PX4_INFO("Parsed data: Fx=%.2f, Fy=%.2f, Fz=%.2f, Tx=%.2f, Ty=%.2f, Tz=%.2f", fx, fy, fz, tx, ty, tz);
        } else {
            PX4_ERR("Failed to parse load cell data");
        }
    }

    return bytes_read;
}



int LoadCell::task_spawn(int argc, char *argv[])
{
	LoadCell *instance = new LoadCell();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}


int LoadCell::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}


int LoadCell::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
LOAD CELL.

)DESCR_STR");



	return 0;
}


extern "C" __EXPORT int loadcell_main(int argc, char *argv[])
{
  return LoadCell::main(argc, argv);
}
