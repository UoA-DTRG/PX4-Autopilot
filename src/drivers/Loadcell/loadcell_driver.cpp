#include "loadcell_driver.h"
#include <termios.h>
#include <string.h>
#include <unistd.h>
#include <stdio.h>

LoadCellDriver::LoadCellDriver(const char *port) : Device(nullptr)
{
    strncpy(_port, port, sizeof(_port) - 1);
    _port[sizeof(_port) - 1] = '\0';
}

LoadCellDriver::~LoadCellDriver()
{
    if (_uart_fd >= 0) {
        ::close(_uart_fd);
    }
}

int LoadCellDriver::init()
{
    _uart_fd = ::open(_port, O_RDWR | O_NOCTTY);

    if (_uart_fd < 0) {
        PX4_ERR("Failed to open UART port %s", _port);
        return -1;
    }

    // Configure UART (baud rate, parity, etc.)
    struct termios uart_config;
    tcgetattr(_uart_fd, &uart_config);
    cfsetspeed(&uart_config, B115200); // Set baud rate
    uart_config.c_cflag &= ~PARENB;   // No parity
    uart_config.c_cflag &= ~CSTOPB;   // 1 stop bit
    uart_config.c_cflag &= ~CSIZE;
    uart_config.c_cflag |= CS8;       // 8 data bits
    tcsetattr(_uart_fd, TCSANOW, &uart_config);

    PX4_INFO("Load cell driver initialized on port %s", _port);
    return PX4_OK;
}

int LoadCellDriver::send_command(const char *command, char *response, int response_size)
{
    // Send command
    int bytes_written = ::write(_uart_fd, command, strlen(command));
    if (bytes_written < 0) {
        PX4_ERR("Failed to send command");
        return -1;
    }

    // Read response
    int bytes_read = ::read(_uart_fd, response, response_size - 1);
    if (bytes_read < 0) {
        PX4_ERR("Failed to read response");
        return -1;
    }

    response[bytes_read] = '\0'; // Null-terminate the response
    return bytes_read;
}

bool LoadCellDriver::parse_loadcell_data(const char *response, float &fx, float &fy, float &fz, float &tx, float &ty, float &tz)
{
    // Example response: Fx=1.23,Fy=4.56,Fz=7.89,Tx=0.12,Ty=3.45,Tz=6.78
    if (sscanf(response, "Fx=%f,Fy=%f,Fz=%f,Tx=%f,Ty=%f,Tz=%f", &fx, &fy, &fz, &tx, &ty, &tz) == 6) {
        return true;
    }
    return false;
}

int LoadCellDriver::read_data()
{
    char response[100];
    float fx, fy, fz, tx, ty, tz;

    // Send command to request data (replace with the actual command)
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

            PX4_INFO("Load cell data: Fx=%.2f, Fy=%.2f, Fz=%.2f, Tx=%.2f, Ty=%.2f, Tz=%.2f", fx, fy, fz, tx, ty, tz);
        } else {
            PX4_ERR("Failed to parse load cell data");
        }
    }

    return bytes_read;
}

extern "C" __EXPORT int loadcell_driver_main(int argc, char *argv[]);

int loadcell_driver_main(int argc, char *argv[])
{
    if (argc < 2) {
        PX4_ERR("Usage: loadcell_driver start -d <uart_port>");
        return -1;
    }

    const char *port = argv[2];
    LoadCellDriver driver(port);

    if (driver.init() != PX4_OK) {
        PX4_ERR("Driver initialization failed");
        return -1;
    }

    while (true) {
        driver.read_data();
        px4_usleep(100000); // Read data at 10 Hz (adjust as needed)
    }

    return 0;
}
