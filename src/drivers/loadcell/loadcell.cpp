#include "loadcell.h"
#include <termios.h>
#include <string.h>
#include <unistd.h>
#include <stdio.h>

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
    PX4_INFO("Initializing loadcell driver...");

    // Open UART port (GPS2 is typically /dev/ttyS3)
    _uart_fd = ::open(_port, O_RDWR | O_NOCTTY);

    if (_uart_fd < 0) {
        PX4_ERR("Failed to open UART port %s, errno: %d", _port, errno);
        return -1;
    }

    PX4_INFO("UART port opened successfully");

    // Configure UART
    struct termios uart_config;
    tcgetattr(_uart_fd, &uart_config);
    cfsetspeed(&uart_config, B115200); // Set baud rate to 115200
    uart_config.c_cflag &= ~PARENB;   // No parity
    uart_config.c_cflag &= ~CSTOPB;   // 1 stop bit
    uart_config.c_cflag &= ~CSIZE;
    uart_config.c_cflag |= CS8;       // 8 data bits
    uart_config.c_cflag |= CLOCAL | CREAD;  // Enable receiver, ignore modem controls

    // Set up for non-canonical mode
    uart_config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    uart_config.c_oflag &= ~OPOST;

    // Fetch bytes as they become available
    uart_config.c_cc[VTIME] = 10;     // Wait up to 1 sec (10 deciseconds)
    uart_config.c_cc[VMIN] = 0;       // No minimum number of bytes

    tcsetattr(_uart_fd, TCSANOW, &uart_config);
    tcflush(_uart_fd, TCIOFLUSH);     // Flush both input and output

    PX4_INFO("UART configured successfully");

    // Initial delay
    px4_usleep(1000000);  // 1 second initial delay

    char response[100];
    int attempt = 0;
    const int max_attempts = 3;

    while (attempt < max_attempts) {
        PX4_INFO("Initialization attempt %d of %d", attempt + 1, max_attempts);

        // Try to enter command mode
        PX4_INFO("Sending +++ to enter command mode");
        // Send +++ with proper type casting
        const char cmd[] = "+++";
        ::write(_uart_fd, static_cast<const void*>(cmd), 3);
        px4_usleep(1000000);  // Wait 1 second

        // Read firmware version
        PX4_INFO("Reading firmware version...");
        if (send_command("AT+SFWV=?\r\n", response, sizeof(response)) > 0) {
            if (strncmp(response, "V", 1) == 0) {
                PX4_INFO("Valid firmware version received: %s", response);
                break;
            }
        }

        px4_usleep(500000);  // 500ms between attempts
        attempt++;
    }

    if (attempt == max_attempts) {
        PX4_ERR("Failed to initialize loadcell after %d attempts", max_attempts);
        return -1;
    }

    // Set sampling rate
    PX4_INFO("Setting sampling rate...");
    if (send_command("AT+SMPF=200\r\n", response, sizeof(response)) > 0) {
        if (strstr(response, "OK") == nullptr) {
            PX4_ERR("Failed to set sampling rate");
            return -1;
        }
    }
    px4_usleep(200000);

    // Set calculation unit
    PX4_INFO("Setting calculation unit...");
    if (send_command("AT+DCPCU=N\r\n", response, sizeof(response)) > 0) {
        if (strstr(response, "OK") == nullptr) {
            PX4_ERR("Failed to set calculation unit");
            return -1;
        }
    }
    px4_usleep(200000);

    // Initialize uORB publication
    _loadcell_data_pub = orb_advertise(ORB_ID(loadcell_data), &_loadcell_data);

    PX4_INFO("Loadcell initialization complete");

    return PX4_OK;
}

int LoadCell::send_command(const char *command, char *response, int response_size)
{
    // Flush any existing data
    tcflush(_uart_fd, TCIOFLUSH);

    // Print command being sent
    PX4_INFO("Sending command: %s", command);

    // Send command
    int bytes_written = ::write(_uart_fd, command, strlen(command));
    if (bytes_written < 0) {
        PX4_ERR("Failed to send command: errno %d", errno);
        return -1;
    }

    // Wait for data to be transmitted
    tcdrain(_uart_fd);

    // Wait a bit for the device to process
    px4_usleep(200000);  // 200ms

    // Read response
    if (response && response_size > 0) {
        int total_read = 0;
        int retry_count = 0;
        const int max_retries = 5;

        while (total_read < (response_size - 1) && retry_count < max_retries) {
            int bytes_read = ::read(_uart_fd, response + total_read, response_size - 1 - total_read);

            if (bytes_read > 0) {
                total_read += bytes_read;
                // Check if we've received the complete response (ending in \r\n)
                if (total_read >= 2 && response[total_read-2] == '\r' && response[total_read-1] == '\n') {
                    break;
                }
            } else if (bytes_read == 0) {
                // No more data
                retry_count++;
                px4_usleep(100000);  // Wait 100ms before retry
            } else {
                PX4_ERR("Read error: errno %d", errno);
                return -1;
            }
        }

        if (total_read > 0) {
            response[total_read] = '\0';  // Null terminate
            PX4_INFO("Received response: %s", response);
            return total_read;
        } else {
            PX4_ERR("No response received after %d retries", max_retries);
            return -1;
        }
    }

    return bytes_written;
}

bool LoadCell::parse_loadcell_data(const char *response, float &fx, float &fy, float &fz, float &tx, float &ty, float &tz)
{
    // Example: Parse data in the format "Fx=1.23,Fy=4.56,Fz=7.89,Tx=0.12,Ty=3.45,Tz=6.78"
    if (sscanf(response, "Fx=%f,Fy=%f,Fz=%f,Tx=%f,Ty=%f,Tz=%f", &fx, &fy, &fz, &tx, &ty, &tz) == 6) {
        return true;
    }
    return false;
}

int LoadCell::read_data()
{
    char response[100];
    float fx, fy, fz, tx, ty, tz;

    // Send command to request data
    const char *command = "AT+GETDATA=?\r\n";
    int bytes_read = send_command(command, response, sizeof(response));

    PX4_INFO("Raw response: %s", response);  // Debug print

    if (bytes_read > 0) {
        // Parse the response
        if (parse_loadcell_data(response, fx, fy, fz, tx, ty, tz)) {
            PX4_INFO("Parsed data: FX:%.2f FY:%.2f FZ:%.2f TX:%.2f TY:%.2f TZ:%.2f",
                     (double)fx, (double)fy, (double)fz, (double)tx, (double)ty, (double)tz);

            // Publish data to uORB
            loadcell_data_s loadcell_data{};
            loadcell_data.timestamp = hrt_absolute_time();
            loadcell_data.force_x = fx;
            loadcell_data.force_y = fy;
            loadcell_data.force_z = fz;
            loadcell_data.torque_x = tx;
            loadcell_data.torque_y = ty;
            loadcell_data.torque_z = tz;

            orb_publish(ORB_ID(loadcell_data), _loadcell_data_pub, &loadcell_data);
            return bytes_read;
        } else {
            PX4_ERR("Failed to parse: %s", response);
        }
    } else {
        PX4_ERR("No data received from loadcell");
    }

    return -1;
}

int LoadCell::task_spawn(int argc, char *argv[])
{
    LoadCell *instance = new LoadCell();

    if (instance) {
        _object.store(instance);
        _task_id = task_id_is_work_queue;

        if (instance->init() == PX4_OK) {
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
Load cell driver for M8123B2 6-axis force/torque sensor.
)DESCR_STR");

    return 0;
}

extern "C" __EXPORT int loadcell_main(int argc, char *argv[])
{
    return LoadCell::main(argc, argv);
}
