#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <drivers/device/device.h>
#include <uORB/uORB.h>
#include <uORB/topics/loadcell_data.h>

class LoadCell : public device::Device, public ModuleBase<LoadCell>
{
public:
    LoadCell();
    LoadCell(const char *port);
    virtual ~LoadCell();

    /** @see ModuleBase */
    static int task_spawn(int argc, char *argv[]);

    /** @see ModuleBase */
    static int custom_command(int argc, char *argv[]);

    /** @see ModuleBase */
    static int print_usage(const char *reason = nullptr);

    virtual int init();
    int read_data();

private:
    int _uart_fd{-1};
    char _port[20] = "/dev/ttyS5";  // Default to GPS2 port

    int send_command(const char *command, char *response, int response_size);
    bool parse_loadcell_data(const char *response, float &fx, float &fy, float &fz, float &tx, float &ty, float &tz);

    orb_advert_t _loadcell_data_pub{nullptr};
    loadcell_data_s _loadcell_data{};
};
