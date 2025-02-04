// #ifndef __LOADCELL_DRIVER_H__
// #define __LOADCELL_DRIVER_H__

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <drivers/device/device.h>
#include <uORB/uORB.h>
#include <uORB/topics/loadcell_data.h>

class LoadCellDriver : public device::Device,public ModuleBase<LoadCellDriver>
{
public:

    LoadCellDriver();
    LoadCellDriver(const char *port);
    virtual ~LoadCellDriver();
    /** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);
    /** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);
    virtual int init();
    int read_data();

private:
    int _uart_fd;
    char _port[20];

    int send_command(const char *command, char *response, int response_size);
    bool parse_loadcell_data(const char *response, float &fx, float &fy, float &fz, float &tx, float &ty, float &tz);
    // void send_loadcell_data_mavlink(float force_x, float force_y, float force_z, float torque_x, float torque_y, float torque_z);
};

// #endif // __LOADCELL_DRIVER_H__
