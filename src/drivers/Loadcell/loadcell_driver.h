#ifndef __LOADCELL_DRIVER_H__
#define __LOADCELL_DRIVER_H__

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <drivers/device/device.h>
#include <uORB/uORB.h>
#include <uORB/topics/loadcell_data.h>

class LoadCellDriver : public device::Device
{
public:
    LoadCellDriver(const char *port);
    virtual ~LoadCellDriver();

    virtual int init();
    virtual int read_data();

private:
    int _uart_fd;
    char _port[20];

    int send_command(const char *command, char *response, int response_size);
    bool parse_loadcell_data(const char *response, float &fx, float &fy, float &fz, float &tx, float &ty, float &tz);
};

#endif // __LOADCELL_DRIVER_H__
