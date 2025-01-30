# CMakeLists.txt for the load cell driver

# Declare the load cell driver module
px4_add_module(
    MODULE loadcell_driver
    MAIN loadcell_driver
    SRCS
        loadcell_driver.cpp
    DEPENDS
        platforms__common
        uORB
    )
