px4_add_module(
    MODULE loadcell_driver
    MAIN loadcell_driver
    SRCS
        loadcell_driver.cpp
    DEPENDS
        platforms__common
        uORB
    )
