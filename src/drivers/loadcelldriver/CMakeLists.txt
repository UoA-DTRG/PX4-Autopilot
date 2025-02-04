px4_add_module(
    MODULE drivers__loadcelldriver
    MAIN loadcelldriver
    SRCS
        loadcell_driver.cpp
    DEPENDS
        uORB
        px4_work_queue
    )
