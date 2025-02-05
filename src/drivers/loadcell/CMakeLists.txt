px4_add_module(
    MODULE drivers__loadcell
    MAIN loadcell
    STACK_MAIN 2000
    SRCS
        loadcell.cpp
        loadcell.h
    DEPENDS
        drivers__device
        uORB
        px4_work_queue
)
