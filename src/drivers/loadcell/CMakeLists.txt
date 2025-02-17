px4_add_module(
    MODULE drivers__loadcell
    MAIN loadcell
    STACK_MAIN 2048
    SRCS
        loadcell.cpp
    DEPENDS
        px4_work_queue
        )
