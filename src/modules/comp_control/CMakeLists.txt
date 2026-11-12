px4_add_module(
 MODULE modules__comp_control
 MAIN comp_control
 SRCS CompControl.cpp
 DEPENDS px4_work_queue
)
