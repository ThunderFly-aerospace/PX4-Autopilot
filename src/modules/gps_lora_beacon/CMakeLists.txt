px4_add_module(
    MODULE modules__gps_lora_beacon
    MAIN gps_lora_beacon
    STACK_MAIN 2000
    SRCS
        gps_lora_beacon.cpp
    EXTERNAL
    )
