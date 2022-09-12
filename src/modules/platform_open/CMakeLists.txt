px4_add_module(
    MODULE modules__platform_open
    MAIN platform_open
    STACK_MAIN 2000
    SRCS
        platform_open.c
    DEPENDS
    )
