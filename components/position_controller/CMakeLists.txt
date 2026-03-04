idf_component_register(
    SRCS
        "position_controller.c"
    INCLUDE_DIRS
        "."
    REQUIRES odometry motor_controller config
)