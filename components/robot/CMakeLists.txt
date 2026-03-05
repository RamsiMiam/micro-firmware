idf_component_register(
    SRCS
        "robot.c"
    INCLUDE_DIRS
        "."
    REQUIRES position_controller odometry motor_controller config
)