idf_component_register(
    SRCS
        "robot.c"
    INCLUDE_DIRS
        "."
    REQUIRES imu position_controller odometry motor_controller telemetry config
)