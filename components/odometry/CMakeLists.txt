idf_component_register(
    SRCS
        "odometry.c"
    INCLUDE_DIRS
        "."
    REQUIRES motor motor_controller imu config
)