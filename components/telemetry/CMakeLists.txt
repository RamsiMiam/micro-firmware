idf_component_register(
    SRCS
        "telemetry.c"
    INCLUDE_DIRS
        "."
    REQUIRES odometry motor motor_controller imu esp_wifi esp_event esp_netif nvs_flash config
)