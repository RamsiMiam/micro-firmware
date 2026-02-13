#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TAG_IMU "IMU"

esp_err_t imu_init(int i2c_sda, int i2c_scl, int int_pin);

float get_roll(void);

float get_pitch(void);

float get_yaw(void);

#ifdef __cplusplus
}
#endif
