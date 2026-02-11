#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  float x;
  float y;
  float z;
} imu_accel_t;

typedef struct {
  float x;
  float y;
  float z;
} imu_gyro_t;

#define TAG_IMU "IMU"

esp_err_t imu_init(int i2c_sda, int i2c_scl, int int_pin);

float get_roll(void);

float get_pitch(void);

float get_yaw(void);

#ifdef __cplusplus
}
#endif
