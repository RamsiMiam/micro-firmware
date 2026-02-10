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

esp_err_t  imu_init(int i2c_sda, int i2c_scl);

void imu_read_accel(imu_accel_t *accel);

void imu_read_gyro(imu_gyro_t *gyro);

#ifdef __cplusplus
}
#endif
