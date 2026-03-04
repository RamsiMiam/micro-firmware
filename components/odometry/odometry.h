#pragma once

#include "esp_err.h"
#include "motor.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  volatile float x;
  volatile float y;
  volatile float theta;
} pose_t;

esp_err_t odometry_init(void);

#ifdef __cplusplus
}
#endif
