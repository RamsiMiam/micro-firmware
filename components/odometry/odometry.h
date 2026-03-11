#pragma once

#include "motor.h"
#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  volatile float x;
  volatile float y;
  volatile float theta;
} pose_t;

esp_err_t odometry_init(motor_t *motors);
esp_err_t get_robot_pose(pose_t *pose);

#ifdef __cplusplus
}
#endif
