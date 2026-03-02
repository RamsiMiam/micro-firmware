#pragma once

#include "motor.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t motor_controller_init(motor_t *motor);
esp_err_t set_motor_speed_target(motor_t *motor, float speed);

#ifdef __cplusplus
}
#endif
