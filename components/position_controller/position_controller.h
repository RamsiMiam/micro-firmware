#pragma once

#include "esp_err.h"
#include "motor.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t position_controller_init(motor_t *motors);

#ifdef __cplusplus
}
#endif