#pragma once

#include "driver/pcnt.h"
#include "esp_err.h"
#include "hal/ledc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TAG_MOTOR_CONTROL "MOTOR"
#define COUNTS_PER_REV 294.0f

typedef struct {
  ledc_channel_t ch_in1;
  ledc_channel_t ch_in2;
  pcnt_unit_t pcnt_unit;

  int32_t last_count;
} motor_t;

esp_err_t motor_init(motor_t *motor, int pin_IN1, int pin_IN2,
                     ledc_channel_t channel_IN1, ledc_channel_t channel_IN2,
                     int pin_A, int pin_B, pcnt_unit_t unit);
void motor_set_pwm(motor_t *motor, int pwm);

#ifdef __cplusplus
}
#endif
