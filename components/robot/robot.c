#include "config.h"
#include "motor.h"
#include "motor_controller.h"

static motor_t motors[NUM_MOTORS];

const int motor_IN1[] = {MOTOR_1_IN1, MOTOR_2_IN1, MOTOR_3_IN1, MOTOR_4_IN1};
const int motor_IN2[] = {MOTOR_1_IN2, MOTOR_2_IN2, MOTOR_3_IN2, MOTOR_4_IN2};
const ledc_channel_t motor_channel_IN1[] = {
    MOTOR_1_CHANNEL_IN1, MOTOR_2_CHANNEL_IN1, MOTOR_3_CHANNEL_IN1,
    MOTOR_4_CHANNEL_IN1};
const ledc_channel_t motor_channel_IN2[] = {
    MOTOR_1_CHANNEL_IN2, MOTOR_2_CHANNEL_IN2, MOTOR_3_CHANNEL_IN2,
    MOTOR_4_CHANNEL_IN2};
const int motor_A[] = {MOTOR_1_A, MOTOR_2_A, MOTOR_3_A, MOTOR_4_A};
const int motor_B[] = {MOTOR_1_B, MOTOR_2_B, MOTOR_3_B, MOTOR_4_B};
const pcnt_unit_t motor_unit[] = {MOTOR_1_UNIT, MOTOR_2_UNIT, MOTOR_3_UNIT,
                                  MOTOR_4_UNIT};

static esp_err_t robot_motors_init(void) {
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (motor_init(&motors[i], i, motor_IN1[i], motor_IN2[i],
                   motor_channel_IN1[i], motor_channel_IN2[i], motor_A[i],
                   motor_B[i], motor_unit[i], 1) != ESP_OK)
      return ESP_FAIL;

    if (motor_controller_init(&motors[i]) != ESP_OK)
      return ESP_FAIL;
  }
  return ESP_OK;
}