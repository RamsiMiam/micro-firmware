#include "odometry.h"
#include "config.h"
#include "esp_err.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "freertos/semphr.h"
#include "imu.h"
#include "motor.h"
#include "motor_controller.h"

static float Length;
static float diameter;

motor_t motors[NUM_MOTORS];

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

static esp_err_t robot_motors_init(void) {
  for (int i = 0; i < 1; i++) {
/*    if (motor_init(&motors[i], i, motor_IN1[i], motor_IN2[i],
                   motor_channel_IN1[i], motor_channel_IN2[i], motor_A[i],
                   motor_B[i], 1) != ESP_OK)
      return ESP_FAIL;*/

    if (motor_controller_init(&motors[i]) != ESP_OK)
      return ESP_FAIL;
  }
  return ESP_OK;
}

esp_err_t odometry_init(float L, float d) {
  if (L <= 0.0f || d <= 0.0f)
    return ESP_FAIL;
  Length = L;
  diameter = d;

  if (imu_init(SDA_GPIO, SCL_GPIO, INT_MPU) != ESP_OK)
    return ESP_FAIL;

  if (robot_motors_init() != ESP_OK)
    return ESP_FAIL;

  set_motor_speed_target(&motors[0], 110.0f);


  return ESP_OK;
}