#include "odometry.h"
#include "config.h"
#include "esp_err.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "freertos/semphr.h"
#include "imu.h"
#include "motor.h"
#include "motor_controller.h"
#include <math.h>

static motor_t motors[NUM_MOTORS];
static pose_t robot_pose;

static SemaphoreHandle_t pose_mutex;

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

static void odometry_task(void *arg) {
  float dt = 0.01f;

  while (1) {

    float omega_L = (motors[0].omega + motors[1].omega) * 0.5f;
    float omega_R = (motors[2].omega + motors[3].omega) * 0.5f;

    float v = (R * 0.5f) * (omega_L + omega_R);
    float theta = get_yaw();

    float x_d = v * cosf(theta);
    float y_d = v * sinf(theta);

    if (xSemaphoreTake(pose_mutex, portMAX_DELAY) == pdTRUE) {
      robot_pose.x += x_d * dt;
      robot_pose.y += y_d * dt;
      robot_pose.theta = theta;

      xSemaphoreGive(pose_mutex);
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

esp_err_t odometry_init() {

  pose_mutex = xSemaphoreCreateMutex();
  if (pose_mutex == NULL)
    return ESP_FAIL;

  if (imu_init(SDA_GPIO, SCL_GPIO, INT_MPU) != ESP_OK)
    return ESP_FAIL;

  if (robot_motors_init() != ESP_OK)
    return ESP_FAIL;

  robot_pose.theta = 0.0f;
  robot_pose.x = 0.0f;
  robot_pose.y = 0.0f;

  xTaskCreate(odometry_task, "odometry_task", 4 * 1024, NULL, 7, NULL);

  return ESP_OK;
}

esp_err_t get_robot_pos(pose_t *pose) {
  if (pose == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  if (xSemaphoreTake(pose_mutex, portMAX_DELAY) == pdTRUE) {
    *pose = robot_pose;

    xSemaphoreGive(pose_mutex);
  } else {
    return ESP_FAIL;
  }

  return ESP_OK;
}