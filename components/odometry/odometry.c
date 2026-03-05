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

static pose_t robot_pose;

static SemaphoreHandle_t pose_mutex;

static void odometry_task(void *arg) {
  motor_t *motors = (motor_t *)arg;
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

esp_err_t odometry_init(motor_t *motors) {
  pose_mutex = xSemaphoreCreateMutex();
  if (pose_mutex == NULL)
    return ESP_FAIL;

  if (imu_init(SDA_GPIO, SCL_GPIO, INT_MPU) != ESP_OK)
    return ESP_FAIL;

  robot_pose.theta = 0.0f;
  robot_pose.x = 0.0f;
  robot_pose.y = 0.0f;

  xTaskCreate(odometry_task, "odometry_task", 4 * 1024, (void *)motors, 7,
              NULL);

  return ESP_OK;
}

esp_err_t get_robot_pose(pose_t *pose) {
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