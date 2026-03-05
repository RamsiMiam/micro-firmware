#include "position_controller.h"
#include "config.h"
#include "esp_err.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "freertos/semphr.h"
#include "motor_controller.h"
#include "odometry.h"
#include <math.h>

static pose_t target_pose;
static SemaphoreHandle_t position_mutex;

static void position_controller_task(void *arg) {
  motor_t *motors = (motor_t *)arg;

  const float K_d = 1.5f;
  const float K_theta = 4.0f;

  while (1) {
    pose_t current_pose;
    if (get_robot_pose(&current_pose) != ESP_OK)
      continue;

    float e_x = target_pose.x - current_pose.x;
    float e_y = target_pose.y - current_pose.y;

    float e_d = sqrtf(e_x * e_x + e_y * e_y);

    float e_theta = target_pose.theta - current_pose.theta;

    float v = K_d * e_d;
    float w = K_theta * e_theta;

    float omega_R = (2 * v + w * L) / (2 * R);
    float omega_L = (2 * v - w * L) / (2 * R);

    set_motor_speed_target(&motors[0], omega_L);
    set_motor_speed_target(&motors[1], omega_L);

    set_motor_speed_target(&motors[2], omega_R);
    set_motor_speed_target(&motors[3], omega_R);

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

esp_err_t position_controller_init(motor_t *motors) {
  if (motors == NULL)
    return ESP_ERR_INVALID_ARG;

  position_mutex = xSemaphoreCreateMutex();
  if (position_mutex == NULL)
    return ESP_FAIL;

  xTaskCreate(position_controller_task, "position_controller_init", 4 * 1024,
              (void *)motors, 6, NULL);

  return ESP_OK;
}

esp_err_t set_target_pose(float x, float y, float theta) {
  if (xSemaphoreTake(position_mutex, portMAX_DELAY) == pdTRUE) {
    target_pose.x = x;
    target_pose.y = y;
    target_pose.theta = theta;

    xSemaphoreGive(position_mutex);
  } else {
    return ESP_FAIL;
  }

  return ESP_OK;
}