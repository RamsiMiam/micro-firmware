#include "motor_controller.h"
#include "esp_err.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "freertos/semphr.h"
#include "motor.h"

static float target_motor_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};

static void motor_controller_task(void *arg) {
  motor_t *motor = (motor_t *)arg;

  const float kp = 20.0f;
  const float ki = 50.0f;
  const float dt = 0.01f;

  float integral = 0.0f;
  float u;
  while (1) {
    float motor_speed = motor_get_speed(motor, dt);
    motor->omega = motor_speed;

    float error = target_motor_speed[motor->id] - motor_speed;
    integral += error * dt;
    u = kp * error + ki * integral;

    motor_set_pwm(motor, u);
    if (motor->plot)
      printf("DATA,%f,%f,%f\n", target_motor_speed[motor->id], motor_speed, u);

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

esp_err_t motor_controller_init(motor_t *motor) {
  if (motor == NULL)
    return ESP_FAIL;

  xTaskCreatePinnedToCore(motor_controller_task, "motor_controller_task",
                          8 * 1024, motor, 8, NULL, 1);

  return ESP_OK;
}

esp_err_t set_motor_speed_target(motor_t *motor, float speed) {
  if (motor == NULL)
    return ESP_FAIL;
  target_motor_speed[motor->id] = speed;
  return ESP_OK;
}