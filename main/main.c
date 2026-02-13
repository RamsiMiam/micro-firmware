#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>

#include "../components/motor/motor.h"
#include "env.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu.h"

#define MAX_DUTY ((1 << 10) - 1) // 1023 para 10 bits

void app_main(void) {
  if (imu_init(SDA_GPIO, SCL_GPIO, INT_MPU) != ESP_OK)
    return;

  motor_t motor_1;
  if (motor_init(&motor_1, MOTOR_1_IN1, MOTOR_1_IN2, MOTOR_1_CHANNEL_IN1,
                 MOTOR_1_CHANNEL_IN2, MOTOR_1_A, MOTOR_1_B, MOTOR_1_UNIT))
    return;
    
  while (true) {
  }
}
