#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>

#include "../components/motor/motor.h"
#include "env.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "imu.h"
#include "motor_controller.h"

#define MAX_DUTY ((1 << 10) - 1) // 1023 para 10 bits

void app_main(void) {
  /*  if (imu_init(SDA_GPIO, SCL_GPIO, INT_MPU) != ESP_OK)
      return;*/

  motor_t motor_1;
  if (motor_init(&motor_1, 0, MOTOR_1_IN1, MOTOR_1_IN2, MOTOR_1_CHANNEL_IN1,
                 MOTOR_1_CHANNEL_IN2, MOTOR_1_A, MOTOR_1_B, MOTOR_1_UNIT,
                 0) != ESP_OK)
    return;

  motor_t motor_2;
  if (motor_init(&motor_2, 1, MOTOR_2_IN1, MOTOR_2_IN2, MOTOR_2_CHANNEL_IN1,
                 MOTOR_2_CHANNEL_IN2, MOTOR_2_A, MOTOR_2_B, MOTOR_2_UNIT,
                 0) != ESP_OK)
    return;

  motor_t motor_3;
  if (motor_init(&motor_3, 2, MOTOR_3_IN1, MOTOR_3_IN2, MOTOR_3_CHANNEL_IN1,
                 MOTOR_3_CHANNEL_IN2, MOTOR_3_A, MOTOR_3_B, MOTOR_3_UNIT,
                 1) != ESP_OK)
    return;

  motor_t motor_4;
  if (motor_init(&motor_4, 3, MOTOR_4_IN1, MOTOR_4_IN2, MOTOR_4_CHANNEL_IN1,
                 MOTOR_4_CHANNEL_IN2, MOTOR_4_A, MOTOR_4_B, MOTOR_4_UNIT,
                 0) != ESP_OK)
    return;

  int delay = 2000;
  if (motor_controller_init(&motor_1) != ESP_OK)
    return;

  if (motor_controller_init(&motor_2) != ESP_OK)
    return;

  if (motor_controller_init(&motor_3) != ESP_OK)
    return;

  if (motor_controller_init(&motor_4) != ESP_OK)
    return;

  set_motor_speed_target(&motor_1, 110.0f);
  set_motor_speed_target(&motor_2, 110.0f);
  set_motor_speed_target(&motor_3, 110.0f);
  set_motor_speed_target(&motor_4, 110.0f);

  vTaskDelay(pdMS_TO_TICKS(delay));

  set_motor_speed_target(&motor_1, 80.0f);
  set_motor_speed_target(&motor_2, 80.0f);
  set_motor_speed_target(&motor_3, 80.0f);
  set_motor_speed_target(&motor_4, 80.0f);

  vTaskDelay(pdMS_TO_TICKS(delay));

  set_motor_speed_target(&motor_1, 100.0f);
  set_motor_speed_target(&motor_2, 100.0f);
  set_motor_speed_target(&motor_3, 100.0f);
  set_motor_speed_target(&motor_4, 100.0f);

  vTaskDelay(pdMS_TO_TICKS(delay));

  set_motor_speed_target(&motor_1, 70.0f);
  set_motor_speed_target(&motor_2, 70.0f);
  set_motor_speed_target(&motor_3, 70.0f);
  set_motor_speed_target(&motor_4, 70.0f);

  vTaskDelay(pdMS_TO_TICKS(delay));

  set_motor_speed_target(&motor_1, 0.0f);
  set_motor_speed_target(&motor_2, 0.0f);
  set_motor_speed_target(&motor_3, 0.0f);
  set_motor_speed_target(&motor_4, 0.0f);

  vTaskDelay(pdMS_TO_TICKS(delay));

  set_motor_speed_target(&motor_1, 15.0f);
  set_motor_speed_target(&motor_2, 15.0f);
  set_motor_speed_target(&motor_3, 15.0f);
  set_motor_speed_target(&motor_4, 15.0f);

  vTaskDelay(pdMS_TO_TICKS(delay));

  set_motor_speed_target(&motor_1, 10.0f);
  set_motor_speed_target(&motor_2, 10.0f);
  set_motor_speed_target(&motor_3, 10.0f);
  set_motor_speed_target(&motor_4, 10.0f);

  vTaskDelay(pdMS_TO_TICKS(delay));

  set_motor_speed_target(&motor_1, 0.0f);
  set_motor_speed_target(&motor_2, 0.0f);
  set_motor_speed_target(&motor_3, 0.0f);
  set_motor_speed_target(&motor_4, 0.0f);

  while (true) {

    // printf("DATA,1000.0,1000.0,1000.0\n");
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
