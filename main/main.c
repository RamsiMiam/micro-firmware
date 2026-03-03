#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"

#include "odometry.h"

#define MAX_DUTY ((1 << 10) - 1) // 1023 para 10 bits

void app_main(void) {
  odometry_init(1.0f, 1.0f);

  while (true) {

    // printf("DATA,1000.0,1000.0,1000.0\n");
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
