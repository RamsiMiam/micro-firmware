#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "odometry.h"

void app_main(void) {
  //odometry_init();

  while (true) {

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
