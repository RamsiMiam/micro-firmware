#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>

#include "env.h"
#include "imu.h"

void app_main(void) {
	int x = imu_init(SDA_GPIO, SCL_GPIO);
	
    while (true) {
        printf("Hello from app_main!\n");
        sleep(1);
    }
}
