#include "imu.h"

#include "bmi160.h"
#include "esp_log.h"
#include "i2cdev.h"
#include <string.h>

static bmi160_t bmi;

#define I2C_PORT I2C_NUM_0
#define I2C_SDA_GPIO GPIO_NUM_41
#define I2C_SCL_GPIO GPIO_NUM_42
#define BMI160_ADDR BMI160_I2C_ADDRESS_GND

esp_err_t imu_init(int i2c_sda, int i2c_scl) {
  esp_err_t err;
  err = i2cdev_init();
  if (err != ESP_OK) {
    return err;
  }

  memset(&bmi.i2c_dev, 0, sizeof(i2c_dev_t));

  err = bmi160_init(&bmi, BMI160_ADDR, I2C_PORT, I2C_SDA_GPIO, I2C_SCL_GPIO);
  if (err != ESP_OK) {
    return err;
  }

  vTaskDelay(pdMS_TO_TICKS(100));
  
  
	bmi160_conf_t conf = {0};
	
	conf.accMode  = BMI160_PMU_ACC_NORMAL;
	conf.accRange = BMI160_ACC_RANGE_2G;
	conf.accOdr   = BMI160_ACC_ODR_100HZ;
	conf.accAvg   = BMI160_ACC_LP_AVG_4;
	conf.accUs    = BMI160_ACC_US_ON;
	
	conf.gyrMode  = BMI160_PMU_GYR_NORMAL;
	conf.gyrRange = BMI160_GYR_RANGE_250DPS;
	conf.gyrOdr   = BMI160_GYR_ODR_100HZ;

  err = bmi160_start(&bmi, &conf);
  if (err != ESP_OK) {
    return err;
  }

  ESP_LOGI(TAG_IMU, "BMI OK!");
  return ESP_OK;
}