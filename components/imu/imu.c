#include "imu.h"

#include "bmi160.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "freertos/semphr.h"
#include "i2cdev.h"
#include <math.h>
#include <string.h>

static bmi160_t bmi;

static TaskHandle_t imu_task_handle = NULL;
static SemaphoreHandle_t imu_mutex = NULL;

static float roll = 0.0f;
static float pitch = 0.0f;
static float yaw = 0.0f;

#define I2C_PORT I2C_NUM_0

#define BMI160_ADDR BMI160_I2C_ADDRESS_GND

static void imu_task(void *arg) {
  imu_task_handle = xTaskGetCurrentTaskHandle();
  int64_t t_prev = esp_timer_get_time();
  const float alpha = 0.98f;

  bmi160_result_t result;

  while (1) {

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    int64_t t_now = esp_timer_get_time();
    float dt = (t_now - t_prev) * 1e-6f;
    t_prev = t_now;

    if (bmi160_read_data(&bmi, &result) == ESP_OK) {
      float roll_acc = atan2(result.accY, result.accZ) * 180.0f / M_PI;
      float pitch_acc = atan2(-result.accX, sqrt(result.accY * result.accY +
                                                 result.accZ * result.accZ)) *
                        180.0f / M_PI;

      if (xSemaphoreTake(imu_mutex, portMAX_DELAY) == pdTRUE) {
        roll = alpha * (roll + result.gyroX * dt) + (1.0f - alpha) * roll_acc;
        pitch =
            alpha * (pitch + result.gyroY * dt) + (1.0f - alpha) * pitch_acc;

        yaw += result.gyroZ * dt;
        xSemaphoreGive(imu_mutex);
      }
    }

    ESP_LOGI(TAG_IMU, "Roll: %.2f Pitch: %.2f Yaw: %.2f", roll, pitch, yaw);

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

static void IRAM_ATTR bmi160_isr(void *arg) {
  BaseType_t hp_task_woken = pdFALSE;
  vTaskNotifyGiveFromISR(imu_task_handle, &hp_task_woken);
  if (hp_task_woken) {
    portYIELD_FROM_ISR();
  }
}

static esp_err_t set_interruption(int int_pin) {
  gpio_config_t io_conf = {.pin_bit_mask = 1ULL << int_pin,
                           .mode = GPIO_MODE_INPUT,
                           .pull_up_en = GPIO_PULLUP_DISABLE,
                           .pull_down_en = GPIO_PULLDOWN_DISABLE,
                           .intr_type = GPIO_INTR_POSEDGE};

  gpio_config(&io_conf);

  gpio_install_isr_service(0);
  gpio_isr_handler_add(int_pin, bmi160_isr, NULL);

  bmi160_int_out_conf_t conf = {0};
  conf.intPin = BMI160_PIN_INT1;
  conf.intEnable = BMI160_INT_ENABLE;
  conf.intLevel = BMI160_INT_ACTIVE_LOW;

  return bmi160_enable_int_new_data(&bmi, &conf);
}

//=====================================================================================================================
//=====================================================================================================================

esp_err_t imu_init(int i2c_sda, int i2c_scl, int int_pin) {

  imu_mutex = xSemaphoreCreateMutex();
  if (imu_mutex == NULL) {
    ESP_LOGE(TAG_IMU, "Error creating IMU mutex");
    return ESP_FAIL;
  }

  esp_err_t err;
  err = i2cdev_init();
  if (err != ESP_OK) {
    return err;
  }

  memset(&bmi.i2c_dev, 0, sizeof(i2c_dev_t));

  err = bmi160_init(&bmi, BMI160_ADDR, I2C_PORT, i2c_sda, i2c_scl);
  if (err != ESP_OK) {
    return err;
  }

  vTaskDelay(pdMS_TO_TICKS(100));

  bmi160_conf_t conf = {0};

  conf.accMode = BMI160_PMU_ACC_NORMAL;
  conf.accRange = BMI160_ACC_RANGE_2G;
  conf.accOdr = BMI160_ACC_ODR_100HZ;
  conf.accAvg = BMI160_ACC_LP_AVG_8;
  conf.accUs = BMI160_ACC_US_ON;

  conf.gyrMode = BMI160_PMU_GYR_NORMAL;
  conf.gyrRange = BMI160_GYR_RANGE_2000DPS;
  conf.gyrOdr = BMI160_GYR_ODR_100HZ;

  err = bmi160_start(&bmi, &conf);
  if (err != ESP_OK) {
    return err;
  }

  err = bmi160_calibrate(&bmi);
  if (err != ESP_OK) {
    return err;
  }

  xTaskCreatePinnedToCore(imu_task, "imu_task", 4 * 1024, NULL, 5, NULL, 1);

  err = set_interruption(int_pin);
  if (err != ESP_OK) {
    return err;
  }

  ESP_LOGI(TAG_IMU, "BMI OK!");

  return ESP_OK;
}

float get_roll(void) {
  float r = 0.0;

  if (xSemaphoreTake(imu_mutex, portMAX_DELAY) == pdTRUE) {
    r = roll;
    xSemaphoreGive(imu_mutex);
  }

  return r;
};

float get_pitch(void) {
  float p = 0.0;

  if (xSemaphoreTake(imu_mutex, portMAX_DELAY) == pdTRUE) {
    p = pitch;
    xSemaphoreGive(imu_mutex);
  }

  return p;
};

float get_yaw(void) {
  float y = 0.0;

  if (xSemaphoreTake(imu_mutex, portMAX_DELAY) == pdTRUE) {
    y = yaw;
    xSemaphoreGive(imu_mutex);
  }

  return y;
};
