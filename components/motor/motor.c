#include "motor.h"

#include "driver/ledc.h"
#include "driver/pcnt.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "hal/ledc_types.h"

static bool pwm_timer_initialized = false;

static inline int clamp_int(int value, int min, int max) {
  if (value < min)
    return min;
  if (value > max)
    return max;
  return value;
}

static esp_err_t pwm_timer_init(void) {
  if (pwm_timer_initialized)
    return ESP_OK;

  ledc_timer_config_t timer = {.speed_mode = LEDC_LOW_SPEED_MODE,
                               .timer_num = LEDC_TIMER_0,
                               .duty_resolution = LEDC_TIMER_10_BIT,
                               .freq_hz = 20000,
                               .clk_cfg = LEDC_AUTO_CLK};

  esp_err_t err = ledc_timer_config(&timer);
  if (err == ESP_OK)
    pwm_timer_initialized = true;

  return err;
}

static esp_err_t pwm_init(int pin_IN1, int pin_IN2, ledc_channel_t channel_IN1,
                          ledc_channel_t channel_IN2) {
  esp_err_t err = pwm_timer_init();
  if (err != ESP_OK)
    return err;

  ledc_channel_config_t ch1 = {.gpio_num = pin_IN1,
                               .speed_mode = LEDC_LOW_SPEED_MODE,
                               .channel = channel_IN1,
                               .timer_sel = LEDC_TIMER_0,
                               .duty = 0,
                               .hpoint = 0};

  ledc_channel_config_t ch2 = {.gpio_num = pin_IN2,
                               .speed_mode = LEDC_LOW_SPEED_MODE,
                               .channel = channel_IN2,
                               .timer_sel = LEDC_TIMER_0,
                               .duty = 0,
                               .hpoint = 0};

  err = ledc_channel_config(&ch1);
  if (err != ESP_OK)
    return err;

  return ledc_channel_config(&ch2);
}

static esp_err_t pcnt_init(int pin_A, int pin_B, pcnt_unit_t unit) {
  pcnt_config_t pcnt_config = {.pulse_gpio_num = pin_A,
                               .ctrl_gpio_num = pin_B,
                               .lctrl_mode = PCNT_MODE_REVERSE,
                               .hctrl_mode = PCNT_MODE_KEEP,
                               .pos_mode = PCNT_COUNT_INC,
                               .neg_mode = PCNT_COUNT_DEC,
                               .counter_h_lim = 32767,
                               .counter_l_lim = -32768,
                               .unit = unit,
                               .channel = PCNT_CHANNEL_0};

  esp_err_t err = pcnt_unit_config(&pcnt_config);
  if (err != ESP_OK)
    return err;

  pcnt_config_t pcnt_config_B = {.pulse_gpio_num = pin_B,
                                 .ctrl_gpio_num = pin_A,
                                 .lctrl_mode = PCNT_MODE_REVERSE,
                                 .hctrl_mode = PCNT_MODE_KEEP,
                                 .pos_mode = PCNT_COUNT_DEC,
                                 .neg_mode = PCNT_COUNT_INC,
                                 .counter_h_lim = 32767,
                                 .counter_l_lim = -32768,
                                 .unit = unit,
                                 .channel = PCNT_CHANNEL_1};

  err = pcnt_unit_config(&pcnt_config_B);
  if (err != ESP_OK)
    return err;

  pcnt_counter_pause(unit);
  pcnt_counter_clear(unit);
  pcnt_counter_resume(unit);
  return ESP_OK;
}

esp_err_t motor_init(motor_t *motor, int pin_IN1, int pin_IN2,
                     ledc_channel_t channel_IN1, ledc_channel_t channel_IN2,
                     int pin_A, int pin_B, pcnt_unit_t unit) {

  if (motor == NULL)
    return ESP_ERR_INVALID_ARG;

  esp_err_t err = pwm_init(pin_IN1, pin_IN2, channel_IN1, channel_IN2);

  if (err != ESP_OK)
    return err;

  err = pcnt_init(pin_A, pin_B, unit);
  if (err != ESP_OK)
    return err;

  motor->ch_in1 = channel_IN1;
  motor->ch_in2 = channel_IN2;
  motor->pcnt_unit = unit;
  motor->last_count = 0;

  ESP_LOGI(TAG_MOTOR_CONTROL, "MOTOR OK!");

  return ESP_OK;
}

void motor_set_pwm(motor_t *motor, int pwm) {

  if (motor == NULL)
    return;

  pwm = clamp_int(pwm, -1023, 1023);

  if (pwm > 0) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, motor->ch_in2, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, motor->ch_in2);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, motor->ch_in1, pwm);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, motor->ch_in1);

  } else {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, motor->ch_in1, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, motor->ch_in1);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, motor->ch_in2, -pwm);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, motor->ch_in2);
  }
}

float motor_get_speed_rpm(motor_t *motor, float dt) {

  if (motor == NULL)
    return 0.0f;

  int16_t current_count = 0;
  pcnt_get_counter_value(motor->pcnt_unit, &current_count);

  int32_t delta = current_count - motor->last_count;
  motor->last_count = current_count;

  float rpm = (delta * 60.0f) / (COUNTS_PER_REV * dt);

  return rpm;
}