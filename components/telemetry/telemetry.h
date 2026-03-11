#pragma once

#include "esp_err.h"
#include "motor.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TAG_TELEMETRY "TELEMETRY"

typedef struct {
  float wheel_speed[4];
  float x;
  float y;
  float theta;
  uint32_t timestamp;
  uint32_t seq;
} robot_state_t;

esp_err_t telemetry_init(const uint8_t peer_mac[6], motor_t *motors);

#ifdef __cplusplus
}
#endif
