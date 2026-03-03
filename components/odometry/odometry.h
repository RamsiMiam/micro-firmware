#pragma once

#include "motor.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	float x;
	float y;
	float theta;
} pose_t;

esp_err_t odometry_init(float L, float d);

#ifdef __cplusplus
}
#endif
