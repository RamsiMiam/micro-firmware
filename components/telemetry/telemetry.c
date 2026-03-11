#include "telemetry.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/projdefs.h"
#include "motor.h"
#include "nvs_flash.h"
#include "odometry.h"
#include <string.h>

static uint8_t peer_addr[6];
static uint32_t seq = 0;

static void espnow_send_cb(const uint8_t *mac_addr,
                           esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) {
    ESP_LOGW(TAG_TELEMETRY, "Send failed");
  }
}

static esp_err_t send_robot_state(robot_state_t *state) {
  state->seq = seq++;

  return esp_now_send(peer_addr, (uint8_t *)state, sizeof(robot_state_t));
}

static void telemetry_task(void *arg) {
  motor_t *motors = (motor_t *)arg;

  while (1) {
    vTaskDelay(pdMS_TO_TICKS(20));

    pose_t robot_pose;
    get_robot_pose(&robot_pose);

    robot_state_t message;
    message.theta = robot_pose.theta;
    message.x = robot_pose.x;
    message.y = robot_pose.y;
    message.wheel_speed[0] = motors[0].omega;
    message.wheel_speed[1] = motors[1].omega;
    message.wheel_speed[2] = motors[2].omega;
    message.wheel_speed[3] = motors[3].omega;
    message.timestamp = esp_timer_get_time();

    send_robot_state(&message);
  }
}

esp_err_t telemetry_init(const uint8_t mac[6], motor_t *motors) {

  if (motors == NULL)
    return ESP_ERR_INVALID_ARG;

  memcpy(peer_addr, mac, 6);
  esp_err_t ret = nvs_flash_init();

  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }

  ESP_ERROR_CHECK(ret);

  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_ERROR_CHECK(esp_now_init());

  ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));

  esp_now_peer_info_t peer = {0};

  memcpy(peer.peer_addr, peer_addr, 6);
  peer.channel = 0;
  peer.encrypt = false;

  ESP_ERROR_CHECK(esp_now_add_peer(&peer));

  xTaskCreate(telemetry_task, "telemetry_task", 4 * 1024, (void *)motors, 1,
              NULL);

  ESP_LOGI(TAG_TELEMETRY, "Telemetry initialized");

  return ESP_OK;
}
