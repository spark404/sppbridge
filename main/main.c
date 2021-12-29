//
// Created by Hugo Trippaers on 26/12/2021.
//
#include <sys/cdefs.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#include "esp_bt.h"
#include "esp_event.h"

#include "mavlink_bridge.h"
#include "server.h"
#include "wifi.h"
#include "bt.h"

#define BUILTIN_LED_PIN 2

#define RD_BUF_SIZE (1024)

#define GPIO_HIGH 1
#define GPIO_LOW 0

#define TAG "app_main"

static uint64_t msg_cnt_in, msg_cnt_out;
static bool spp_peer_found;
static mavlink_status_t mavlink_status;
static mavlink_message_t mavlink_msg;
static QueueHandle_t bt_mavlink_message_queue;
static QueueHandle_t server_mavlink_message_queue;

void bt_spp_receive_callback(const uint8_t *data, uint16_t len) {
    for (int i = 0; i < len; i++) {
        if (mavlink_frame_char(MAVLINK_COMM_0, *(data + i), &mavlink_msg, &mavlink_status) !=
            MAVLINK_FRAMING_INCOMPLETE) {
            // Make a copy of the message and send it on the queue
            if (xQueueSend(bt_mavlink_message_queue, &mavlink_msg, 0) != pdPASS) {
                ESP_LOGE(TAG, "Unable to send message to handler queue");
            }
        }
    }
}

void bt_spp_status_callback(const bool connected) {
    spp_peer_found = connected;
}

void server_receive_callback(const uint8_t *data, const uint16_t len) {
    for (int i = 0; i < len; i++) {
        if (mavlink_frame_char(MAVLINK_COMM_0, *(data + i), &mavlink_msg, &mavlink_status) !=
            MAVLINK_FRAMING_INCOMPLETE) {
            // Make a copy of the message and send it on the queue
            if (xQueueSend(server_mavlink_message_queue, &mavlink_msg, 0) != pdPASS) {
                ESP_LOGE(TAG, "Unable to send message to handler queue");
            }
        }
    }
}

_Noreturn void mavlink_to_spp_task(__attribute__((unused)) void *pvParameters) {
    mavlink_message_t *queued_msg;
    uint8_t buf[2048];
    queued_msg = calloc(1, sizeof(mavlink_message_t));

    for (;;) {
        xQueueReceive(bt_mavlink_message_queue, (void *) queued_msg, (portTickType) portMAX_DELAY);
        msg_cnt_in++;
        size_t size = mavlink_msg_to_send_buffer(buf, queued_msg);
        server_write(buf, size);
    }
}

_Noreturn void mavlink_from_spp_task(__attribute__((unused)) void *pvParameters) {
    mavlink_message_t *queued_msg;
    uint8_t buf[2048];
    queued_msg = calloc(1, sizeof(mavlink_message_t));

    for (;;) {
        xQueueReceive(server_mavlink_message_queue, (void *) queued_msg, (portTickType) portMAX_DELAY);
        msg_cnt_out++;
        size_t size = mavlink_msg_to_send_buffer(buf, queued_msg);
        bt_spp_write(buf, size);
    }
}

_Noreturn void indicator_task(__attribute__((unused)) void *pvParameters) {
    // Enable the onboard LED
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_direction(BUILTIN_LED_PIN, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_level(BUILTIN_LED_PIN, GPIO_LOW));

    bool led_toggle = 0;

    for(;;) {
        if (spp_peer_found) {
            ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_level(BUILTIN_LED_PIN, GPIO_LOW));
        } else {
            led_toggle = !led_toggle;
            ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_level(BUILTIN_LED_PIN, led_toggle));
        }
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

_Noreturn void app_main(void) {
    esp_log_level_set("app_main", ESP_LOG_INFO);
    esp_log_level_set("app_bt", ESP_LOG_INFO);
    esp_log_level_set("app_wifi", ESP_LOG_INFO);
    esp_log_level_set("app_server", ESP_LOG_INFO);

    // Initialize NVS.
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // Not using BLE, so release the memory
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    // Initialize Event Loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    bt_mavlink_message_queue = xQueueCreate(20, sizeof(mavlink_message_t));
    server_mavlink_message_queue = xQueueCreate(20, sizeof(mavlink_message_t));

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(wifi_ap_init());

    spp_device device = {
            .hwaddr = CONFIG_PEER_HWADDR,
            .pin = CONFIG_PEER_PIN
    };
    ESP_ERROR_CHECK(bt_spp_server_init(&device));

    xTaskCreate(indicator_task, "Indicator Task", 1024, NULL, 20, NULL);
    xTaskCreate(mavlink_from_spp_task, "Mavlink Incoming Task", 4096, NULL, 9, NULL);
    xTaskCreate(mavlink_to_spp_task, "Mavlink Outgoing Task", 4096, NULL, 10, NULL);
    xTaskCreate(server_task, "Server Task", 4096, NULL, 15, NULL);

    for(;;) {
        ESP_LOGI(TAG, "Messages; in: %llu, out: %llu", msg_cnt_in, msg_cnt_out);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
