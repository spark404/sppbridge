//
// Created by Hugo Trippaers on 27/12/2021.
//
#include "wifi.h"

#include <string.h>
#include "esp_log.h"
#include "esp_wifi.h"

#define TAG "app_wifi"

esp_err_t wifi_ap_init() {
    esp_err_t ret;

    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
            .ap = {
                    .ssid = CONFIG_AP_ESSID,
                    .ssid_len = strlen(CONFIG_AP_ESSID),
                    .ssid_hidden = false,
                    .password = CONFIG_AP_KEY,
                    .authmode = WIFI_AUTH_WPA_WPA2_PSK,
                    .max_connection = 2
            },
    };

    if (!(strlen(CONFIG_AP_KEY) > 0)) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    if ((ret = esp_wifi_set_mode(WIFI_MODE_AP)) != ESP_OK) {
        ESP_LOGE(TAG, "%s wifi mode set failed: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }

    if ((ret = esp_wifi_set_config(WIFI_IF_AP, &wifi_config)) != ESP_OK) {
        ESP_LOGE(TAG, "%s wifi config set failed: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }

    if ((ret = esp_wifi_start()) != ESP_OK) {
        ESP_LOGE(TAG, "%s wifi start failed: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Wireless AP started with ESSID %s", CONFIG_AP_ESSID);
    return ESP_OK;
}
