#include <sys/cdefs.h>
//
// Created by Hugo Trippaers on 26/12/2021.
//
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"
#include "esp_wifi.h"
#include "esp_event.h"

#include "mavlink_bridge.h"
#include "server.h"

void bt_receive_callback(const uint8_t *data, uint16_t len);

static esp_err_t wifi_ap_init();
static esp_err_t bt_spp_server_init();

#define BUILTIN_LED_PIN 2

#define RD_BUF_SIZE (1024)

#define GPIO_HIGH 1
#define GPIO_LOW 0

#define TAG "app_main"

static const char *peer_device_addr_str = CONFIG_PEER_HWADDR;
static const char *peer_device_pin_str = CONFIG_PEER_PIN;
static esp_bd_addr_t peer_device_addr;
static esp_bt_pin_code_t peer_device_pin;

static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_master = ESP_SPP_ROLE_MASTER;

static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;
esp_bd_addr_t peer_bd_addr = {0};
static uint8_t peer_bdname_len;
static char peer_bdname[ESP_BT_GAP_MAX_BDNAME_LEN + 1];

static const esp_bt_inq_mode_t inq_mode = ESP_BT_INQ_MODE_GENERAL_INQUIRY;
static const uint8_t inq_len = 30;
static const uint8_t inq_num_rsps = 0;

static uint32_t handle;

static uint64_t msg_cnt_in, msg_cnt_out;

static SemaphoreHandle_t bt_write_semaphore;
static bool spp_peer_found;

static mavlink_status_t mavlink_status;
static mavlink_message_t mavlink_msg;
static QueueHandle_t bt_mavlink_message_queue;
static QueueHandle_t server_mavlink_message_queue;


static bool get_name_from_eir(uint8_t *eir, char *bdname, uint8_t *bdname_len) {
    uint8_t *rmt_bdname = NULL;
    uint8_t rmt_bdname_len = 0;

    if (!eir) {
        return false;
    }

    rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &rmt_bdname_len);
    if (!rmt_bdname) {
        rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &rmt_bdname_len);
    }

    if (rmt_bdname) {
        if (rmt_bdname_len > ESP_BT_GAP_MAX_BDNAME_LEN) {
            rmt_bdname_len = ESP_BT_GAP_MAX_BDNAME_LEN;
        }

        if (bdname) {
            memcpy(bdname, rmt_bdname, rmt_bdname_len);
            bdname[rmt_bdname_len] = '\0';
        }
        if (bdname_len) {
            *bdname_len = rmt_bdname_len;
        }
        return true;
    }

    return false;
}

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
    switch (event) {
        case ESP_SPP_INIT_EVT:
            ESP_LOGD(TAG, "ESP_SPP_INIT_EVT; status=%d", param->init.status);
            esp_bt_dev_set_device_name("SppBridge");
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            esp_bt_gap_start_discovery(inq_mode, inq_len, inq_num_rsps);
            break;
        case ESP_SPP_DISCOVERY_COMP_EVT:
            ESP_LOGD(TAG, "ESP_SPP_DISCOVERY_COMP_EVT; status=%d ", param->disc_comp.status);
            if (param->disc_comp.status == ESP_SPP_SUCCESS) {
                esp_spp_connect(sec_mask, role_master, param->disc_comp.scn[0], peer_bd_addr);
            }
            break;
        case ESP_SPP_OPEN_EVT:
            ESP_LOGD(TAG, "ESP_SPP_OPEN_EVT; status=%d", param->open.status);
            esp_spp_write(param->open.handle, 13, (uint8_t *) "HELLO WORLD\r\n");
            handle = param->open.handle;
            break;
        case ESP_SPP_CLOSE_EVT:
            ESP_LOGD(TAG, "ESP_SPP_CLOSE_EVT; status=%d", param->close.status);
            handle = 0;
            spp_peer_found = false;
            esp_bt_gap_start_discovery(inq_mode, inq_len, inq_num_rsps); // Restart discovery
            break;
        case ESP_SPP_CL_INIT_EVT:
            ESP_LOGD(TAG, "ESP_SPP_CL_INIT_EVT; status=%d", param->cl_init.status);
            struct spp_cl_init_evt_param *cl_init = &param->cl_init;
            ESP_LOGD(TAG, "[Handle %x] Client init status %d, sec_id %d, use_co %d",
                     cl_init->handle, cl_init->status, cl_init->sec_id, cl_init->use_co);
            break;
        case ESP_SPP_DATA_IND_EVT:
            bt_receive_callback(param->data_ind.data, param->data_ind.len);
            break;
        case ESP_SPP_CONG_EVT:
            ESP_LOGD(TAG, "ESP_SPP_CONG_EVT; status=%d", param->cong.status);
            if (xSemaphoreGive(bt_write_semaphore) != pdTRUE) {
                ESP_LOGE(TAG, "Failed to return write semaphore from ESP_SPP_CONG_EVT");
            }
            break;
        case ESP_SPP_WRITE_EVT:
            ESP_LOGD(TAG, "ESP_SPP_WRITE_EVT; status=%d", param->write.status);
            if (param->write.status != ESP_SPP_SUCCESS) {
                ESP_LOGW(TAG, "last write failed");
            }
            if (!param->write.cong) {
                // Not congested, clear the write flag
                if (xSemaphoreGive(bt_write_semaphore) != pdTRUE) {
                    ESP_LOGE(TAG, "Failed to return write semaphore from ESP_SPP_WRITE_EVT");
                }
            }
            break;
        default:
            // ESP_SPP_UNINIT_EVT
            // ESP_SPP_START_EVT
            // ESP_SPP_UNINIT_EVT
            // ESP_SPP_SRV_OPEN_EVT
            // ESP_SPP_SRV_STOP_EVT
            ESP_LOGD(TAG, "Unhandled EVT_SPP %d", event);
            break;
    }
}

static void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
    switch (event) {
        case ESP_BT_GAP_DISC_RES_EVT:
            ESP_LOGD(TAG, "ESP_BT_GAP_DISC_RES_EVT");
            ESP_LOGD(TAG, "Discovered %02x:%02x:%02x:%02x:%02x:%02x (%d properties)",
                     param->disc_res.bda[0], param->disc_res.bda[1],
                     param->disc_res.bda[2], param->disc_res.bda[3],
                     param->disc_res.bda[4], param->disc_res.bda[5],
                     param->disc_res.num_prop
            );
            for (int i = 0; i < param->disc_res.num_prop; i++) {
                esp_bt_gap_dev_prop_t *prop = &param->disc_res.prop[i];
                switch (prop->type) {
                    case ESP_BT_GAP_DEV_PROP_BDNAME:
                        memcpy(peer_bdname, prop->val, prop->len);
                        peer_bdname[prop->len] = '\0';
                        ESP_LOGD(TAG, "  Devicename: %s", peer_bdname);
                        break;
                    case ESP_BT_GAP_DEV_PROP_COD:
                        ESP_LOGD(TAG, "Property ESP_BT_GAP_DEV_PROP_COD present");
                        uint32_t *cod = prop->val;
                        ESP_LOGD(TAG, "  Cod: %d", *cod);
                        break;
                    case ESP_BT_GAP_DEV_PROP_RSSI:
                        ESP_LOGD(TAG, "Property ESP_BT_GAP_DEV_PROP_RSSI present");
                        int8_t *rssi = prop->val;
                        ESP_LOGD(TAG, "  RSSI: %d", *rssi);
                        break;
                    case ESP_BT_GAP_DEV_PROP_EIR:
                        ESP_LOGD(TAG, "Property ESP_BT_GAP_DEV_PROP_EIR present");
                        if (get_name_from_eir(param->disc_res.prop[i].val, peer_bdname, &peer_bdname_len)) {
                            ESP_LOGD(TAG, "  Devicename: %s", peer_bdname);
                        }
                        break;
                    default:
                        ESP_LOGD(TAG, "Property %d present", prop->type);
                        break;
                }
            }

            if (memcmp(param->disc_res.bda, peer_device_addr, 6) == 0) {
                ESP_LOGI(TAG, "Remote device detected, starting SPP discovery");
                memcpy(peer_bd_addr, param->disc_res.bda, ESP_BD_ADDR_LEN);
                spp_peer_found = true;
                esp_spp_start_discovery(peer_bd_addr);
                esp_bt_gap_cancel_discovery();
            }
            break;
        case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
            ESP_LOGD(TAG, "ESP_BT_GAP_DISC_STATE_CHANGED_EVT");
            if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED && !spp_peer_found) {
                ESP_LOGI(TAG, "Restarting discovery");
                esp_bt_gap_start_discovery(inq_mode, inq_len, inq_num_rsps);
            }
             break;
        case ESP_BT_GAP_AUTH_CMPL_EVT:
            ESP_LOGD(TAG, "ESP_BT_GAP_AUTH_CMPL_EVT");
            ESP_LOGD(TAG, "Authentication status for %02x:%02x:%02x:%02x:%02x:%02x is %d",
                     param->auth_cmpl.bda[0], param->auth_cmpl.bda[1],
                     param->auth_cmpl.bda[2], param->auth_cmpl.bda[3],
                     param->auth_cmpl.bda[4], param->auth_cmpl.bda[5],
                     param->auth_cmpl.stat
            );
            break;
        case ESP_BT_GAP_PIN_REQ_EVT:
            ESP_LOGD(TAG, "ESP_BT_GAP_PIN_REQ_EVT");
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, peer_device_pin);
            break;
        default:
            // ESP_BT_GAP_RMT_SRVCS_EVT
            // ESP_BT_GAP_RMT_SRVC_REC_EVT
            // ESP_BT_GAP_CFM_REQ_EVT
            // ESP_BT_GAP_KEY_NOTIF_EVT
            // ESP_BT_GAP_KEY_REQ_EVT
            // ESP_BT_GAP_READ_RSSI_DELTA_EVT
            // ESP_BT_GAP_CONFIG_EIR_DATA_EVT
            // ESP_BT_GAP_SET_AFH_CHANNELS_EVT
            // ESP_BT_GAP_READ_REMOTE_NAME_EVT
            // ESP_BT_GAP_MODE_CHG_EVT
            // ESP_BT_GAP_REMOVE_BOND_DEV_COMPLETE_EVT
            // ESP_BT_GAP_QOS_CMPL_EVT
            // ESP_BT_GAP_EVT_MAX
            ESP_LOGD(TAG, "Unhandled ESP_BT event %d", event);
            break;
    }
}

void bt_receive_callback(const uint8_t *data, const uint16_t len) {
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

_Noreturn void mavlink_handler_task(void *pvParameters) {
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

_Noreturn void bt_mavlink_outgoing_task(void *pvParameters) {
    mavlink_message_t *queued_msg;
    uint8_t buf[2048];
    queued_msg = calloc(1, sizeof(mavlink_message_t));

    for (;;) {
        xQueueReceive(server_mavlink_message_queue, (void *) queued_msg, (portTickType) portMAX_DELAY);
        msg_cnt_out++;
        if (handle != 0) {
            if (xSemaphoreTake(bt_write_semaphore, pdMS_TO_TICKS(250)) != pdTRUE) {
                ESP_LOGW(TAG, "Discarding message, no write semaphore in 250ms");
                continue;
            }
            size_t size = mavlink_msg_to_send_buffer(buf, queued_msg);
            esp_spp_write(handle, size, buf);
        }
    }
}

_Noreturn void indicator_task(void *pvParameters) {
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

esp_err_t configure_peer() {
    if (strlen(peer_device_addr_str) != 17) {
        return ESP_ERR_INVALID_ARG;
    }

    if (strlen(peer_device_pin_str) != 4) {
        return ESP_ERR_INVALID_ARG;
    }

    char * pEnd;
    for (int i = 0; i<6; i++) {
        peer_device_addr[i] = strtoul(peer_device_addr_str + (i * 3), &pEnd, 16);
    }

    size_t n = sscanf(peer_device_pin_str, "%c%c%c%c",
               &peer_device_pin[0], &peer_device_pin[1],
               &peer_device_pin[2], &peer_device_pin[3]);
    if (n != 4) {
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}


_Noreturn void app_main(void) {
    esp_log_level_set("app_main", ESP_LOG_INFO);
    esp_log_level_set("server", ESP_LOG_INFO);
    esp_log_level_set("BT_BTM", ESP_LOG_WARN);
    esp_log_level_set("BT_HCI", ESP_LOG_WARN);

    // Initialize NVS.
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // Not using BLE, so release the memory
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    ESP_ERROR_CHECK(configure_peer());

    // Initialize Event Loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    bt_write_semaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(bt_write_semaphore);

    bt_mavlink_message_queue = xQueueCreate(20, sizeof(mavlink_message_t));
    server_mavlink_message_queue = xQueueCreate(20, sizeof(mavlink_message_t));

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(wifi_ap_init());
    ESP_ERROR_CHECK(bt_spp_server_init());

    xTaskCreate(indicator_task, "Indicator Task", 1024, NULL, 20, NULL);
    xTaskCreate(bt_mavlink_outgoing_task, "BT Mavlink Task", 4096, NULL, 9, NULL);
    xTaskCreate(mavlink_handler_task, "Mavlink Task", 4096, NULL, 10, NULL);
    xTaskCreate(server_task, "Server Task", 4096, NULL, 15, NULL);

    for(;;) {
        ESP_LOGI(TAG, "Messages; in: %llu, out: %llu", msg_cnt_in, msg_cnt_out);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

static esp_err_t wifi_ap_init() {
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

static esp_err_t bt_spp_server_init() {
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_err_t ret;

    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
        ESP_LOGE(TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }

    if ((ret = esp_bluedroid_init()) != ESP_OK) {
        ESP_LOGE(TAG, "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }

    if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(TAG, "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }

    if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK) {
        ESP_LOGE(TAG, "%s gap register failed: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }

    if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK) {
        ESP_LOGE(TAG, "%s spp register failed: %s\n", __func__, esp_err_to_name(ret));
        return ret ;
    }

    if ((ret = esp_spp_init(esp_spp_mode)) != ESP_OK) {
        ESP_LOGE(TAG, "%s spp init failed: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}
