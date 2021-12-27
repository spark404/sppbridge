//
// Created by Hugo Trippaers on 26/12/2021.
//
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"

#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"

#define BUILTIN_LED_PIN 2

#define GPIO_HIGH 1
#define GPIO_LOW 0

#define TAG "app_main"

static const esp_bd_addr_t remote_device = { 0xc2, 0x2c, 0x1b, 0x04, 0x0f, 0xe0};

static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_master = ESP_SPP_ROLE_MASTER;

static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;
esp_bd_addr_t peer_bd_addr = {0};
static uint8_t peer_bdname_len;
static char peer_bdname[ESP_BT_GAP_MAX_BDNAME_LEN + 1];
static struct timeval time_new, time_old;
static long data_num = 0;

static const esp_bt_inq_mode_t inq_mode = ESP_BT_INQ_MODE_GENERAL_INQUIRY;
static const uint8_t inq_len = 30;
static const uint8_t inq_num_rsps = 0;

static bool get_name_from_eir(uint8_t *eir, char *bdname, uint8_t *bdname_len)
{
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
            ESP_LOGI(TAG, "ESP_SPP_INIT_EVT");
            esp_bt_dev_set_device_name("SppBridge");
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            esp_bt_gap_start_discovery(inq_mode, inq_len, inq_num_rsps);
            break;
        case ESP_SPP_UNINIT_EVT:
            ESP_LOGI(TAG, "ESP_SPP_UNINIT_EVT");
            break;
        case ESP_SPP_DISCOVERY_COMP_EVT:
            ESP_LOGI(TAG, "ESP_SPP_DISCOVERY_COMP_EVT status=%d scn_num=%d",param->disc_comp.status, param->disc_comp.scn_num);
            if (param->disc_comp.status == ESP_SPP_SUCCESS) {
                esp_spp_connect(sec_mask, role_master, param->disc_comp.scn[0], peer_bd_addr);
            }
            break;
            break;
        case ESP_SPP_OPEN_EVT:
            ESP_LOGI(TAG, "ESP_SPP_OPEN_EVT");
            esp_spp_write(param->open.handle, 13, (uint8_t *)"HELLO WORLD\r\n");
            break;
        case ESP_SPP_CLOSE_EVT:
            ESP_LOGI(TAG, "ESP_SPP_CLOSE_EVT");
            esp_bt_gap_start_discovery(inq_mode, inq_len, inq_num_rsps); // Restart discovery
            break;
        case ESP_SPP_START_EVT:
            ESP_LOGI(TAG, "ESP_SPP_START_EVT");
            break;
        case ESP_SPP_CL_INIT_EVT:
            ESP_LOGI(TAG, "ESP_SPP_CL_INIT_EVT");
            break;
        case ESP_SPP_DATA_IND_EVT:
            ESP_LOGI(TAG, "ESP_SPP_DATA_IND_EVT");
            ESP_LOGI(TAG, "Received %d bytes for connection %x (status %d)",
                     param->data_ind.len, param->data_ind.handle, param->data_ind.status);
            ESP_LOG_BUFFER_HEX(TAG, param->data_ind.data, param->data_ind.len);
            break;
        case ESP_SPP_CONG_EVT:
            ESP_LOGI(TAG, "ESP_SPP_CONG_EVT");
            break;
        case ESP_SPP_WRITE_EVT:
            ESP_LOGI(TAG, "ESP_SPP_WRITE_EVT");
            break;
        case ESP_SPP_SRV_OPEN_EVT:
            ESP_LOGI(TAG, "ESP_SPP_SRV_OPEN_EVT");
            break;
        case ESP_SPP_SRV_STOP_EVT:
            ESP_LOGI(TAG, "ESP_SPP_SRV_STOP_EVT");
            break;
        default:
            ESP_LOGD(TAG, "Unhandled EVT_SPP %d", event);
            break;
    }
}

static void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
    switch (event) {
        case ESP_BT_GAP_DISC_RES_EVT:
            ESP_LOGI(TAG, "ESP_BT_GAP_DISC_RES_EVT");
            ESP_LOGI(TAG, "Discovered %02x:%02x:%02x:%02x:%02x:%02x (%d properties)",
                     param->disc_res.bda[0], param->disc_res.bda[1],
                     param->disc_res.bda[2], param->disc_res.bda[3],
                     param->disc_res.bda[4], param->disc_res.bda[5],
                     param->disc_res.num_prop
                     );
            for (int i = 0; i < param->disc_res.num_prop; i++){
                esp_bt_gap_dev_prop_t *prop = &param->disc_res.prop[i];
                switch (prop->type) {
                    case ESP_BT_GAP_DEV_PROP_BDNAME:
                        memcpy(peer_bdname, prop->val, prop->len);
                        peer_bdname[prop->len] = '\0';
                        ESP_LOGI(TAG, "  Devicename: %s", peer_bdname);
                        break;
                    case ESP_BT_GAP_DEV_PROP_COD:
                        ESP_LOGI(TAG, "Property ESP_BT_GAP_DEV_PROP_COD present");
                        uint32_t *cod = prop->val;
                        ESP_LOGI(TAG, "  Cod: %d", *cod);
                        break;
                    case ESP_BT_GAP_DEV_PROP_RSSI:
                        ESP_LOGI(TAG, "Property ESP_BT_GAP_DEV_PROP_RSSI present");
                        int8_t *rssi = prop->val;
                        ESP_LOGI(TAG, "  RSSI: %d", *rssi);
                        break;
                    case ESP_BT_GAP_DEV_PROP_EIR:
                        ESP_LOGI(TAG, "Property ESP_BT_GAP_DEV_PROP_EIR present");
                        if (get_name_from_eir(param->disc_res.prop[i].val, peer_bdname, &peer_bdname_len)) {
                            ESP_LOGI(TAG, "  Devicename: %s", peer_bdname);
                        }
                        break;
                    default:
                        ESP_LOGI(TAG, "Property %d present", prop->type);
                        break;
                }
            }

            if (memcmp(param->disc_res.bda, remote_device, 6) == 0) {
                ESP_LOGI(TAG, "Remote device detected, starting SPP discovery");
                memcpy(peer_bd_addr, param->disc_res.bda, ESP_BD_ADDR_LEN);
                esp_spp_start_discovery(peer_bd_addr);
                esp_bt_gap_cancel_discovery();
            }
            break;
        case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
            ESP_LOGI(TAG, "ESP_BT_GAP_DISC_STATE_CHANGED_EVT");
            break;
        case ESP_BT_GAP_RMT_SRVCS_EVT:
            ESP_LOGI(TAG, "ESP_BT_GAP_RMT_SRVCS_EVT");
            break;
        case ESP_BT_GAP_RMT_SRVC_REC_EVT:
            ESP_LOGI(TAG, "ESP_BT_GAP_RMT_SRVC_REC_EVT");
            break;
        case ESP_BT_GAP_AUTH_CMPL_EVT:
            ESP_LOGI(TAG, "ESP_BT_GAP_AUTH_CMPL_EVT");
            ESP_LOGI(TAG, "Authentication status for %02x:%02x:%02x:%02x:%02x:%02x is %d",
                     param->auth_cmpl.bda[0], param->auth_cmpl.bda[1],
                     param->auth_cmpl.bda[2], param->auth_cmpl.bda[3],
                     param->auth_cmpl.bda[4], param->auth_cmpl.bda[5],
                     param->auth_cmpl.stat
            );
            break;
        case ESP_BT_GAP_PIN_REQ_EVT:
            ESP_LOGI(TAG, "ESP_BT_GAP_PIN_REQ_EVT");
            esp_bt_pin_code_t pin_code = {'1','2','3','4'};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
            break;
        case ESP_BT_GAP_CFM_REQ_EVT:
            ESP_LOGI(TAG, "ESP_BT_GAP_CFM_REQ_EVT");
            break;
        case ESP_BT_GAP_KEY_NOTIF_EVT:
            ESP_LOGI(TAG, "ESP_BT_GAP_KEY_NOTIF_EVT");
            break;
        case ESP_BT_GAP_KEY_REQ_EVT:
            ESP_LOGI(TAG, "ESP_BT_GAP_KEY_REQ_EVT");
            break;
        case ESP_BT_GAP_READ_RSSI_DELTA_EVT:
            ESP_LOGI(TAG, "ESP_BT_GAP_READ_RSSI_DELTA_EVT");
            break;
        case ESP_BT_GAP_CONFIG_EIR_DATA_EVT:
            ESP_LOGI(TAG, "ESP_BT_GAP_CONFIG_EIR_DATA_EVT");
            break;
        case ESP_BT_GAP_SET_AFH_CHANNELS_EVT:
            ESP_LOGI(TAG, "ESP_BT_GAP_SET_AFH_CHANNELS_EVT");
            break;
        case ESP_BT_GAP_READ_REMOTE_NAME_EVT:
            ESP_LOGI(TAG, "ESP_BT_GAP_READ_REMOTE_NAME_EVT");
            break;
        case ESP_BT_GAP_MODE_CHG_EVT:
            ESP_LOGI(TAG, "ESP_BT_GAP_MODE_CHG_EVT");
            break;
        case ESP_BT_GAP_REMOVE_BOND_DEV_COMPLETE_EVT:
            ESP_LOGI(TAG, "ESP_BT_GAP_REMOVE_BOND_DEV_COMPLETE_EVT");
            break;
        case ESP_BT_GAP_QOS_CMPL_EVT:
            ESP_LOGI(TAG, "ESP_BT_GAP_QOS_CMPL_EVT");
            break;
        case ESP_BT_GAP_EVT_MAX:
            ESP_LOGI(TAG, "ESP_BT_GAP_EVT_MAX");
            break;
        default:
            ESP_LOGD(TAG, "Unhandled ESP_BT event %d", event);
            break;
    }
}

void app_main(void) {
    // Initialize NVS.
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // Not using BLE, so release the memory
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_err_t ret;

    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
        ESP_LOGE(TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_init()) != ESP_OK) {
        ESP_LOGE(TAG, "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(TAG, "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK) {
        ESP_LOGE(TAG, "%s gap register failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK) {
        ESP_LOGE(TAG, "%s spp register failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_spp_init(esp_spp_mode)) != ESP_OK) {
        ESP_LOGE(TAG, "%s spp init failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    // Enable the onboard LED
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_direction(BUILTIN_LED_PIN, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_level(BUILTIN_LED_PIN, GPIO_LOW));

}