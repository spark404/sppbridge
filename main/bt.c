//
// Created by Hugo Trippaers on 29/12/2021.
//

#include "bt.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"
#include "esp_log.h"

#define TAG "app_bt"

static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_master = ESP_SPP_ROLE_MASTER;
static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;

static uint32_t handle;
static esp_bd_addr_t peer_device_addr;
static esp_bt_pin_code_t peer_device_pin;
static SemaphoreHandle_t bt_write_semaphore;

static void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);
static esp_err_t hwaddr_string_to_bin(const char* hwaddr_str, esp_bd_addr_t hwaddr);
static esp_err_t pin_string_to_bin(const char *pin_str, esp_bt_pin_code_t pin);
static esp_err_t restart_discovery();

#ifdef USE_DISCOVERY
esp_bd_addr_t peer_bd_addr = {0};
static uint8_t peer_bdname_len;
static char peer_bdname[ESP_BT_GAP_MAX_BDNAME_LEN + 1];

static const esp_bt_inq_mode_t inq_mode = ESP_BT_INQ_MODE_GENERAL_INQUIRY;
static const uint8_t inq_len = 30;
static const uint8_t inq_num_rsps = 0;
#endif

esp_err_t bt_spp_server_init(spp_device *device) {
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_err_t ret;

    if ((ret = hwaddr_string_to_bin(device->hwaddr, peer_device_addr)) != ESP_OK) {
        ESP_LOGE(TAG, "%s hwaddr conversion failed: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }

    if ((ret = pin_string_to_bin(device->pin, peer_device_pin)) != ESP_OK) {
        ESP_LOGE(TAG, "%s pin conversion failed: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }

    bt_write_semaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(bt_write_semaphore);


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

esp_err_t bt_spp_write(uint8_t *buf, int len) {
    esp_err_t ret;
    if (handle != 0) {
        if (xSemaphoreTake(bt_write_semaphore, pdMS_TO_TICKS(250)) != pdTRUE) {
            ESP_LOGW(TAG, "%s discarding message, no write semaphore within timeout", __func__);
            return ESP_ERR_TIMEOUT;
        }

        if ((ret = esp_spp_write(handle, len, buf)) != ESP_OK) {
            ESP_LOGE(TAG, "%s spp write failed: %s\n", __func__, esp_err_to_name(ret));
        }
    }

    return ESP_OK;
}

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
    switch (event) {
        case ESP_SPP_INIT_EVT:
            ESP_LOGD(TAG, "ESP_SPP_INIT_EVT; status=%d", param->init.status);
            esp_bt_dev_set_device_name("SppBridge");
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            restart_discovery();
            break;
        case ESP_SPP_DISCOVERY_COMP_EVT:
            ESP_LOGD(TAG, "ESP_SPP_DISCOVERY_COMP_EVT; status=%d ", param->disc_comp.status);
            if (param->disc_comp.status == ESP_SPP_SUCCESS) {
                ESP_LOGD(TAG, "Device discovered, starting connect");
                esp_spp_connect(sec_mask, role_master, param->disc_comp.scn[0], peer_device_addr);
            } else {
                restart_discovery();
            }
            break;
        case ESP_SPP_OPEN_EVT:
            ESP_LOGD(TAG, "ESP_SPP_OPEN_EVT; status=%d", param->open.status);
            esp_spp_write(param->open.handle, 13, (uint8_t *) "HELLO WORLD\r\n");
            handle = param->open.handle;
            bt_spp_status_callback(true);
            break;
        case ESP_SPP_CLOSE_EVT:
            ESP_LOGD(TAG, "ESP_SPP_CLOSE_EVT; status=%d", param->close.status);
            handle = 0;
            bt_spp_status_callback(false);
            restart_discovery();
            break;
        case ESP_SPP_CL_INIT_EVT:
            ESP_LOGD(TAG, "ESP_SPP_CL_INIT_EVT; status=%d", param->cl_init.status);
            struct spp_cl_init_evt_param *cl_init = &param->cl_init;
            ESP_LOGD(TAG, "[Handle %x] Client init status %d, sec_id %d, use_co %d",
                     cl_init->handle, cl_init->status, cl_init->sec_id, cl_init->use_co);
            break;
        case ESP_SPP_DATA_IND_EVT:
            bt_spp_receive_callback(param->data_ind.data, param->data_ind.len);
            break;
        case ESP_SPP_CONG_EVT:
            ESP_LOGD(TAG, "ESP_SPP_CONG_EVT; status=%d, cong=%d", param->cong.status, param->cong.cong);
            if (!param->cong.cong) {
                ESP_LOGD(TAG, "Clearing write semaphore in ESP_SPP_CONG_EVT");
                if (xSemaphoreGive(bt_write_semaphore) != pdTRUE) {
                    ESP_LOGE(TAG, "Failed to return write semaphore from ESP_SPP_CONG_EVT");
                }
            }
            break;
        case ESP_SPP_WRITE_EVT:
            ESP_LOGD(TAG, "ESP_SPP_WRITE_EVT; status=%d, cong=%d", param->write.status, param->write.cong);
            if (param->write.status != ESP_SPP_SUCCESS) {
                ESP_LOGW(TAG, "write failed");
            }
            if (!param->write.cong) {
                // Not congested, clear write flag
                ESP_LOGD(TAG, "Clearing write semaphore in ESP_SPP_WRITE_EVT");
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
#ifdef USE_DISCOVERY
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
#endif
        case ESP_BT_GAP_CONFIG_EIR_DATA_EVT:
            ESP_LOGD(TAG, "ESP_BT_GAP_CONFIG_EIR_DATA_EVT");
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

#ifdef USE_DISCOVERY
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
#endif

static esp_err_t restart_discovery() {
    ESP_LOGD(TAG, "Device not found, restarting discovery");
    return esp_spp_start_discovery(peer_device_addr);
}

static esp_err_t hwaddr_string_to_bin(const char* hwaddr_str, esp_bd_addr_t hwaddr) {
    if (strlen(hwaddr_str) != 17) {
        return ESP_ERR_INVALID_ARG;
    }

    char * pEnd;
    for (int i = 0; i<6; i++) {
        peer_device_addr[i] = strtoul(hwaddr_str + (i * 3), &pEnd, 16);
    }


    return ESP_OK;
}

static esp_err_t pin_string_to_bin(const char *pin_str, esp_bt_pin_code_t pin) {
    if (strlen(pin_str) != 4) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t n = sscanf(pin_str, "%c%c%c%c",
                      &pin[0], &pin[1],
                      &pin[2], &pin[3]);
    if (n != 4) {
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}