//
// Created by Hugo Trippaers on 29/12/2021.
//

#ifndef SPPBRIDGE_BT_H
#define SPPBRIDGE_BT_H

#include <stdbool.h>
#include "esp_err.h"

typedef struct {
    char *hwaddr;
    char *pin;
} spp_device;

esp_err_t bt_spp_write(uint8_t *buf, int len);
esp_err_t bt_spp_server_init(spp_device *device);

extern void bt_spp_receive_callback(const uint8_t *data, uint16_t len);
extern void bt_spp_status_callback(bool connected);

#endif //SPPBRIDGE_BT_H
