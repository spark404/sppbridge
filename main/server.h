//
// Created by Hugo Trippaers on 28/12/2021.
//

#ifndef SPPBRIDGE_SERVER_H
#define SPPBRIDGE_SERVER_H

typedef struct {
    int connection_active;
    int socket;
    char client_addr_string[128];
} esp_rtsp_server_connection_t;

void server_task(void *pvParameters);
esp_err_t server_write(void *data, uint16_t len);

#endif //SPPBRIDGE_SERVER_H
