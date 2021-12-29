//
// Created by Hugo Trippaers on 28/12/2021.
//
#include <sys/param.h>
#include "esp_log.h"

#include "lwip/err.h"
#include "lwip/sockets.h"

#include "server.h"

#define MAX_CLIENTS 1
#define TAG "server"

#define PORT 5760  // Default for QGroundControl

#define KEEPALIVE_IDLE              5
#define KEEPALIVE_INTERVAL          5
#define KEEPALIVE_COUNT             3

esp_rtsp_server_connection_t connections[MAX_CLIENTS];

extern void server_receive_callback(const uint8_t *data, uint16_t len);

static int create_listening_socket(int port) {
    int listen_sock = socket(AF_INET6, SOCK_STREAM, 0);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        return -1;
    }

    struct in6_addr inaddr_any = IN6ADDR_ANY_INIT;
    struct sockaddr_in6 serv_addr = {
            .sin6_family  = PF_INET6,
            .sin6_addr    = inaddr_any,
            .sin6_port    = htons(PORT)
    };

    int opt = 1;
    if (setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
        ESP_LOGE(TAG, "Unable to set socket SO_REUSEADDR: errno %d", errno);
        goto CLEAN_UP;
    }

    int err = bind(listen_sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr));
    if (err != 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        goto CLEAN_UP;
    }

    err = listen(listen_sock, 1);
    if (err != 0) {
        ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
        goto CLEAN_UP;
    }

    ESP_LOGI(TAG, "Created listening socket on port %d", port);
    return listen_sock;

    CLEAN_UP:
    close(listen_sock);
    return -1;
}

static int server_read_block(esp_rtsp_server_connection_t *connection) {
    uint8_t buffer[1024];
    if (!connection->connection_active) {
        return -1;
    }

    int sock = connection->socket;

    int n = read(sock, buffer, 1023);
    if (n < 0) {
        if (errno == EWOULDBLOCK || errno == EAGAIN) {
            ESP_LOGW(TAG, "Receive timeout");
            return -1;
        } else {
            ESP_LOGE(TAG, "Read error: %d", errno);
            return -1;
        }
    }

    if (n == 0) {
        return n;
    }

    buffer[n] = 0x0;

    ESP_LOGD(TAG, "IN < (%d bytes): %s", n, buffer);
    server_receive_callback(buffer, n);

    return n;
}

static int server_connection_close(esp_rtsp_server_connection_t *connection) {
    ESP_LOGI(TAG, "Closing connection with %s", connection->client_addr_string);
    if (!connection->connection_active) {
        return 0;
    }

    connection->connection_active = false;
    shutdown(connection->socket, 0);
    close(connection->socket);

    memset(connection, 0, sizeof(esp_rtsp_server_connection_t));

    return 0;
}

static int handle_read(esp_rtsp_server_connection_t *connection) {
    if (!connection) {
        ESP_LOGW(TAG, "Read on socket %d, but no registered connection", connection->socket);
        return -1;
    }

    if (!connection->connection_active) {
        ESP_LOGW(TAG, "Read on socket inactive socket %d", connection->socket);
        return -1;
    }

    ssize_t n = server_read_block(connection);
    if (n < 0) {
        ESP_LOGE(TAG, "Read failed");
        server_connection_close(connection);
        return -1;
    }

    if (n == 0) {
        server_connection_close(connection);
        return 0;
    }

    return 0;
}


static esp_err_t server_accept(int listen_sock) {
    int keepAlive = 1;
    int keepIdle = KEEPALIVE_IDLE;
    int keepInterval = KEEPALIVE_INTERVAL;
    int keepCount = KEEPALIVE_COUNT;

    struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
    socklen_t addr_len = sizeof(source_addr);
    int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
        return ESP_FAIL;
    }

    esp_rtsp_server_connection_t *connection = NULL;

    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (!connections[i].connection_active) {
            // claim this connection
            connections[i].connection_active = true;
            connection = &connections[i];
            break;
        }
    }

    if (!connection) {
        ESP_LOGW(TAG, "No free connections");
        shutdown(sock, 0);
        close(sock);
        return ESP_FAIL;
    }

    // Set tcp keepalive option
    setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
    setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
    setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
    setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));

    struct timeval to;
    to.tv_sec = 3;
    to.tv_usec = 0;

    if (setsockopt(sock,SOL_SOCKET,SO_RCVTIMEO,&to,sizeof(to)) < 0) {
        ESP_LOGW(TAG, "Set recv timeout failed");
        shutdown(sock, 0);
        close(sock);
        return ESP_FAIL;
    }

    // Convert ip address to string
    if (source_addr.ss_family == PF_INET) {
        inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, connection->client_addr_string, sizeof(connection->client_addr_string) - 1);
    }
    else if (source_addr.ss_family == PF_INET6) {
        inet6_ntoa_r(((struct sockaddr_in6 *)&source_addr)->sin6_addr, connection->client_addr_string, sizeof(connection->client_addr_string) - 1);
    }
    ESP_LOGI(TAG, "Socket accepted ip address: %s", connection->client_addr_string);

    connection->socket = sock;
    return ESP_OK;
}

esp_err_t server_write(void *data, uint16_t len) {
    if (len <= 0) {
        return ESP_FAIL;
    }

    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (connections[i].connection_active) {
            size_t n = write(connections[i].socket, data, len);
            if (n != len) {
                ESP_LOGW(TAG, "Incomplete write to socket");
            }
        }
    }

    return ESP_OK;
}

void server_task(void *pvParameters) {
    int listen_sock = create_listening_socket(PORT);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Failed to start server_task");
        vTaskDelete(NULL);
    }

    for (;;) {
        int sock_max = listen_sock;

        fd_set read_set;
        FD_ZERO(&read_set);
        FD_SET(listen_sock, &read_set);

        for (int i = 0; i < MAX_CLIENTS; i++) {
            if (connections[i].connection_active) {
                FD_SET(connections[i].socket, &read_set);
                if (connections[i].socket > sock_max) {
                    sock_max = connections[i].socket;
                }
            }
        }

        ESP_LOGD(TAG, "Entering select");
        int n = select(sock_max + 1, &read_set, NULL, NULL, NULL);
        if (n < 0) {
            if (errno == EINTR) {
                ESP_LOGW(TAG, "select interrupted");
                continue;
            }

            ESP_LOGE(TAG, "Failure in select, errno %d", errno);
            break;
        }

        for (int i = 0; i < MAX_CLIENTS; i++) {
            if (FD_ISSET(connections[i].socket, &read_set)) {
                ESP_LOGD(TAG, "Read on connection %d", i);
                handle_read(&connections[i]);
            }
        }

        if (FD_ISSET(listen_sock, &read_set)) {
            ESP_LOGD(TAG, "Read on listen socket");
            esp_err_t err = server_accept(listen_sock);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "Failed to accept connection");
            }
        }
    }

    ESP_LOGI(TAG, "Shutting down listening socket");
    close(listen_sock);
    vTaskDelete(NULL);
}
