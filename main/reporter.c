#include "reporter.h"
#include "config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include <string.h>
#include <stdio.h>
#include <sys/time.h>

static const char *TAG = "reporter";

typedef struct {
    Classification event;
    int64_t        timestamp_ms;
} ReporterMsg;

#define QUEUE_DEPTH  16

static QueueHandle_t s_queue;

/* Returns true if `tv_sec` looks like a real Unix timestamp (year >= 2020). */
static bool sntp_synced(const struct timeval *tv)
{
    return tv->tv_sec > 1577836800LL;  /* 2020-01-01 00:00:00 UTC */
}

static int open_connection(void)
{
    struct addrinfo hints = {
        .ai_family   = AF_INET,
        .ai_socktype = SOCK_STREAM,
    };
    struct addrinfo *res = NULL;

    char port_str[8];
    snprintf(port_str, sizeof(port_str), "%d", SERVER_PORT);

    if (getaddrinfo(SERVER_HOST, port_str, &hints, &res) != 0 || !res) {
        ESP_LOGW(TAG, "DNS lookup failed for %s", SERVER_HOST);
        return -1;
    }

    int sock = socket(res->ai_family, res->ai_socktype, 0);
    if (sock < 0) {
        freeaddrinfo(res);
        return -1;
    }

    int flag = 1;
    setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

    if (connect(sock, res->ai_addr, res->ai_addrlen) != 0) {
        ESP_LOGW(TAG, "Connect to %s:%d failed", SERVER_HOST, SERVER_PORT);
        close(sock);
        freeaddrinfo(res);
        return -1;
    }

    freeaddrinfo(res);
    ESP_LOGI(TAG, "Connected to %s:%d", SERVER_HOST, SERVER_PORT);
    return sock;
}

static void reporter_task(void *arg)
{
    int sock = -1;
    ReporterMsg msg;

    for (;;) {
        if (sock < 0) {
            sock = open_connection();
            if (sock < 0) {
                vTaskDelay(pdMS_TO_TICKS(TCP_RECONNECT_MS));
                continue;
            }
        }

        if (xQueueReceive(s_queue, &msg, pdMS_TO_TICKS(200)) == pdTRUE) {
            /* Format: "IN 1746270000123\n" or "BOOT_IN 12345\n" */
            struct timeval tv;
            gettimeofday(&tv, NULL);
            const char *prefix = sntp_synced(&tv) ? "" : "BOOT_";
            const char *label  = (msg.event == CLASS_IN) ? "IN" : "OUT";

            char line[48];
            int len = snprintf(line, sizeof(line), "%s%s %lld\n",
                               prefix, label, (long long)msg.timestamp_ms);

            if (send(sock, line, len, 0) < 0) {
                ESP_LOGW(TAG, "Send failed, reconnecting");
                close(sock);
                sock = -1;
            }
        }
        /* no else: loop back, keep connection alive */
    }
}

void reporter_init(void)
{
    s_queue = xQueueCreate(QUEUE_DEPTH, sizeof(ReporterMsg));
    xTaskCreatePinnedToCore(reporter_task, "reporter", 4096, NULL, 4, NULL, 0);
}

void reporter_send(Classification event, int64_t timestamp_ms)
{
    ReporterMsg msg = { event, timestamp_ms };
    xQueueSend(s_queue, &msg, 0);  /* drop if full */
}
