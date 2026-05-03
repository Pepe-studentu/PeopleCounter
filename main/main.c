#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_sntp.h"
#include "esp_timer.h"
#include "config.h"
#include "amg8833.h"
#include "detector.h"
#include "tracker.h"
#include "classifier.h"
#include "reporter.h"
#include <sys/time.h>

static const char *TAG = "main";

/* ------------------------------------------------------------------ */
/* WiFi                                                               */
/* ------------------------------------------------------------------ */

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static EventGroupHandle_t s_wifi_event_group;
static int s_retry_count = 0;
#define WIFI_MAX_RETRY 10

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_count < WIFI_MAX_RETRY) {
            esp_wifi_connect();
            s_retry_count++;
            ESP_LOGW(TAG, "WiFi retry %d/%d", s_retry_count, WIFI_MAX_RETRY);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *evt = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&evt->ip_info.ip));
        s_retry_count = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t h_any, h_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL, &h_any));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL, &h_ip));

    wifi_config_t wifi_cfg = {
        .sta = {
            .ssid     = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE, pdFALSE, portMAX_DELAY);
    if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "WiFi connection failed");
    }
}

/* ------------------------------------------------------------------ */
/* SNTP                                                               */
/* ------------------------------------------------------------------ */

static void sntp_init_simple(void)
{
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();
    /* Don't block here; reporter uses a runtime sync-check per event. */
    ESP_LOGI(TAG, "SNTP started");
}

/* ------------------------------------------------------------------ */
/* Timestamp helper (shared with reporter via sys/time.h)             */
/* ------------------------------------------------------------------ */

static int64_t get_timestamp_ms(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    /* If SNTP has not synced yet, tv_sec will be near epoch (< 2020).
       Fall back to esp_timer*/
    if (tv.tv_sec > 1577836800LL) {
        return (int64_t)tv.tv_sec * 1000LL + (int64_t)(tv.tv_usec / 1000);
    }
    return (int64_t)(esp_timer_get_time() / 1000);
}

/* ------------------------------------------------------------------ */
/* Pipeline task                                                      */
/* ------------------------------------------------------------------ */

static void pipeline_task(void *arg)
{
    static float         frame[64];
    static BlobDetection dets[MAX_BLOBS];
    static Track         dead[MAX_TRACKS];
    static MultiTracker  tracker;

    tracker_init(&tracker);

    const TickType_t period_ticks = pdMS_TO_TICKS(1000 / AMG_FRAME_RATE_HZ);
    TickType_t       last_wake    = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&last_wake, period_ticks);

        if (!amg_read_pixels(AMG_I2C_PORT, frame)) {
            ESP_LOGW(TAG, "Sensor read failed");
            continue;
        }

        int n_dets = detector_run(frame, dets);
        int n_dead = tracker_update(&tracker, dets, n_dets, dead);

        for (int i = 0; i < n_dead; i++) {
            Classification cls = classifier_run(&dead[i]);
            if (cls != CLASS_NOISE) {
                reporter_send(cls, get_timestamp_ms());
            }
        }
    }
}

/* ------------------------------------------------------------------ */
/* Entry point                                                        */
/* ------------------------------------------------------------------ */

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_sta();
    sntp_init_simple();
    reporter_init();

    if (!amg_init(AMG_I2C_PORT)) {
        ESP_LOGE(TAG, "AMG8833 not found on I2C bus — halting");
        for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG, "AMG8833 ready");

    xTaskCreatePinnedToCore(pipeline_task, "pipeline", 8192, NULL, 5, NULL, 1);
}
