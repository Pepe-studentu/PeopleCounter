idf_component_register(
    SRCS
        "main.c"
        "amg8833.c"
        "detector.c"
        "tracker.c"
        "classifier.c"
        "reporter.c"
    INCLUDE_DIRS "."
    REQUIRES
        driver
        esp_wifi
        esp_event
        esp_netif
        lwip
        nvs_flash
        esp_timer
)
