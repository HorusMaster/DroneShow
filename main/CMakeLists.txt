idf_component_register(
    SRCS
        "main.c"
        "wifi_module.c"
        "ws_server.c"
    INCLUDE_DIRS "."
    REQUIRES
        driver
        esp_event
        esp_http_server
        esp_netif
        esp_wifi
        nvs_flash
        json
        dshot_component
        imu_component
    EMBED_FILES "index.html"
)
