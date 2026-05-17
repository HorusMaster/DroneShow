idf_component_register(
    SRCS
        "dshot.c"
        "include/dshot_esc_encoder.c"
    INCLUDE_DIRS "include"
    REQUIRES "esp_driver_rmt" "esp_driver_gpio"
)
