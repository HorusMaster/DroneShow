file(GLOB_RECURSE MODULE_SRCS "./modules/src/*.c")
file(GLOB_RECURSE HAL_SRCS "./hal/src/*.c")

idf_component_register(
    SRCS ${MODULE_SRCS} ${HAL_SRCS}
    INCLUDE_DIRS "./modules/include" "./hal/include" "../config/include"
    REQUIRES driver esp_timer esp_wifi
)