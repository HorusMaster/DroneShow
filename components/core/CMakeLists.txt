file(GLOB_RECURSE MODULE_SRCS "./modules/src/*.c")
file(GLOB_RECURSE HAL_SRCS "./hal/src/*.c")
file(GLOB_RECURSE UTILS_SRCS "./utils/src/*.c")

idf_component_register(
    SRCS ${MODULE_SRCS} ${HAL_SRCS} ${UTILS_SRCS}
    INCLUDE_DIRS "./modules/include" "./hal/include" "./utils/include" "../config/include" "../lib/dsp_lib/include" "../platform/include"
    REQUIRES driver esp_timer esp_wifi mqtt)