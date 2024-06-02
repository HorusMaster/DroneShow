#ifndef DSHOT_H
#define DSHOT_H

#include "driver/gpio.h"
#include "esp_err.h"

#define ESC_GPIO_PIN_1 18
#define ESC_GPIO_PIN_2 19
#define ESC_GPIO_PIN_3 23
#define ESC_GPIO_PIN_4 5

typedef enum {
    DSHOT150 = 150,
    DSHOT300 = 300,
    DSHOT600 = 600,
    DSHOT1200 = 1200
} dshot_type_t;

typedef struct {
    gpio_num_t gpio_num;
    dshot_type_t type;
    uint32_t clk_src;
} dshot_config_t;

esp_err_t dshot_init(const dshot_config_t *config);
void dshot_set_throttle(gpio_num_t gpio_num, uint16_t throttle, bool telemetry);
void dshot_set_throttle2(gpio_num_t gpio_num, uint16_t throttle, bool telemetry);

#endif // DSHOT_H
