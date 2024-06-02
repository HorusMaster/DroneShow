#ifndef ESC_CONTROL_H
#define ESC_CONTROL_H

#include "driver/dshot.h"

#define ESC_GPIO_PIN_1 18
#define ESC_GPIO_PIN_2 19
#define ESC_GPIO_PIN_3 23
#define ESC_GPIO_PIN_4 5
#define DSHOT_SPEED DSHOT150

void init_escs(void);
void set_esc_throttle(uint16_t throttle);

#endif // ESC_CONTROL_H
