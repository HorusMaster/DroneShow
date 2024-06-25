
#include <string.h>

#include "power_distribution.h"

#include <string.h>
#include "num.h"
#include "platform.h"
#include "dshot.h"
#include "esp_log.h"
#include "stabilizer_types.h"
#include "esp_log.h"
#include "driver/i2c.h"

static const char *TAG = "PowerDistribution";

motor_power_t motorPower;

#ifndef DEFAULT_IDLE_THRUST
#define DEFAULT_IDLE_THRUST 0
#endif

// static uint32_t idleThrust = DEFAULT_IDLE_THRUST;

static void init_escs(void)
{
  dshot_config_t config1 = {
      .gpio_num = ESC_GPIO_PIN_1,
      .type = DSHOT300,
      .clk_src = RMT_CLK_SRC_DEFAULT,
  };
  dshot_init(&config1);

  dshot_config_t config2 = {
      .gpio_num = ESC_GPIO_PIN_2,
      .type = DSHOT300,
      .clk_src = RMT_CLK_SRC_DEFAULT,
  };
  dshot_init(&config2);

  dshot_config_t config3 = {
      .gpio_num = ESC_GPIO_PIN_3,
      .type = DSHOT300,
      .clk_src = RMT_CLK_SRC_DEFAULT,
  };
  dshot_init(&config3);

  dshot_config_t config4 = {
      .gpio_num = ESC_GPIO_PIN_4,
      .type = DSHOT300,
      .clk_src = RMT_CLK_SRC_DEFAULT,
  };
  dshot_init(&config4);

  ESP_LOGI(TAG, "ESCs initialized");
}

void powerDistributionInit(void)
{
  init_escs();
}

bool powerDistributionTest(void)
{
  bool pass = true;

  // pass &= motorsTest();

  return pass;
}

#define limitThrust(VAL) limitUint16(VAL)

void powerStop()
{
  dshot_set_throttle(ESC_GPIO_PIN_1, 0, false);
  dshot_set_throttle(ESC_GPIO_PIN_2, 0, false);
  dshot_set_throttle(ESC_GPIO_PIN_3, 0, false);
  dshot_set_throttle(ESC_GPIO_PIN_4, 0, false);
}

void test_motors()
{
  int test_throttle = 300;                       // Valor de throttle para la prueba
  int test_duration = 3000 / portTICK_PERIOD_MS; // Duración de la prueba en ticks (3 segundos)

  // Testear motor 1
  dshot_set_throttle(ESC_GPIO_PIN_1, test_throttle, false);
  vTaskDelay(test_duration);
  dshot_set_throttle(ESC_GPIO_PIN_1, 0, false);

  // Testear motor 2
  dshot_set_throttle(ESC_GPIO_PIN_2, test_throttle, false);
  vTaskDelay(test_duration);
  dshot_set_throttle(ESC_GPIO_PIN_2, 0, false);

  // Testear motor 3
  dshot_set_throttle(ESC_GPIO_PIN_3, test_throttle, false);
  vTaskDelay(test_duration);
  dshot_set_throttle(ESC_GPIO_PIN_3, 0, false);

  // Testear motor 4
  dshot_set_throttle(ESC_GPIO_PIN_4, test_throttle, false);
  vTaskDelay(test_duration);
  dshot_set_throttle(ESC_GPIO_PIN_4, 0, false);
}

void powerDistribution(const control_t *control, motor_power_t *motorPower)
{
#ifdef QUAD_FORMATION_X
  int16_t r = control->roll / 2.0f;
  int16_t p = control->pitch / 2.0f;
  motorPower.m1 = limitThrust(control->thrust - r + p + control->yaw);
  motorPower.m2 = limitThrust(control->thrust - r - p - control->yaw);
  motorPower.m3 = limitThrust(control->thrust + r - p + control->yaw);
  motorPower.m4 = limitThrust(control->thrust + r + p - control->yaw);
#else // QUAD_FORMATION_NORMAL
  motorPower->m1 = limitThrust(control->thrust + control->pitch +
                               control->yaw);
  motorPower->m2 = limitThrust(control->thrust - control->roll -
                               control->yaw);
  motorPower->m3 = limitThrust(control->thrust - control->pitch +
                               control->yaw);
  motorPower->m4 = limitThrust(control->thrust + control->roll -
                               control->yaw);
#endif
  dshot_set_throttle(ESC_GPIO_PIN_1, motorPower->m1, false); // Esquina superior derecha (CCW)
  dshot_set_throttle(ESC_GPIO_PIN_2, motorPower->m2, false); // Esquina inferior derecha (CW)
  dshot_set_throttle(ESC_GPIO_PIN_3, motorPower->m3, false); // Esquina inferior izquierda (CCW)
  dshot_set_throttle(ESC_GPIO_PIN_4, motorPower->m4, false); // Esquina superior izquierda (CW)
   

    /*
                X
                ^
    Motor 4 (CW)|           Motor 1 (CCW)
    (Top Left)  |           (Top Right)
                +---------+
                |   Head  |
                |         |
                |         |
                |         |
                |   Tail  |
                +--------+ ----> Y
    Motor 3 (CCW)   Motor 2 (CW)
    (Bottom Left)  (Bottom Right)

    Roll positivo (dron se inclina a la derecha):
    Motores 1 y 2 (derecha): Aumentar throttle.
    Motores 3 y 4 (izquierda): Disminuir throttle.

    Pitch negativo (dron baja la nariz y sube la cola):
    Motores 1 y 4 (delanteros): Aumentar throttle.
    Motores 2 y 3 (traseros): Disminuir throttle.

    Yaw positivo (dron gira la nariz a la derecha):
    Motores 1 y 3 (CCW): Aumentar throttle.
    Motores 2 y 4 (CW): Disminuir throttle.
    */
}
