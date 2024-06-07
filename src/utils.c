
// void test_motors()
// {
//     int max_throttle = 1000;
//     int increment = 10;

//     for (int i = 0; i <= max_throttle; i += increment)
//     {
//         dshot_set_throttle(ESC_GPIO_PIN_1, i, false);
//         dshot_set_throttle(ESC_GPIO_PIN_2, i, false);
//         dshot_set_throttle(ESC_GPIO_PIN_3, i, false);
//         dshot_set_throttle(ESC_GPIO_PIN_4, i, false);
//         vTaskDelay(pdMS_TO_TICKS(50)); // Espera 50ms entre cada incremento
//     }

//     vTaskDelay(pdMS_TO_TICKS(5000)); // Mantén el throttle en 1000 por 5 segundos

//     for (int i = max_throttle; i >= 0; i -= increment)
//     {
//         dshot_set_throttle(ESC_GPIO_PIN_1, i, false);
//         dshot_set_throttle(ESC_GPIO_PIN_2, i, false);
//         dshot_set_throttle(ESC_GPIO_PIN_3, i, false);
//         dshot_set_throttle(ESC_GPIO_PIN_4, i, false);
//         vTaskDelay(pdMS_TO_TICKS(50)); // Espera 50ms entre cada decremento
//     }
// }

// static void i2c_scanner(void *arg)
// {
//     int i;
//     esp_err_t espRc;
//     for (i = 1; i < 127; i++)
//     {
//         i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//         i2c_master_start(cmd);
//         i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, true);
//         i2c_master_stop(cmd);
//         espRc = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 10 / portTICK_PERIOD_MS);
//         i2c_cmd_link_delete(cmd);
//         if (espRc == ESP_OK)
//         {
//             ESP_LOGI(TAG, "I2C device found at address: 0x%02x", i);
//         }
//     }
//     vTaskDelete(NULL);
// }