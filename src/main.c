#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <math.h>
#include "wifi_module.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_system.h"
#include "esp_netif.h"
#include "mqtt_module.h"
#include <errno.h>
#include "dshot.h"
#include "Waveshare_10DOF_D.h"
#include "i2c_config.h"

#define BLINK_GPIO GPIO_NUM_2
static const char *TAG = "main";

static void i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));
}

void blink_task(void *pvParameter)
{
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    while (1)
    {
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

typedef struct
{
    float pitch;
    float roll;
    float yaw;
    float altitude;
} euler_angles_t;

typedef struct
{
    float kp;
    float ki;
    float kd;
    float setpoint;
    float integral;
    float previous_error;
} pid_controller_t;

pid_controller_t pid_altitude = {
    .kp = 1.0, // Ajusta estos valores según sea necesario
    .ki = 0.5,
    .kd = 0.1,
    .setpoint = 1.0, // Altitud deseada en metros
    .integral = 0,
    .previous_error = 0};

pid_controller_t pid_pitch = {
    .kp = 1.0,
    .ki = 0.5,
    .kd = 0.1,
    .setpoint = 0.0, // Pitch deseado en grados
    .integral = 0,
    .previous_error = 0};

pid_controller_t pid_roll = {
    .kp = 1.0,
    .ki = 0.5,
    .kd = 0.1,
    .setpoint = 0.0, // Roll deseado en grados
    .integral = 0,
    .previous_error = 0};

pid_controller_t pid_yaw = {
    .kp = 1.0,
    .ki = 0.5,
    .kd = 0.1,
    .setpoint = 0.0, // Yaw deseado en grados
    .integral = 0,
    .previous_error = 0};

float pid_compute(pid_controller_t *pid, float current_value)
{
    float error = pid->setpoint - current_value;
    pid->integral += error;
    float derivative = error - pid->previous_error;
    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    pid->previous_error = error;
    return output;
}

float calculate_altitude(float pressure)
{
    const float sea_level_pressure = 1013.25; // Presión a nivel del mar en hPa
    return 44330.0 * (1.0 - pow(pressure / sea_level_pressure, 0.1903));
}

// static euler_angles_t get_euler_angles(icm20948_acce_value_t acce, icm20948_gyro_value_t gyro)
// {
//     euler_angles_t angles;

//     // Calculate roll and pitch from accelerometer data
//     angles.roll = atan2(acce.acce_y, acce.acce_z) * 180 / M_PI;
//     angles.pitch = atan2(-acce.acce_x, sqrt(acce.acce_y * acce.acce_y + acce.acce_z * acce.acce_z)) * 180 / M_PI;

//     // Yaw calculation requires integrating the gyroscope data
//     static float previous_yaw = 0;
//     float dt = 0.5; // Assuming a fixed delay of 500ms between readings

//     angles.yaw = previous_yaw + gyro.gyro_z * dt;
//     previous_yaw = angles.yaw;

//     return angles;
// }

// static void init_escs(void)
// {
//     dshot_config_t config1 = {
//         .gpio_num = ESC_GPIO_PIN_1,
//         .type = DSHOT300,
//         .clk_src = RMT_CLK_SRC_DEFAULT,
//     };
//     dshot_init(&config1);

//     dshot_config_t config2 = {
//         .gpio_num = ESC_GPIO_PIN_2,
//         .type = DSHOT300,
//         .clk_src = RMT_CLK_SRC_DEFAULT,
//     };
//     dshot_init(&config2);

//     dshot_config_t config3 = {
//         .gpio_num = ESC_GPIO_PIN_3,
//         .type = DSHOT300,
//         .clk_src = RMT_CLK_SRC_DEFAULT,
//     };
//     dshot_init(&config3);

//     dshot_config_t config4 = {
//         .gpio_num = ESC_GPIO_PIN_4,
//         .type = DSHOT300,
//         .clk_src = RMT_CLK_SRC_DEFAULT,
//     };
//     dshot_init(&config4);

//     ESP_LOGI(TAG, "ESCs initialized");
//     vTaskDelay(pdMS_TO_TICKS(5000)); // Esperar 5 segundos
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

void imu_task(void *pvParameters)
{
    IMU_EN_SENSOR_TYPE enMotionSensorType, enPressureType;
    IMU_ST_ANGLES_DATA stAngles;
    IMU_ST_SENSOR_DATA stGyroRawData;
    IMU_ST_SENSOR_DATA stAccelRawData;
    IMU_ST_SENSOR_DATA stMagnRawData;
    int32_t s32PressureVal = 0, s32TemperatureVal = 0, s32AltitudeVal = 0;
    // char message[100];    //FOR MQTT
    imuInit(&enMotionSensorType, &enPressureType);

    while (1)
    {
        imuDataGet(&stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);
        pressSensorDataGet(&s32TemperatureVal, &s32PressureVal, &s32AltitudeVal);
        ESP_LOGI(TAG, "Pitch: %.2f, Roll: %.2f, Yaw: %.2f, Altitude: %.2f, Temp: %.2f", stAngles.fPitch, stAngles.fRoll, stAngles.fYaw, (float)s32AltitudeVal / 100, (float)s32TemperatureVal / 100);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void app_main()
{
    esp_err_t ret;
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    i2c_master_init();
    //  init_mqtt();
    //  init_wifi();
    //  init_escs();
    // xTaskCreate(i2c_scanner, "i2c_scanner", 1024 * 2, NULL, 10, NULL);
    xTaskCreate(blink_task, "blink_task", 1024, NULL, 5, NULL);
    xTaskCreate(imu_task, "imu_task", 4096, NULL, 5, NULL);
}
