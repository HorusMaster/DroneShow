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
    vTaskDelay(pdMS_TO_TICKS(5000)); // Esperar 5 segundos
}

typedef struct
{
    float kP;
    float kI;
    float kD;
    float previous_error;
    float integral;
} PIDController;

void pid_init(PIDController *pid, float kP, float kI, float kD)
{
    pid->kP = kP;
    pid->kI = kI;
    pid->kD = kD;
    pid->previous_error = 0;
    pid->integral = 0;
}

float pid_compute(PIDController *pid, float setpoint, float measured)
{
    float error = setpoint - measured;
    pid->integral += error;
    float derivative = error - pid->previous_error;
    pid->previous_error = error;
    return (pid->kP * error) + (pid->kI * pid->integral) + (pid->kD * derivative);
}

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

    // PID--------------------------------------------
    // Variables para el control de los motores
    int motor1_throttle = 0;
    int motor2_throttle = 0;
    int motor3_throttle = 0;
    int motor4_throttle = 0;
    int base_throttle = 48;  // Ejemplo de valor base de throttle
    int max_throttle = 1000; // Ejemplo de valor máximo de throttle

    // Inicializar controladores PID
    PIDController pid_pitch, pid_roll, pid_yaw, pid_altitude;
    pid_init(&pid_pitch, 1.0, 0.0, 0.0);
    pid_init(&pid_roll, 1.0, 0.0, 0.0);
    pid_init(&pid_yaw, 1.0, 0.0, 0.0);
    pid_init(&pid_altitude, 1.0, 0.0, 0.0);

    float altitude_setpoint = 1.0; // Setpoint de altitud en metros

    while (1)
    {
        imuDataGet(&stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);
        pressSensorDataGet(&s32TemperatureVal, &s32PressureVal, &s32AltitudeVal);
        float current_altitude = (float)s32AltitudeVal / 100.0; // Convertir la altitud a metros
        ESP_LOGI(TAG, "Pitch: %.2f, Roll: %.2f, Yaw: %.2f, Altitude: %.2f, Temp: %.2f", stAngles.fPitch, stAngles.fRoll, stAngles.fYaw, current_altitude, (float)s32TemperatureVal / 100);

        // Control de los motores basado en los ángulos
        float pid_output_pitch = pid_compute(&pid_pitch, 0.0, stAngles.fPitch);
        float pid_output_roll = pid_compute(&pid_roll, 0.0, stAngles.fRoll);
        float pid_output_yaw = pid_compute(&pid_yaw, 0.0, stAngles.fYaw);
        float pid_output_altitude = pid_compute(&pid_altitude, altitude_setpoint, current_altitude);

        // Cálculo del throttle para cada motor usando las salidas PID
        motor1_throttle = base_throttle - pid_output_pitch + pid_output_roll + pid_output_yaw + pid_output_altitude;
        motor2_throttle = base_throttle + pid_output_pitch + pid_output_roll - pid_output_yaw + pid_output_altitude;
        motor3_throttle = base_throttle + pid_output_pitch - pid_output_roll + pid_output_yaw + pid_output_altitude;
        motor4_throttle = base_throttle - pid_output_pitch - pid_output_roll - pid_output_yaw + pid_output_altitude;

        // Limitar los valores de throttle entre 0 y el máximo permitido
        motor1_throttle = fmax(0, fmin(max_throttle, motor1_throttle));
        motor2_throttle = fmax(0, fmin(max_throttle, motor2_throttle));
        motor3_throttle = fmax(0, fmin(max_throttle, motor3_throttle));
        motor4_throttle = fmax(0, fmin(max_throttle, motor4_throttle));

        ESP_LOGI(TAG, "Motor Throttles: Motor1: %d, Motor2: %d, Motor3: %d, Motor4: %d", motor1_throttle, motor2_throttle, motor3_throttle, motor4_throttle);

        // Enviar el throttle a los ESCs
        dshot_set_throttle(ESC_GPIO_PIN_1, motor1_throttle, false);
        dshot_set_throttle(ESC_GPIO_PIN_2, motor2_throttle, false);
        dshot_set_throttle(ESC_GPIO_PIN_3, motor3_throttle, false);
        dshot_set_throttle(ESC_GPIO_PIN_4, motor4_throttle, false);

        vTaskDelay(pdMS_TO_TICKS(10));
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
    init_escs();
    //  init_mqtt();
    //  init_wifi();
    // xTaskCreate(i2c_scanner, "i2c_scanner", 1024 * 2, NULL, 10, NULL);
    xTaskCreate(blink_task, "blink_task", 1024, NULL, 5, NULL);
    xTaskCreate(imu_task, "imu_task", 4096, NULL, 5, NULL);
}

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