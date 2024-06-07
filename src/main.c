#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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
#include "10Dof_IMU.h"
#include "i2c_config.h"
#include "mqtt_client.h"

#define BLINK_GPIO GPIO_NUM_2
#define STACK_SIZE_LARGE 4096

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
    TickType_t lastTime;
} PIDController;

typedef struct
{
    float pitch;
    float roll;
    float yaw;
    float altitude;
} IMU_Data;

IMU_Data imu_data;

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
    TickType_t now = xTaskGetTickCount();
    float dt = (now - pid->lastTime) / portTICK_PERIOD_MS;
    pid->lastTime = now;

    float error = setpoint - measured;
    pid->integral += error * dt;
    float derivative = (error - pid->previous_error) / dt;
    pid->previous_error = error;

    return (pid->kP * error) + (pid->kI * pid->integral) + (pid->kD * derivative);
}

void mqtt_task(void *pvParameters)
{
    char message[100];
    while (1)
    {
        snprintf(message, sizeof(message), "{\"pitch\": %.2f, \"roll\": %.2f, \"yaw\": %.2f, \"altitude\": %.2f}", imu_data.pitch, imu_data.roll, imu_data.yaw, imu_data.altitude);
        send_message(message);
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay de 1000 ms (1 segundo) para el envío de mensajes MQTT
    }
}

void imu_task(void *pvParameters)
{
    IMU_EN_SENSOR_TYPE enMotionSensorType, enPressureType;
    IMU_ST_ANGLES_DATA stAngles;
    IMU_ST_SENSOR_DATA stGyroRawData;
    IMU_ST_SENSOR_DATA stAccelRawData;
    IMU_ST_SENSOR_DATA stMagnRawData;
    int32_t s32PressureVal = 0, s32TemperatureVal = 0, s32AltitudeVal = 0;
    imuInit(&enMotionSensorType, &enPressureType);

    // Variables para el control de los motores
    int motor1_throttle = 0;
    int motor2_throttle = 0;
    int motor3_throttle = 0;
    int motor4_throttle = 0;
    int base_throttle = 500; // Ejemplo de valor base de throttle
    int max_throttle = 1000; // Ejemplo de valor máximo de throttle
    int min_throttle = 0;    // Ejemplo de valor mínimo de throttle

    // Inicializar controladores PID
    PIDController pid_pitch, pid_roll, pid_yaw, pid_altitude;
    pid_init(&pid_pitch, 1.0, 0.0, 0.0);
    pid_init(&pid_roll, 1.0, 0.0, 0.0);
    pid_init(&pid_yaw, 1.0, 0.0, 0.0);
    pid_init(&pid_altitude, 1.0, 0.0, 0.0);

    while (1)
    {

        imuDataGet(&stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);
        pressSensorDataGet(&s32TemperatureVal, &s32PressureVal, &s32AltitudeVal);
        float current_altitude = (float)s32AltitudeVal / 100.0; // Convertir la altitud a metros
        float altitude_setpoint = current_altitude + 0.0;       // Setpoint de altitud en metros, un metro por encima de la altitud actual

        // snprintf(message, sizeof(message), "{\"pitch\": %.2f, \"roll\": %.2f, \"yaw\": %.2f, \"altitude\": %.2f}", stAngles.fPitch, stAngles.fRoll, stAngles.fYaw, current_altitude);
        // send_message(message);
        imu_data.pitch = -stAngles.fPitch;
        imu_data.roll = stAngles.fRoll;
        imu_data.yaw = stAngles.fYaw;
        imu_data.altitude = current_altitude;

        if (get_restart_escs()){
            init_escs();
            set_restart_escs(false);
        }

        if (get_full_stop()) // EMERGENCY STOP
        {
            dshot_set_throttle(ESC_GPIO_PIN_1, 0, false);
            dshot_set_throttle(ESC_GPIO_PIN_2, 0, false);
            dshot_set_throttle(ESC_GPIO_PIN_3, 0, false);
            dshot_set_throttle(ESC_GPIO_PIN_4, 0, false);
            vTaskDelay(pdMS_TO_TICKS(100)); // Wait 100 ms
            continue;
        }

        // Control de los motores basado en los ángulos
        float pid_output_pitch = pid_compute(&pid_pitch, 0.0, imu_data.pitch);
        float pid_output_roll = pid_compute(&pid_roll, 0.0, imu_data.roll);
        float pid_output_yaw = pid_compute(&pid_yaw, 0.0, imu_data.yaw);
        float pid_output_altitude = pid_compute(&pid_altitude, altitude_setpoint, current_altitude);

        // ESP_LOGI(TAG, "PID Outputs: Pitch: %.2f, Roll: %.2f, Yaw: %.2f, Altitude: %.2f", pid_output_pitch, pid_output_roll, pid_output_yaw, pid_output_altitude);

        // Cálculo del throttle para cada motor usando las salidas PID
        motor1_throttle = base_throttle - pid_output_pitch + pid_output_roll + pid_output_yaw + pid_output_altitude;
        motor2_throttle = base_throttle + pid_output_pitch + pid_output_roll - pid_output_yaw + pid_output_altitude;
        motor3_throttle = base_throttle + pid_output_pitch - pid_output_roll + pid_output_yaw + pid_output_altitude;
        motor4_throttle = base_throttle - pid_output_pitch - pid_output_roll - pid_output_yaw + pid_output_altitude;

        // Limitar los valores de throttle entre 0 y el máximo permitido
        motor1_throttle = fmax(min_throttle, fmin(max_throttle, motor1_throttle));
        motor2_throttle = fmax(min_throttle, fmin(max_throttle, motor2_throttle));
        motor3_throttle = fmax(min_throttle, fmin(max_throttle, motor3_throttle));
        motor4_throttle = fmax(min_throttle, fmin(max_throttle, motor4_throttle));

        // ESP_LOGI(TAG, "Motor Throttles: Motor1: %d, Motor2: %d, Motor3: %d, Motor4: %d", motor1_throttle, motor2_throttle, motor3_throttle, motor4_throttle);

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
    init_wifi();
    init_mqtt();

    xTaskCreate(blink_task, "blink_task", 1024, NULL, 5, NULL);
    xTaskCreate(imu_task, "imu_task", 4096, NULL, 5, NULL);
    xTaskCreate(mqtt_task, "mqtt_task", STACK_SIZE_LARGE, NULL, 5, NULL);
}
