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
    float motor1;
    float motor2;
    float motor3;
    float motor4;
    float pidpitch;
    float pidroll;
    float pidyaw;
    float pidalt;
} TELEMETRY_Data;

TELEMETRY_Data tmd; // Telemetry data

void pid_init(PIDController *pid, float kP, float kI, float kD)
{
    pid->kP = kP;
    pid->kI = kI;
    pid->kD = kD;
    pid->previous_error = 0;
    pid->integral = 0;
}

void mqtt_task(void *pvParameters)
{
    char message[512];
    while (1)
    {
        if (isnan(tmd.pidpitch) || isnan(tmd.pidroll) || isnan(tmd.pidyaw) ||
            isnan(tmd.pidalt))
        {
            vTaskDelay(pdMS_TO_TICKS(100)); // Increased delay to allow sufficient processing time
            continue;
        }
        snprintf(message, sizeof(message),
                 "{\"pitch\": %.2f, \"roll\": %.2f, \"yaw\": %.2f, \"altitude\": %.2f, "
                 "\"motor1\": %.2f, \"motor2\": %.2f, \"motor3\": %.2f, \"motor4\": %.2f, "
                 "\"pidpitch\": %.2f, \"pidroll\": %.2f, \"pidyaw\": %.2f, \"pidalt\": %.2f}",
                 tmd.pitch, tmd.roll, tmd.yaw, tmd.altitude,
                 tmd.motor1, tmd.motor2, tmd.motor3, tmd.motor4,
                 tmd.pidpitch, tmd.pidroll, tmd.pidyaw, tmd.pidalt);
        send_message(message);
        ESP_LOGI(TAG, "Switching to bank 3----------------------------------------------------------------");
        vTaskDelay(pdMS_TO_TICKS(200)); // Delay de 1000 ms (1 segundo) para el envío de mensajes MQTT
    }
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

float pid_compute(PIDController *pid, float setpoint, float measured)
{
    TickType_t now = xTaskGetTickCount();
    // Calculate the time difference (dt) in seconds
    float dt = (now - pid->lastTime) / (float)portTICK_PERIOD_MS / 1000.0;
    if (dt <= 0.0f)
    {
        dt = 1e-6; // Assign a small non-zero value to prevent division by zero
    }
    pid->lastTime = now;
    float error = setpoint - measured;
    pid->integral += error * dt;
    float derivative = (error - pid->previous_error) / dt;
    pid->previous_error = error;

    // Calculate the PID output
    float output = (pid->kP * error) + (pid->kI * pid->integral) + (pid->kD * derivative);
    // ESP_LOGI("PID", "Setpoint: %.2f, Measured: %.2f, Error: %.2f, Integral: %.2f, Derivative: %.2f, Output: %.2f",
    //          setpoint, measured, error, pid->integral, derivative, output);
    return output;
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
    int base_throttle = 710; // Ejemplo de valor base de throttle
    int max_throttle = 1000; // Ejemplo de valor máximo de throttle
    int min_throttle = 0;    // Ejemplo de valor mínimo de throttle

    // Inicializar controladores PID
    PIDController pid_pitch, pid_roll, pid_yaw, pid_altitude;
    pid_init(&pid_pitch, 0.5, 0.1, 0.1);
    pid_init(&pid_roll, 0.5, 0.1, 0.1);
    pid_init(&pid_yaw, 0.1, 0.0, 0.0);
    pid_init(&pid_altitude, 1.0, 0.0, 0.0);

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(20)); // 20 ms delay for 50 Hz frequency
        imuDataGet(&stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);
        pressSensorDataGet(&s32TemperatureVal, &s32PressureVal, &s32AltitudeVal);
        float current_altitude = (float)s32AltitudeVal / 100.0; // Convertir la altitud a metros
        float altitude_setpoint = current_altitude + 0.0;       // Setpoint de altitud en metros, un metro por encima de la altitud actual

        tmd.pitch = -stAngles.fPitch;
        tmd.roll = stAngles.fRoll;
        tmd.yaw = stAngles.fYaw;
        tmd.altitude = current_altitude;

        tmd.pidpitch = pid_compute(&pid_pitch, 0.0, tmd.pitch);
        tmd.pidroll = pid_compute(&pid_roll, 0.0, tmd.roll);
        tmd.pidyaw = pid_compute(&pid_yaw, -166.0, tmd.yaw);
        tmd.pidalt = pid_compute(&pid_altitude, altitude_setpoint, current_altitude);

        // ESP_LOGI(TAG, "PID Outputs: Pitch: %.2f, Roll: %.2f, Yaw: %.2f, Altitude: %.2f", tmd.pidpitch, tmd.pidroll, tmd.pidyaw, tmd.pidalt);

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

        motor1_throttle = base_throttle + tmd.pidpitch - tmd.pidroll + tmd.pidyaw + tmd.pidalt; // Esquina superior derecha (CCW)
        motor2_throttle = base_throttle - tmd.pidpitch - tmd.pidroll - tmd.pidyaw + tmd.pidalt; // Esquina superior izquierda (CW)
        motor3_throttle = base_throttle - tmd.pidpitch + tmd.pidroll + tmd.pidyaw + tmd.pidalt; // Esquina inferior izquierda (CCW)
        motor4_throttle = base_throttle + tmd.pidpitch + tmd.pidroll - tmd.pidyaw + tmd.pidalt; // Esquina inferior derecha (CW)

        // Limitar los valores de throttle entre 0 y el máximo permitido
        tmd.motor1 = fmax(min_throttle, fmin(max_throttle, motor1_throttle));
        tmd.motor2 = fmax(min_throttle, fmin(max_throttle, motor2_throttle));
        tmd.motor3 = fmax(min_throttle, fmin(max_throttle, motor3_throttle));
        tmd.motor4 = fmax(min_throttle, fmin(max_throttle, motor4_throttle));

        // ESP_LOGI(TAG, "Motor Throttles: Motor1: %d, Motor2: %d, Motor3: %d, Motor4: %d", motor1_throttle, motor2_throttle, motor3_throttle, motor4_throttle);
        if (get_restart_escs())
        {
            init_escs();
            test_motors();
            set_restart_escs(false);
        }

        if (get_full_stop()) // EMERGENCY STOP
        {
            dshot_set_throttle(ESC_GPIO_PIN_1, 0, false);
            dshot_set_throttle(ESC_GPIO_PIN_2, 0, false);
            dshot_set_throttle(ESC_GPIO_PIN_3, 0, false);
            dshot_set_throttle(ESC_GPIO_PIN_4, 0, false);
            continue;
        }

        // Enviar el throttle a los ESCs
        dshot_set_throttle(ESC_GPIO_PIN_1, tmd.motor1, false); // Esquina superior derecha (CCW)
        dshot_set_throttle(ESC_GPIO_PIN_2, tmd.motor2, false); // Esquina inferior derecha (CW)
        dshot_set_throttle(ESC_GPIO_PIN_3, tmd.motor3, false); // Esquina inferior izquierda (CCW)
        dshot_set_throttle(ESC_GPIO_PIN_4, tmd.motor4, false); // Esquina superior izquierda (CW)
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
