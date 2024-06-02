#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "mpu6050.h"
#include <math.h>
#include "wifi_module.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_system.h"
#include "esp_netif.h"
#include "mqtt_module.h"
#include <errno.h>
#include "dshot.h"
#include "bmx280.h"

#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000

#define MPU6050_ADDR 0x68
#define BMP280_ADDR 0x76

#define BLINK_GPIO GPIO_NUM_2

static mpu6050_handle_t mpu6050 = NULL;
static bmx280_t *bmx280 = NULL;
static const char *TAG = "main";

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

typedef struct {
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
    .previous_error = 0
};

pid_controller_t pid_pitch = {
    .kp = 1.0,
    .ki = 0.5,
    .kd = 0.1,
    .setpoint = 0.0, // Pitch deseado en grados
    .integral = 0,
    .previous_error = 0
};

pid_controller_t pid_roll = {
    .kp = 1.0,
    .ki = 0.5,
    .kd = 0.1,
    .setpoint = 0.0, // Roll deseado en grados
    .integral = 0,
    .previous_error = 0
};

pid_controller_t pid_yaw = {
    .kp = 1.0,
    .ki = 0.5,
    .kd = 0.1,
    .setpoint = 0.0, // Yaw deseado en grados
    .integral = 0,
    .previous_error = 0
};

float pid_compute(pid_controller_t *pid, float current_value) {
    float error = pid->setpoint - current_value;
    pid->integral += error;
    float derivative = error - pid->previous_error;
    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    pid->previous_error = error;
    return output;
}

static euler_angles_t get_euler_angles(mpu6050_acce_value_t acce, mpu6050_gyro_value_t gyro, float altitude)
{
    euler_angles_t angles;

    // Calculate roll and pitch from accelerometer data
    angles.roll = atan2(acce.acce_y, acce.acce_z) * 180 / M_PI;
    angles.pitch = atan2(-acce.acce_x, sqrt(acce.acce_y * acce.acce_y + acce.acce_z * acce.acce_z)) * 180 / M_PI;

    // Yaw calculation requires integrating the gyroscope data
    static float previous_yaw = 0;
    float dt = 0.5; // Assuming a fixed delay of 500ms between readings

    angles.yaw = previous_yaw + gyro.gyro_z * dt;
    previous_yaw = angles.yaw;

    angles.altitude = altitude;

    return angles;
}

float calculate_altitude(float pressure)
{
    const float sea_level_pressure = 1013.25; // Presión a nivel del mar en hPa
    return 44330.0 * (1.0 - pow(pressure / sea_level_pressure, 0.1903));
}

void mpu6050_task(void *pvParameters)
{
    char message[100];
    mpu6050_acce_value_t acce;
    mpu6050_gyro_value_t gyro;
    mpu6050_temp_value_t temp;    
    // Iniciar los ESCs enviando un throttle de cero por un tiempo

    while (1)
    {       
        float temp_env = 0, pres = 0, hum = 0;
        ESP_ERROR_CHECK(bmx280_readoutFloat(bmx280, &temp_env, &pres, &hum));        
        float pressure_hPa = pres / 100.0; // Convertir de Pa a hPa
        float altitude = calculate_altitude(pressure_hPa);
        mpu6050_get_acce(mpu6050, &acce);       
        mpu6050_get_gyro(mpu6050, &gyro);        
        mpu6050_get_temp(mpu6050, &temp);
        euler_angles_t angles = get_euler_angles(acce, gyro, altitude);
        ESP_LOGI(TAG, "Pitch: %.2f, Roll: %.2f, Yaw: %.2f, Altitude: %.2f, Temp: %.2f", angles.pitch, angles.roll, angles.yaw, angles.altitude, temp_env);

        // Crear mensaje JSON con los valores de pitch, roll y yaw
        snprintf(message, sizeof(message), "{\"pitch\": %.2f, \"roll\": %.2f, \"yaw\": %.2f, \"altitude\": %.2f}", angles.pitch, angles.roll, angles.yaw, angles.altitude);
        // Aquí podrías agregar la lógica para ajustar el throttle según los ángulos calculados.
        // Por ejemplo, puedes mapear los ángulos a valores de throttle y enviar esos valores a los ESCs.
        float pid_output_altitude = pid_compute(&pid_altitude, altitude);
        float pid_output_pitch = pid_compute(&pid_pitch, angles.pitch);
        float pid_output_roll = pid_compute(&pid_roll, angles.roll);
        float pid_output_yaw = pid_compute(&pid_yaw, angles.yaw);

        // Ajustar el throttle y los motores según los outputs de los PIDs
        uint16_t throttle_base = (uint16_t)pid_output_altitude;
        if (throttle_base > 2047) {
            throttle_base = 2047;
        } else if (throttle_base < 0) {
            throttle_base = 0;
        }
        
        uint16_t motor1_throttle = throttle_base + pid_output_pitch - pid_output_roll + pid_output_yaw;
        uint16_t motor2_throttle = throttle_base + pid_output_pitch + pid_output_roll - pid_output_yaw;
        uint16_t motor3_throttle = throttle_base - pid_output_pitch + pid_output_roll + pid_output_yaw;
        uint16_t motor4_throttle = throttle_base - pid_output_pitch - pid_output_roll - pid_output_yaw;

        // Limitar los valores de throttle a 2047
        if (motor1_throttle > 2047) motor1_throttle = 2047;
        if (motor2_throttle > 2047) motor2_throttle = 2047;
        if (motor3_throttle > 2047) motor3_throttle = 2047;
        if (motor4_throttle > 2047) motor4_throttle = 2047;

        // Imprimir valores de throttle de cada motor
        ESP_LOGI(TAG, "Throttle Motors: M1: %d, M2: %d, M3: %d, M4: %d", motor1_throttle, motor2_throttle, motor3_throttle, motor4_throttle);
        
        dshot_set_throttle(ESC_GPIO_PIN_1, motor1_throttle, false);
        dshot_set_throttle(ESC_GPIO_PIN_2, motor2_throttle, false);
        dshot_set_throttle(ESC_GPIO_PIN_3, motor3_throttle, false);
        dshot_set_throttle(ESC_GPIO_PIN_4, motor4_throttle, false);

        // Enviar mensaje a través de MQTT
        // send_message(message);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void i2c_bus_init(void)
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0));
}

/**
 * @brief i2c master initialization
 */
static void i2c_sensor_mpu6050_init(void)
{
    mpu6050 = mpu6050_create(I2C_MASTER_NUM, MPU6050_I2C_ADDRESS);
    mpu6050_config(mpu6050, ACCE_FS_4G, GYRO_FS_500DPS);
    mpu6050_wake_up(mpu6050);
}

static void i2c_sensor_bmp280_init(void)
{   
    bmx280_config_t bmx_cfg = BMX280_DEFAULT_CONFIG;
    bmx280 = bmx280_create(I2C_NUM_0);
    if (!bmx280) { 
        ESP_LOGE("test", "Could not create bmx280 driver.");
        return;
    }
    
    ESP_ERROR_CHECK(bmx280_init(bmx280));    
    ESP_ERROR_CHECK(bmx280_configure(bmx280, &bmx_cfg));
    ESP_ERROR_CHECK(bmx280_setMode(bmx280, BMX280_MODE_FORCE));
    
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

// void esc_task(void *pvParameters)
// {
//     // Incrementar el valor de throttle gradualmente
//     static uint16_t throttle_value = 100; // Iniciar con el valor mínimo para encender el motor
//     while (1)
//     {
//         dshot_set_throttle(ESC_GPIO_PIN_2, throttle_value, false);

//         ESP_LOGI(TAG, "Throttle value: %d", throttle_value);

//         // Incrementar el valor de throttle hasta un máximo de 2047
//         if (throttle_value < 200)
//         {
//             throttle_value += 10; // Incrementar en pasos de 50
//         }
//         else
//         {
//             throttle_value = 100; // Reiniciar a 48 después de alcanzar el máximo
//         }
//         vTaskDelay(pdMS_TO_TICKS(1000)); //
//     }
// }

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
    // init_mqtt();
    i2c_bus_init();
    i2c_sensor_mpu6050_init();
    i2c_sensor_bmp280_init();
    init_wifi();
    init_escs();

    //xTaskCreate(esc_task, "esc_task", 4096, NULL, 5, NULL);
    xTaskCreate(blink_task, "blink_task", 1024, NULL, 5, NULL);
    xTaskCreate(mpu6050_task, "mpu6050_task", 4096, NULL, 5, NULL);
    // mpu6050_delete(mpu6050);
}

// MPU6050 mpu;
// QueueHandle_t mpu6050_data_queue;

// typedef struct {
//     float accel_x;
//     float accel_y;
//     float accel_z;
//     float gyro_x;
//     float gyro_y;
//     float gyro_z;
//     float temperature;
// } mpu6050_data_t;

// void i2c_master_init() {
//     i2c_config_t conf = {
//         .mode = I2C_MODE_MASTER,
//         .sda_io_num = I2C_MASTER_SDA_IO,
//         .sda_pullup_en = GPIO_PULLUP_ENABLE,
//         .scl_io_num = I2C_MASTER_SCL_IO,
//         .scl_pullup_en = GPIO_PULLUP_ENABLE,
//         .master.clk_speed = I2C_MASTER_FREQ_HZ,
//     };
//     i2c_param_config(I2C_MASTER_NUM, &conf);
//     i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
// }

// void mpu6050_task(void *pvParameters) {
//     while (1) {
//         mpu6050_data_t data;

//         sensors_event_t a, g, temp;
//         mpu.getEvent(&a, &g, &temp);

//         data.accel_x = a.acceleration.x;
//         data.accel_y = a.acceleration.y;
//         data.accel_z = a.acceleration.z;
//         data.gyro_x = g.gyro.x;
//         data.gyro_y = g.gyro.y;
//         data.gyro_z = g.gyro.z;
//         data.temperature = temp.temperature;

//         if (xQueueSend(mpu6050_data_queue, &data, portMAX_DELAY) != pdPASS) {
//             ESP_LOGE(TAG, "Failed to send data to queue");
//         }

//         vTaskDelay(pdMS_TO_TICKS(500));
//     }
// }

// void process_data_task(void *pvParameters) {
//     mpu6050_data_t data;
//     while (1) {
//         if (xQueueReceive(mpu6050_data_queue, &data, portMAX_DELAY) == pdPASS) {
//             ESP_LOGI(TAG, "Accel X: %.2f, Y: %.2f, Z: %.2f", data.accel_x, data.accel_y, data.accel_z);
//             ESP_LOGI(TAG, "Gyro X: %.2f, Y: %.2f, Z: %.2f", data.gyro_x, data.gyro_y, data.gyro_z);
//             ESP_LOGI(TAG, "Temp: %.2f degC", data.temperature);
//         }
//     }
// }

// extern "C" void app_main() {
//     ESP_LOGI(TAG, "Initializing I2C...");
//     i2c_master_init();

//     ESP_LOGI(TAG, "Initializing MPU6050...");
//     if (!mpu.begin(MPU6050_ADDR, I2C_MASTER_NUM)) {
//         ESP_LOGE(TAG, "Failed to initialize MPU6050!");
//         while (1) {
//             vTaskDelay(pdMS_TO_TICKS(10));
//         }
//     }

//     mpu.setAccelerometerRange(MPU6050_RANGE_2G);
//     mpu.setGyroRange(MPU6050_RANGE_250DEG);

//     mpu6050_data_queue = xQueueCreate(10, sizeof(mpu6050_data_t));
//     if (mpu6050_data_queue == NULL) {
//         ESP_LOGE(TAG, "Failed to create queue");
//         while (1) {
//             vTaskDelay(pdMS_TO_TICKS(10));
//         }
//     }

//     xTaskCreate(mpu6050_task, "mpu6050_task", 2048, NULL, 5, NULL);
//     xTaskCreate(process_data_task, "process_data_task", 2048, NULL, 5, NULL);
// }
