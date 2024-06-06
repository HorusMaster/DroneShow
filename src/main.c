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
#include "bmx280.h"
#include "icm20948.h"
#include "hmc5883l.h"

#define BLINK_GPIO GPIO_NUM_2

static icm20948_handle_t icm20948 = NULL;
// static bmx280_t *bmx280 = NULL;
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

static esp_err_t icm20948_configure(icm20948_acce_fs_t acce_fs, icm20948_gyro_fs_t gyro_fs)
{
    esp_err_t ret;

    /*
     * One might need to change ICM20948_I2C_ADDRESS to ICM20948_I2C_ADDRESS_1
     * if address pin pulled low (to GND)
     */
    icm20948 = icm20948_create(I2C_MASTER_NUM, ICM20948_I2C_ADDRESS_1);
    if (icm20948 == NULL)
    {
        ESP_LOGE(TAG, "ICM20948 create returned NULL!");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "ICM20948 creation successfull!");

    ret = icm20948_reset(icm20948);
    if (ret != ESP_OK)
        return ret;

    vTaskDelay(10 / portTICK_PERIOD_MS);

    ret = icm20948_wake_up(icm20948);
    if (ret != ESP_OK)
        return ret;

    ret = icm20948_set_bank(icm20948, 0);
    if (ret != ESP_OK)
        return ret;

    uint8_t device_id;
    ret = icm20948_get_deviceid(icm20948, &device_id);
    if (ret != ESP_OK)
        return ret;
    ESP_LOGI(TAG, "0x%02X", device_id);
    if (device_id != ICM20948_WHO_AM_I_VAL)
        return ESP_FAIL;

    ret = icm20948_set_gyro_fs(icm20948, gyro_fs);
    if (ret != ESP_OK)
        return ESP_FAIL;

    ret = icm20948_set_acce_fs(icm20948, acce_fs);
    if (ret != ESP_OK)
        return ESP_FAIL;

    return ret;
}

static euler_angles_t get_euler_angles(icm20948_acce_value_t acce, icm20948_gyro_value_t gyro)
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

    return angles;
}

void icm20948_task(void *pvParameters)
{
    // char message[100];    //FOR MQTT
    esp_err_t ret = icm20948_configure(ACCE_FS_2G, GYRO_FS_1000DPS);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "ICM configuration failure");
        vTaskDelete(NULL);
    }
    ESP_LOGI(TAG, "ICM20948 configuration successfull!");

    int16_t mag_data[3];
    float angle;

    while (1)
    {

        icm20948_acce_value_t acce;
        icm20948_gyro_value_t gyro;

        ret = icm20948_get_acce(icm20948, &acce);
        if (ret == ESP_OK)
            ESP_LOGI(TAG, "ax: %lf ay: %lf az: %lf", acce.acce_x, acce.acce_y, acce.acce_z);
        ret = icm20948_get_gyro(icm20948, &gyro);
        if (ret == ESP_OK)
            ESP_LOGI(TAG, "gx: %lf gy: %lf gz: %lf", gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);

        HMC5883L_DataOutRegister(mag_data);
        // Calculate compass angle
        HMC5883L_Compass(mag_data[0], mag_data[1], &angle);
        // Print magnetometer data and angle
        printf("Magnetometer data: X: %d, Y: %d, Z: %d\n", mag_data[0], mag_data[1], mag_data[2]);
        printf("Compass angle: %0.2f\n", angle);

        // ESP_ERROR_CHECK(bmx280_setMode(bmx280, BMX280_MODE_FORCE));
        // do {
        //     vTaskDelay(pdMS_TO_TICKS(1));
        // } while (bmx280_isSampling(bmx280));
        // float temp_env = 0, pres = 0, hum = 0;
        // esp_err_t ret = bmx280_readoutFloat(bmx280, &temp_env, &pres, &hum);
        // if (ret != ESP_OK) {
        //     ESP_LOGE(TAG, "Failed to read data from BMP280: %s", esp_err_to_name(ret));
        //     continue; // Si falla, continuar al siguiente ciclo
        // }
        // float pressure_hPa = pres / 100.0; // Convertir de Pa a hPa
        // // // TODO filter kalman altitud and others
        // float altitude = calculate_altitude(pressure_hPa);
        // ESP_LOGI(TAG, "Altitude: %.2f", altitude);
        // mpu6050_get_acce(mpu6050, &acce);
        // mpu6050_get_gyro(mpu6050, &gyro);
        // mpu6050_get_temp(mpu6050, &temp);
        // euler_angles_t angles = get_euler_angles(acce, gyro, altitude);
        // ESP_LOGI(TAG, "Pitch: %.2f, Roll: %.2f, Yaw: %.2f, Altitude: %.2f, Temp: %.2f", angles.pitch, angles.roll, angles.yaw, angles.altitude, temp_env);

        // // Crear mensaje JSON con los valores de pitch, roll y yaw
        // snprintf(message, sizeof(message), "{\"pitch\": %.2f, \"roll\": %.2f, \"yaw\": %.2f, \"altitude\": %.2f}", angles.pitch, angles.roll, angles.yaw, angles.altitude);
        // // Aquí podrías agregar la lógica para ajustar el throttle según los ángulos calculados.
        // // Por ejemplo, puedes mapear los ángulos a valores de throttle y enviar esos valores a los ESCs.
        // float pid_output_altitude = pid_compute(&pid_altitude, altitude);
        // float pid_output_pitch = pid_compute(&pid_pitch, angles.pitch);
        // float pid_output_roll = pid_compute(&pid_roll, angles.roll);
        // float pid_output_yaw = pid_compute(&pid_yaw, angles.yaw);

        // // Ajustar el throttle y los motores según los outputs de los PIDs
        // uint16_t throttle_base = (uint16_t)pid_output_altitude;
        // if (throttle_base > 2047) {
        //     throttle_base = 2047;
        // }

        // uint16_t motor1_throttle = throttle_base + pid_output_pitch - pid_output_roll + pid_output_yaw;
        // uint16_t motor2_throttle = throttle_base + pid_output_pitch + pid_output_roll - pid_output_yaw;
        // uint16_t motor3_throttle = throttle_base - pid_output_pitch + pid_output_roll + pid_output_yaw;
        // uint16_t motor4_throttle = throttle_base - pid_output_pitch - pid_output_roll - pid_output_yaw;

        // // Limitar los valores de throttle a 2047
        // if (motor1_throttle > 2047) motor1_throttle = 2047;
        // if (motor2_throttle > 2047) motor2_throttle = 2047;
        // if (motor3_throttle > 2047) motor3_throttle = 2047;
        // if (motor4_throttle > 2047) motor4_throttle = 2047;

        // // Imprimir valores de throttle de cada motor
        // ESP_LOGI(TAG, "Throttle Motors: M1: %d, M2: %d, M3: %d, M4: %d", motor1_throttle, motor2_throttle, motor3_throttle, motor4_throttle);

        // dshot_set_throttle(ESC_GPIO_PIN_1, motor1_throttle, false);
        // dshot_set_throttle(ESC_GPIO_PIN_2, motor2_throttle, false);
        // dshot_set_throttle(ESC_GPIO_PIN_3, motor3_throttle, false);
        // dshot_set_throttle(ESC_GPIO_PIN_4, motor4_throttle, false);

        // // Enviar mensaje a través de MQTT
        // // send_message(message);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// /**
//  * @brief i2c master initialization
//  */
// static void i2c_sensor_mpu6050_init(void)
// {
//     mpu6050 = mpu6050_create(I2C_MASTER_NUM, MPU6050_I2C_ADDRESS);
//     mpu6050_config(mpu6050, ACCE_FS_4G, GYRO_FS_500DPS);
//     mpu6050_wake_up(mpu6050);
// }

// static void i2c_sensor_bmp280_init(void)
// {
//     bmx280_config_t bmx_cfg = BMX280_DEFAULT_CONFIG;
//     bmx280 = bmx280_create(I2C_NUM_0);
//     if (!bmx280) {
//         ESP_LOGE("test", "Could not create bmx280 driver.");
//         return;
//     }

//     ESP_ERROR_CHECK(bmx280_init(bmx280));
//     ESP_ERROR_CHECK(bmx280_configure(bmx280, &bmx_cfg));
//     ESP_ERROR_CHECK(bmx280_setMode(bmx280, BMX280_MODE_FORCE));

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

static void i2c_scanner(void *arg)
{
    int i;
    esp_err_t espRc;
    for (i = 1; i < 127; i++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        espRc = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 10 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        if (espRc == ESP_OK) {
            ESP_LOGI(TAG, "I2C device found at address: 0x%02x", i);
        }
    }
    vTaskDelete(NULL);
}

static esp_err_t i2c_master_read_register(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, size_t length) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_READ, true);
    if (length > 1) {
        i2c_master_read(cmd, data, length - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + length - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
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
    // init_mqtt();
    i2c_master_init();
    HMC5883L_Init();
    // i2c_sensor_mpu6050_init();
    uint8_t data[3];
   
    ret = i2c_master_read_register(HMC5883L_DEVICE_ADDR, 0x0A, data, 3);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "HMC5883L Identification Registers: 0x%02X 0x%02X 0x%02X", data[0], data[1], data[2]);
    } else {
        ESP_LOGE(TAG, "Failed to read identification registers");
    }
    // i2c_sensor_bmp280_init();
    // init_wifi();
    // init_escs();

    // xTaskCreate(esc_task, "esc_task", 4096, NULL, 5, NULL);
    xTaskCreate(i2c_scanner, "i2c_scanner", 1024 * 2, NULL, 10, NULL);
    xTaskCreate(blink_task, "blink_task", 1024, NULL, 5, NULL);
    xTaskCreate(icm20948_task, "mpu6050_task", 4096, NULL, 5, NULL);
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
