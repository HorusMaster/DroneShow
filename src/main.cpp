#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "mpu6050.h"

#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define MPU6050_ADDR 0x68

#define BLINK_GPIO GPIO_NUM_2

static mpu6050_handle_t mpu6050 = NULL;
static const char *TAG = "mpu6050";

void blink_task(void *pvParameter) {
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    while (1) {
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}


void mpu6050_task(void *pvParameters) {   
    mpu6050_acce_value_t acce;
    mpu6050_gyro_value_t gyro;
    mpu6050_temp_value_t temp;
    while (1) {
        mpu6050_get_acce(mpu6050, &acce);
        ESP_LOGI(TAG, "acce_x:%.2f, acce_y:%.2f, acce_z:%.2f", acce.acce_x, acce.acce_y, acce.acce_z);
        mpu6050_get_gyro(mpu6050, &gyro);
        ESP_LOGI(TAG, "gyro_x:%.2f, gyro_y:%.2f, gyro_z:%.2f", gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);
        mpu6050_get_temp(mpu6050, &temp);
        ESP_LOGI(TAG, "t:%.2f \n", temp.temp);
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

    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);    
}

/**
 * @brief i2c master initialization
 */
static void i2c_sensor_mpu6050_init(void)
{
    //esp_err_t ret;
    i2c_bus_init();
    mpu6050 = mpu6050_create(I2C_MASTER_NUM, MPU6050_I2C_ADDRESS);
    mpu6050_config(mpu6050, ACCE_FS_4G, GYRO_FS_500DPS);      
    mpu6050_wake_up(mpu6050);
}

extern "C" void app_main() {
    esp_err_t ret;
    uint8_t mpu6050_deviceid;     

    ESP_LOGI(TAG, "Initializing I2C...");      
    i2c_sensor_mpu6050_init();    
    ret = mpu6050_get_deviceid(mpu6050, &mpu6050_deviceid);    
    ESP_LOGI(TAG, "mpuinit  %i", ret);        
    xTaskCreate(blink_task, "blink_task", 1024, NULL, 5, NULL);   
    xTaskCreate(mpu6050_task, "mpu6050_task", 4096, NULL, 5, NULL);
    //mpu6050_delete(mpu6050);
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
