#include "sensors_icm_20948_bmp_280.h"
#include "10Dof_IMU.h"
#include "driver/i2c.h"
#include "i2c_config.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/queue.h"
#include "static_mem.h"
#include "config.h"
#include "stm32_legacy.h"

static const char *TAG = "sensor_icm_20948_bmp_280";

static bool isInit = false;

static QueueHandle_t accelerometerDataQueue;
STATIC_MEM_QUEUE_ALLOC(accelerometerDataQueue, 1, sizeof(Axis3f));
static QueueHandle_t gyroDataQueue;
STATIC_MEM_QUEUE_ALLOC(gyroDataQueue, 1, sizeof(Axis3f));
static QueueHandle_t magnetometerDataQueue;
STATIC_MEM_QUEUE_ALLOC(magnetometerDataQueue, 1, sizeof(Axis3f));
static QueueHandle_t barometerDataQueue;
STATIC_MEM_QUEUE_ALLOC(barometerDataQueue, 1, sizeof(baro_t));

STATIC_MEM_TASK_ALLOC(sensorsTask, SENSORS_TASK_STACKSIZE);

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

static void sensorsDeviceInit(void)
{
    IMU_EN_SENSOR_TYPE enMotionSensorType, enPressureType;
    i2c_master_init();
    imuInit(&enMotionSensorType, &enPressureType);
}

static void sensorsTask(void *arg)
{
    IMU_ST_ANGLES_DATA stAngles;
    IMU_ST_SENSOR_DATA stGyroRawData;
    IMU_ST_SENSOR_DATA stAccelRawData;
    IMU_ST_SENSOR_DATA stMagnRawData;
    while (1)
    {
        imuDataGet(&stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);
        ESP_LOGI(TAG, "Pitch: %f, Roll: %f, Yaw: %f", stAngles.fPitch, stAngles.fRoll, stAngles.fYaw);
        vTaskDelay(M2T(500));
    }
}

static void sensorsTaskInit(void)
{
    accelerometerDataQueue = STATIC_MEM_QUEUE_CREATE(accelerometerDataQueue);
    gyroDataQueue = STATIC_MEM_QUEUE_CREATE(gyroDataQueue);
    magnetometerDataQueue = STATIC_MEM_QUEUE_CREATE(magnetometerDataQueue);
    barometerDataQueue = STATIC_MEM_QUEUE_CREATE(barometerDataQueue);

    STATIC_MEM_TASK_CREATE(sensorsTask, sensorsTask, SENSORS_TASK_NAME, NULL, SENSORS_TASK_PRI);
    ESP_LOGI(TAG, "sensorsTask created");
}



void sensorsICM20948BMP280Init(void)
{
    if (isInit)
    {
        return;
    }

    sensorsDeviceInit();
    sensorsTaskInit();
    isInit = true;
}
