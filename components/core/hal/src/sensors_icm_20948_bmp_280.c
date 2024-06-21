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
#include "system.h"

static const char *TAG = "sensor_icm_20948_bmp_280";

static bool isInit = false;
static bool isBarometerPresent = true;
static bool isMagnetometerPresent = true;

static QueueHandle_t accelerometerDataQueue;
STATIC_MEM_QUEUE_ALLOC(accelerometerDataQueue, 1, sizeof(Axis3f));
static QueueHandle_t gyroDataQueue;
STATIC_MEM_QUEUE_ALLOC(gyroDataQueue, 1, sizeof(Axis3f));
static QueueHandle_t magnetometerDataQueue;
STATIC_MEM_QUEUE_ALLOC(magnetometerDataQueue, 1, sizeof(Axis3f));
static QueueHandle_t barometerDataQueue;
STATIC_MEM_QUEUE_ALLOC(barometerDataQueue, 1, sizeof(baro_t));

STATIC_MEM_TASK_ALLOC(sensorsTask, SENSORS_TASK_STACKSIZE);

static sensorData_t sensorData;

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

void processSensorData(IMU_ST_SENSOR_DATA *stGyroRawData, IMU_ST_SENSOR_DATA *stAccelRawData)
{
    sensorData.gyro.x = stGyroRawData->s16X;
    sensorData.gyro.y = stGyroRawData->s16Y;
    sensorData.gyro.z = stGyroRawData->s16Z;

    /* TODO sensors step 2.6 Compensate for a miss-aligned accelerometer. */
    sensorData.acc.x = stAccelRawData->s16X;
    sensorData.acc.y = stAccelRawData->s16Y;
    sensorData.acc.z = stAccelRawData->s16Z;
}

void processMagnetometerMeasurements(IMU_ST_SENSOR_DATA *stMagnRawData)
{
    sensorData.mag.x = stMagnRawData->s16X;
    sensorData.mag.y = stMagnRawData->s16Y;
    sensorData.mag.z = stMagnRawData->s16Z;
}

void processBarometerMeasurements(int32_t *s32TemperatureVal, int32_t *s32PressureVal, int32_t *s32AltitudeVal)
{
    sensorData.baro.temperature = *s32TemperatureVal;
    sensorData.baro.pressure = *s32PressureVal;
    sensorData.baro.asl = *s32AltitudeVal;
}

bool sensorsICM20948BMP280ReadGyro(Axis3f *gyro)
{
    return (pdTRUE == xQueueReceive(gyroDataQueue, gyro, 0));
}

bool sensorsICM20948BMP280ReadAcc(Axis3f *acc)
{
    return (pdTRUE == xQueueReceive(accelerometerDataQueue, acc, 0));
}

bool sensorsICM20948BMP280ReadMag(Axis3f *mag)
{
    return (pdTRUE == xQueueReceive(magnetometerDataQueue, mag, 0));
}

bool sensorsICM20948BMP280ReadBaro(baro_t *baro)
{
    return (pdTRUE == xQueueReceive(barometerDataQueue, baro, 0));
}

void sensorsICM20948BMP280Acquire(sensorData_t *sensors, const uint32_t tick)
{

    sensorsReadGyro(&sensors->gyro);
    sensorsReadAcc(&sensors->acc);
    sensorsReadMag(&sensors->mag);
    sensorsReadBaro(&sensors->baro);
    // sensors->interruptTimestamp = sensorData.interruptTimestamp;
}

static void sensorsTask(void *arg)
{
    IMU_ST_ANGLES_DATA stAngles;
    IMU_ST_SENSOR_DATA stGyroRawData;
    IMU_ST_SENSOR_DATA stAccelRawData;
    IMU_ST_SENSOR_DATA stMagnRawData;
    int32_t s32PressureVal = 0, s32TemperatureVal = 0, s32AltitudeVal = 0;
    // systemWaitStart();
    vTaskDelay(M2T(200));
    ESP_LOGI(TAG, "sensorsTask started");
    while (1)
    {
        imuDataGet(&stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);

        processSensorData(&stGyroRawData, &stAccelRawData);
        pressSensorDataGet(&s32TemperatureVal, &s32PressureVal, &s32AltitudeVal);

        if (isMagnetometerPresent)
        {
            processMagnetometerMeasurements(&stMagnRawData);
        }

        if (isBarometerPresent)
        {
            processBarometerMeasurements(&s32TemperatureVal, &s32PressureVal, &s32AltitudeVal);
        }

        /* sensors step 3- queue sensors data  on the output queues */
        xQueueOverwrite(accelerometerDataQueue, &sensorData.acc);
        xQueueOverwrite(gyroDataQueue, &sensorData.gyro);

        if (isMagnetometerPresent)
        {
            xQueueOverwrite(magnetometerDataQueue, &sensorData.mag);
        }

        if (isBarometerPresent)
        {
            xQueueOverwrite(barometerDataQueue, &sensorData.baro);
        }
#ifdef DEBUG_EP2
        ESP_LOGI(TAG, "Pitch: %f, Roll: %f, Yaw: %f", stAngles.fPitch, stAngles.fRoll, stAngles.fYaw);
        ESP_LOGI(TAG, "Temperature: %f, Pressure: %f, Altitude: %f", sensorData.baro.temperature, sensorData.baro.pressure, sensorData.baro.asl);
#endif

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
