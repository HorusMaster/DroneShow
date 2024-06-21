#include "static_mem.h"
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "stabilizer_types.h"
#include "config.h"
#include "esp_log.h"
#include "stm32_legacy.h"
#include "cf_math.h"
#include "physicalConstants.h"
#include "sensors.h"

static const char *TAG = "estimator_kalman";

static bool isInit = false;

static Axis3f accAccumulator;
static float thrustAccumulator;
static Axis3f gyroAccumulator;
static float baroAslAccumulator;
static uint32_t accAccumulatorCount;
static uint32_t thrustAccumulatorCount;
static uint32_t gyroAccumulatorCount;
static uint32_t baroAccumulatorCount;
static bool quadIsFlying = false;
static uint32_t lastFlightCmd;
static uint32_t takeoffTime;

// NO_DMA_CCM_SAFE_ZERO_INIT static kalmanCoreData_t coreData;

// Distance-to-point measurements
static QueueHandle_t distDataQueue;
STATIC_MEM_QUEUE_ALLOC(distDataQueue, 10, sizeof(distanceMeasurement_t));

static inline bool stateEstimatorHasDistanceMeasurement(distanceMeasurement_t *dist)
{
    return (pdTRUE == xQueueReceive(distDataQueue, dist, 0));
}

// Direct measurements of Crazyflie position
static QueueHandle_t posDataQueue;
STATIC_MEM_QUEUE_ALLOC(posDataQueue, 10, sizeof(positionMeasurement_t));

static inline bool stateEstimatorHasPositionMeasurement(positionMeasurement_t *pos)
{
    return (pdTRUE == xQueueReceive(posDataQueue, pos, 0));
}

// Direct measurements of Crazyflie pose
static QueueHandle_t poseDataQueue;
STATIC_MEM_QUEUE_ALLOC(poseDataQueue, 10, sizeof(poseMeasurement_t));

static inline bool stateEstimatorHasPoseMeasurement(poseMeasurement_t *pose)
{
    return (pdTRUE == xQueueReceive(poseDataQueue, pose, 0));
}

// Measurements of a UWB Tx/Rx
static QueueHandle_t tdoaDataQueue;
STATIC_MEM_QUEUE_ALLOC(tdoaDataQueue, 10, sizeof(tdoaMeasurement_t));

static inline bool stateEstimatorHasTDOAPacket(tdoaMeasurement_t *uwb)
{
    return (pdTRUE == xQueueReceive(tdoaDataQueue, uwb, 0));
}

// Measurements of flow (dnx, dny)
static QueueHandle_t flowDataQueue;
STATIC_MEM_QUEUE_ALLOC(flowDataQueue, 10, sizeof(flowMeasurement_t));

static inline bool stateEstimatorHasFlowPacket(flowMeasurement_t *flow)
{
    return (pdTRUE == xQueueReceive(flowDataQueue, flow, 0));
}

// Measurements of TOF from laser sensor
static QueueHandle_t tofDataQueue;
STATIC_MEM_QUEUE_ALLOC(tofDataQueue, 10, sizeof(tofMeasurement_t));

static inline bool stateEstimatorHasTOFPacket(tofMeasurement_t *tof)
{
    return (pdTRUE == xQueueReceive(tofDataQueue, tof, 0));
}

// Absolute height measurement along the room Z
static QueueHandle_t heightDataQueue;
STATIC_MEM_QUEUE_ALLOC(heightDataQueue, 10, sizeof(heightMeasurement_t));

static inline bool stateEstimatorHasHeightPacket(heightMeasurement_t *height)
{
    return (pdTRUE == xQueueReceive(heightDataQueue, height, 0));
}

static QueueHandle_t yawErrorDataQueue;
STATIC_MEM_QUEUE_ALLOC(yawErrorDataQueue, 10, sizeof(yawErrorMeasurement_t));

static inline bool stateEstimatorHasYawErrorPacket(yawErrorMeasurement_t *error)
{
    return (pdTRUE == xQueueReceive(yawErrorDataQueue, error, 0));
}

// static QueueHandle_t sweepAnglesDataQueue;
// STATIC_MEM_QUEUE_ALLOC(sweepAnglesDataQueue, 10, sizeof(sweepAngleMeasurement_t));

// static inline bool stateEstimatorHasSweepAnglesPacket(sweepAngleMeasurement_t *angles)
// {
//   return (pdTRUE == xQueueReceive(sweepAnglesDataQueue, angles, 0));
// }

static SemaphoreHandle_t runTaskSemaphore; // Semaphore to signal the task to run is available

// Mutex to protect data that is shared between the task and
// functions called by the stabilizer loop
static SemaphoreHandle_t dataMutex;
static StaticSemaphore_t dataMutexBuffer;
// Data used to enable the task and stabilizer loop to run with minimal locking
// static state_t taskEstimatorState; // The estimator state produced by the task, copied to the stabilzer when needed.
// static Axis3f gyroSnapshot; // A snpashot of the latest gyro data, used by the task
// static Axis3f accSnapshot; // A snpashot of the latest acc data, used by the task
// Called one time during system startup
// thrust is thrust mapped for 65536 <==> 60 GRAMS!
#define CONTROL_TO_ACC (GRAVITY_MAGNITUDE * 60.0f / (CF_MASS * 1000.0f) / 65536.0f)

/**
 * Tuning parameters
 */
#define PREDICT_RATE RATE_100_HZ // this is slower than the IMU update rate of 500Hz
#define BARO_RATE RATE_25_HZ

// the point at which the dynamics change from stationary to flying
#define IN_FLIGHT_THRUST_THRESHOLD (GRAVITY_MAGNITUDE * 0.1f)
#define IN_FLIGHT_TIME_THRESHOLD (500)

// The bounds on the covariance, these shouldn't be hit, but sometimes are... why?
#define MAX_COVARIANCE (100)
#define MIN_COVARIANCE (1e-6f)
// #ifdef KALMAN_USE_BARO_UPDATE
// static const bool useBaroUpdate = true;
// #else
// static const bool useBaroUpdate = false;
// #endif
STATIC_MEM_TASK_ALLOC_STACK_NO_DMA_CCM_SAFE(kalmanTask, 3 * configMINIMAL_STACK_SIZE);

void estimatorKalman(state_t *state, sensorData_t *sensorData, control_t *control, const uint32_t tick)
{
    // This function is called from the stabilizer loop. It is important that this call returns
    // as quickly as possible. The dataMutex must only be locked short periods by the task.
    xSemaphoreTake(dataMutex, portMAX_DELAY);

    // Average the last IMU measurements. We do this because the prediction loop is
    // slower than the IMU loop, but the IMU information is required externally at
    // a higher rate (for body rate control).
    if (sensorsReadGyro(&sensorData->gyro))
    {
        gyroAccumulator.x += sensorData->gyro.x;
        gyroAccumulator.y += sensorData->gyro.y;
        gyroAccumulator.z += sensorData->gyro.z;
        gyroAccumulatorCount++;
    }
}

static void kalmanTask(void *parameters)
{
    while (true)
    {
        ESP_LOGI(KALMAN_TASK_NAME, "Kalman Task Running");
        vTaskDelay(M2T(1000));
    }
}

void estimatorKalmanTaskInit()
{
    distDataQueue = STATIC_MEM_QUEUE_CREATE(distDataQueue);
    posDataQueue = STATIC_MEM_QUEUE_CREATE(posDataQueue);
    poseDataQueue = STATIC_MEM_QUEUE_CREATE(poseDataQueue);
    tdoaDataQueue = STATIC_MEM_QUEUE_CREATE(tdoaDataQueue);
    flowDataQueue = STATIC_MEM_QUEUE_CREATE(flowDataQueue);
    tofDataQueue = STATIC_MEM_QUEUE_CREATE(tofDataQueue);
    heightDataQueue = STATIC_MEM_QUEUE_CREATE(heightDataQueue);
    yawErrorDataQueue = STATIC_MEM_QUEUE_CREATE(yawErrorDataQueue);
    // sweepAnglesDataQueue = STATIC_MEM_QUEUE_CREATE(sweepAnglesDataQueue);

    runTaskSemaphore = xSemaphoreCreateBinary();
    dataMutex = xSemaphoreCreateMutexStatic(&dataMutexBuffer);

    STATIC_MEM_TASK_CREATE(kalmanTask, kalmanTask, KALMAN_TASK_NAME, NULL, KALMAN_TASK_PRI);

    isInit = true;
}

void estimatorKalmanInit(void)
{
    ESP_LOGI(TAG, "-----------estimatorKalmanInit---------------");
    // xQueueReset(distDataQueue);
    // xQueueReset(posDataQueue);
    // xQueueReset(poseDataQueue);
    // xQueueReset(tdoaDataQueue);
    // xQueueReset(flowDataQueue);
    // xQueueReset(tofDataQueue);
}
