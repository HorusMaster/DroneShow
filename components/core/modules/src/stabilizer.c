
#include "stabilizer.h"
#include "static_mem.h"
#include "stm32_legacy.h"
#include "esp_log.h"
#include "config.h"
#include "freertos/FreeRTOS.h"
#include "estimator.h"
#include "sensors.h"
#include "system.h"
#include "mqtt_module.h"
#include "rateSupervisor.h"
#include "commander.h"
#include "controller.h"

static const char *TAG = "stabilizer";
static bool isInit;

static state_t state;
static sensorData_t sensorData;
static setpoint_t setpoint;
static control_t control;

static const char *estimatorType;
static const char *controllerType;
// Se debe de incluir FREERTOS para el error de unknown type name 'StackType_t'
STATIC_MEM_TASK_ALLOC(stabilizerTask, STABILIZER_TASK_STACKSIZE);
STATIC_MEM_TASK_ALLOC(mqttTask, STABILIZER_TASK_STACKSIZE);

static rateSupervisor_t rateSupervisorContext;
static bool rateWarningDisplayed = false;

static void mqttTask(void *param)
{
  while (1)
  {
    send_message(&state, &control);
    vTaskDelay(M2T(100));
  }
}

static void stabilizerTask(void *param)
{
  uint32_t tick;
  systemWaitStart();
  tick = 1;
  rateSupervisorInit(&rateSupervisorContext, xTaskGetTickCount(), M2T(1000), 997, 1003, 1);

  while (1)
  {
    sensorsWaitDataReady();
    // sensorsAcquire(&sensorData, tick);
    stateEstimator(&state, &sensorData, &control, tick);
    commanderGetSetpoint(&setpoint, &state);
    controller(&control, &setpoint, &sensorData, &state, tick);
    //ESP_LOGI(TAG, "Control: %i %i %i %f", control.roll, control.pitch, control.yaw, control.thrust);
    tick++;
    if (!rateSupervisorValidate(&rateSupervisorContext, xTaskGetTickCount()))
    {
      if (!rateWarningDisplayed)
      {
        ESP_LOGW(TAG, "WARNING: stabilizer loop rate is off (%" PRIu32 ")\n", rateSupervisorLatestCount(&rateSupervisorContext));
        rateWarningDisplayed = true;
      }
    }
    vTaskDelay(M2T(20));
  }
}

void stabilizerInit(StateEstimatorType estimator)
{
  if (isInit)
    return;

  stateEstimatorInit(estimator);
  controllerInit(ControllerTypeAny);
  
  estimatorType = stateEstimatorGetName();
  controllerType = controllerGetName();

  ESP_LOGI(TAG, "Stabilizer initialized with estimator %s and controller %s", estimatorType, controllerType);
  STATIC_MEM_TASK_CREATE(stabilizerTask, stabilizerTask, STABILIZER_TASK_NAME, NULL, STABILIZER_TASK_PRI);
  STATIC_MEM_TASK_CREATE(mqttTask, mqttTask, STABILIZER_TASK_NAME, NULL, STABILIZER_TASK_PRI);

  isInit = true;
}