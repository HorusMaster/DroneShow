
#include "stabilizer.h"
#include "static_mem.h"
#include "stm32_legacy.h"
#include "esp_log.h"
#include "config.h"
#include "freertos/FreeRTOS.h"
#include "estimator.h"
#include "sensors.h"

static const char *TAG = "stabilizer";
static bool isInit;

static state_t state;
static sensorData_t sensorData;
static control_t control;
// Se debe de incluir FREERTOS para el error de unknown type name 'StackType_t'
STATIC_MEM_TASK_ALLOC(stabilizerTask, STABILIZER_TASK_STACKSIZE);

static void stabilizerTask(void *param)
{
  uint32_t tick;
  tick = 1;
  while (1)
  {    
    //sensorsAcquire(&sensorData, tick);    
    stateEstimator(&state, &sensorData, &control, tick);        
    tick++;
    vTaskDelay(M2T(20));
  }
}

void stabilizerInit(StateEstimatorType estimator)
{
  if(isInit)
    return;


  stateEstimatorInit(estimator);

  STATIC_MEM_TASK_CREATE(stabilizerTask, stabilizerTask, STABILIZER_TASK_NAME, NULL, STABILIZER_TASK_PRI);

  isInit = true;
}