
#include "system.h"
#include "static_mem.h"
#include "config.h"
#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "wifi_module.h"
#include "estimator.h"
#include "estimator_kalman.h"
#include "stm32_legacy.h"
#include "sensors.h"
#include "stabilizer.h"

static bool isInit;
SemaphoreHandle_t canStartMutex;
static StaticSemaphore_t canStartMutexBuffer;

STATIC_MEM_TASK_ALLOC(systemTask, SYSTEM_TASK_STACKSIZE);

/* Private functions */
static void systemTask(void *arg);
/* Public functions */
void systemLaunch(void)
{
  STATIC_MEM_TASK_CREATE(systemTask, systemTask, SYSTEM_TASK_NAME, NULL, SYSTEM_TASK_PRI);
}


void systemInit(void)
{
  if(isInit)
    return;

  canStartMutex = xSemaphoreCreateMutexStatic(&canStartMutexBuffer);
  xSemaphoreTake(canStartMutex, portMAX_DELAY);

  //init_wifi();

  isInit = true;
}

void systemTask(void *arg)
{ 
  systemInit();  
  sensorsInit();
  StateEstimatorType estimator = anyEstimator;
  //estimatorKalmanTaskInit();
  stabilizerInit(estimator);

  {
    while (1)
    {
      vTaskDelay(M2T(500));
    }
  }
}

/* Global system variables */
void systemStart()
{
  xSemaphoreGive(canStartMutex);
#ifndef DEBUG_EP2
  //watchdogInit();
#endif
}

void systemWaitStart(void)
{
  //This permits to guarantee that the system task is initialized before other
  //tasks waits for the start event.
  while(!isInit)
    vTaskDelay(2);

  xSemaphoreTake(canStartMutex, portMAX_DELAY);
  xSemaphoreGive(canStartMutex);
}
