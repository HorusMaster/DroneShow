
#include "system.h"
#include "static_mem.h"
#include "config.h"
#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "wifi_module.h"

STATIC_MEM_TASK_ALLOC(systemTask, SYSTEM_TASK_STACKSIZE);

/* Private functions */
static void systemTask(void *arg);
/* Public functions */
void systemLaunch(void)
{
  STATIC_MEM_TASK_CREATE(systemTask, systemTask, SYSTEM_TASK_NAME, NULL, SYSTEM_TASK_PRI);
}

void systemTask(void *arg)
{
  init_wifi();
}