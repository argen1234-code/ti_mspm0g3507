#include "ti_msp_dl_config.h"

/* FreeRTOS includes. */
#include <FreeRTOS.h>
#include <task.h>

#include "app_chassis.h"

static void vHeartbeatTask(void *pvParameters)
{
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void prvSetupHardware(void)
{
    SYSCFG_DL_init();
}

#if (configCHECK_FOR_STACK_OVERFLOW)
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    while (1) {
    }
}
#endif

int main(void)
{
    prvSetupHardware();

    xTaskCreate(vHeartbeatTask, "heartbeat", configMINIMAL_STACK_SIZE,
        NULL, tskIDLE_PRIORITY + 1, NULL);

#if CHASSIS_BOARD_TASK
    xTaskCreate(chassis_task, "chassis", configMINIMAL_STACK_SIZE * 4,
        NULL, tskIDLE_PRIORITY + 2, NULL);
#endif

    vTaskStartScheduler();

    return 0;
}
