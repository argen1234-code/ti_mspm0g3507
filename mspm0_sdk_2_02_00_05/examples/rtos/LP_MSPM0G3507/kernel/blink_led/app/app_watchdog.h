#ifndef APP_WATCHDOG_H
#define APP_WATCHDOG_H

#include <stdbool.h>
#include <stdint.h>

#define APP_WATCHDOG_FAULT_CHASSIS   (1UL << 0)
#define APP_WATCHDOG_FAULT_OLED      (1UL << 1)
#define APP_WATCHDOG_FAULT_ZDT       (1UL << 2)

typedef enum {
    APP_WATCHDOG_TASK_CHASSIS = 0,
    APP_WATCHDOG_TASK_OLED
} app_watchdog_task_t;

typedef struct {
    volatile uint32_t chassis_heartbeat;
    volatile uint32_t oled_heartbeat;
    volatile uint32_t monitor_heartbeat;
    volatile uint32_t feed_count;
    volatile uint32_t fault_mask;
    volatile bool all_tasks_healthy;
    volatile bool hardware_running;

    uint32_t last_chassis_heartbeat;
    uint32_t last_oled_heartbeat;
    uint32_t last_zdt_heartbeat;
} app_watchdog_t;

void app_watchdog_init(app_watchdog_t *watchdog);
void app_watchdog_task_heartbeat(app_watchdog_t *watchdog,
                                 app_watchdog_task_t task);
void app_watchdog_monitor(app_watchdog_t *watchdog,
                          uint32_t zdt_heartbeat);

#endif
