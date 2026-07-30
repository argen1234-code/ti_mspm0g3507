#ifndef APP_TASK_STATE_MACHINE_H
#define APP_TASK_STATE_MACHINE_H

#include <stdbool.h>
#include <stdint.h>

#define APP_TASK1_TIME_REQUIREMENT_MS (20000U)

typedef enum {
    APP_TASK_STATE_IDLE = 0,
    APP_TASK_STATE_TASK1_TRACKING,
    APP_TASK_STATE_TASK1_FINISHED
} app_task_state_t;

typedef struct {
    app_task_state_t state;
    uint32_t elapsed_time_ms;
    uint32_t final_time_ms;
    uint32_t start_count;
    bool tracking_active;
    bool timer_running;
    bool start_key_previous;
    bool final_time_within_requirement;
} app_task_state_machine_t;

void app_task_state_machine_init(app_task_state_machine_t *machine);
bool app_task_state_machine_update(app_task_state_machine_t *machine,
                                   bool task1_start_key_pressed,
                                   uint32_t elapsed_ms);
bool app_task_state_machine_finish_task1(app_task_state_machine_t *machine);

#endif
