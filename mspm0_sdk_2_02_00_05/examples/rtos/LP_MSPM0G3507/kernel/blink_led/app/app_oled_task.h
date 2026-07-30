/*
 * OLED 显示刷新任务
 *
 * 优先级: tskIDLE_PRIORITY + 2 (低于底盘控制, 高于空闲)
 * 刷新率: 5Hz (200ms)
 * 显示内容: 八路循迹状态 / 双轮转速 / IMU欧拉角(roll/pitch/yaw)
 */

#ifndef APP_OLED_TASK_H
#define APP_OLED_TASK_H

extern void oled_display_task(void *pvParameters);

#endif
