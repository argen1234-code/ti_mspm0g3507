#ifndef BSP_JY901S_H
#define BSP_JY901S_H

#include <stdint.h>

typedef struct {
    float accel[3];
    float gyro[3];
    float angle[3];
    float mag[3];
    float temperature;
    float roll;
    float pitch;
    float yaw;
} jy901s_data_t;

void bsp_jy901s_init(jy901s_data_t *imu);
void bsp_jy901s_process(jy901s_data_t *imu);

#endif
