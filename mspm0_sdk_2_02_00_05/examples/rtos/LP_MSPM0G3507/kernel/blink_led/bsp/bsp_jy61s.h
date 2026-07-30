#ifndef BSP_JY61S_H
#define BSP_JY61S_H

#include <stdint.h>

#define JY61S_VALID_TIME  (1U << 0)
#define JY61S_VALID_ACCEL (1U << 1)
#define JY61S_VALID_GYRO  (1U << 2)
#define JY61S_VALID_ANGLE (1U << 3)
#define JY61S_VALID_MAG   (1U << 4)

typedef struct {
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint16_t millisecond;
} jy61s_time_t;

typedef struct {
    jy61s_time_t time;
    float accel[3];       /* g */
    float gyro[3];        /* degree/s */
    float angle[3];       /* degree */
    float mag[3];         /* sensor raw value */
    float temperature;    /* degree Celsius */
    float roll;
    float pitch;
    float yaw;
    uint16_t valid_mask;
    uint32_t valid_frames;
    uint32_t checksum_errors;
    uint32_t unsupported_frames;
} jy61s_data_t;

void bsp_jy61s_init(jy61s_data_t *imu);
void bsp_jy61s_process(jy61s_data_t *imu);

#endif
