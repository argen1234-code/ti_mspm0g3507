#ifndef BSP_JY901S_H
#define BSP_JY901S_H

#include <stdint.h>

#define JY901S_VALID_TIME       (1U << 0)
#define JY901S_VALID_ACCEL      (1U << 1)
#define JY901S_VALID_GYRO       (1U << 2)
#define JY901S_VALID_ANGLE      (1U << 3)
#define JY901S_VALID_MAG        (1U << 4)
#define JY901S_VALID_DSTATUS    (1U << 5)
#define JY901S_VALID_PRESSURE   (1U << 6)
#define JY901S_VALID_LONGITUDE  (1U << 7)
#define JY901S_VALID_GPS        (1U << 8)

typedef struct {
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint16_t millisecond;
} jy901s_time_t;

typedef struct {
    jy901s_time_t time;
    float accel[3];       /* g */
    float gyro[3];        /* degree/s */
    float angle[3];       /* degree */
    float mag[3];         /* sensor raw value */
    float temperature;    /* degree Celsius */
    float roll;
    float pitch;
    float yaw;
    int16_t digital_status[4];
    int32_t pressure;
    int32_t altitude;
    int32_t longitude;
    int32_t latitude;
    int16_t gps_height;
    int16_t gps_yaw;
    int32_t gps_velocity;
    uint16_t valid_mask;
    uint32_t valid_frames;
    uint32_t checksum_errors;
    uint32_t unsupported_frames;
} jy901s_data_t;

void bsp_jy901s_init(jy901s_data_t *imu);
void bsp_jy901s_process(jy901s_data_t *imu);

#endif
