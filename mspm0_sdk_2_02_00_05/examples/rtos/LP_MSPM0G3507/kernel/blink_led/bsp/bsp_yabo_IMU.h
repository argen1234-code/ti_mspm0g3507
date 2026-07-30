#ifndef BSP_YABO_IMU_H
#define BSP_YABO_IMU_H

#include <stdint.h>

/* Legacy Yabo/WitMotion variable-length UART protocol. */
#define IMU_SYNC_BYTE   0x7EU
#define IMU_DEV_ADDR    0x23U
#define IMU_PACKET_SIZE 23U

#define IMU_TYPE_RAW    0x04U
#define IMU_TYPE_EULER  0x26U
#define IMU_TYPE_QUAT   0x16U
#define IMU_TYPE_BARO   0x32U

typedef struct {
    float accel[3];   /* g */
    float gyro[3];    /* degree/s */
    float mag[3];     /* uT */
    float roll;       /* degree */
    float pitch;      /* degree */
    float yaw;        /* degree */
    float quat[4];
    float baro_alt;
    float baro_temp;
    float baro_pressure;
    float baro_pressure_ref;
} imu_yabo_data_t;

void imu_yabo_init(imu_yabo_data_t *imu);
void imu_yabo_process(imu_yabo_data_t *imu);

#endif
