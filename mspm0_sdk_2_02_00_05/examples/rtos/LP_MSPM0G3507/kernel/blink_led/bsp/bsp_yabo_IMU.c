#include "bsp_yabo_IMU.h"
#include "bsp_uart.h"

#include <string.h>

#define RAD2DEG    57.2957795f
#define GYRO_SCALE (1000.0f / 32767.0f)

static uint8_t yabo_state;
static uint8_t yabo_packet[IMU_PACKET_SIZE];
static uint8_t yabo_index;

static int16_t yabo_i16(const uint8_t *data)
{
    return (int16_t) ((uint16_t) data[0] | ((uint16_t) data[1] << 8));
}

static float yabo_float(const uint8_t *data)
{
    float value;
    uint32_t raw = (uint32_t) data[0] |
                   ((uint32_t) data[1] << 8) |
                   ((uint32_t) data[2] << 16) |
                   ((uint32_t) data[3] << 24);

    memcpy(&value, &raw, sizeof(value));
    return value;
}

static uint8_t yabo_checksum(const uint8_t *packet, uint8_t length)
{
    uint8_t sum = 0U;
    uint8_t i;

    for (i = 0U; i < (uint8_t) (length - 1U); i++) {
        sum = (uint8_t) (sum + packet[i]);
    }
    return sum;
}

static void yabo_decode(const uint8_t *packet, imu_yabo_data_t *imu)
{
    switch (packet[3]) {
    case IMU_TYPE_RAW:
        imu->accel[0] = (float) yabo_i16(&packet[4]) * (16.0f / 32767.0f);
        imu->accel[1] = (float) yabo_i16(&packet[6]) * (16.0f / 32767.0f);
        imu->accel[2] = (float) yabo_i16(&packet[8]) * (16.0f / 32767.0f);
        imu->gyro[0] = (float) yabo_i16(&packet[10]) * GYRO_SCALE;
        imu->gyro[1] = (float) yabo_i16(&packet[12]) * GYRO_SCALE;
        imu->gyro[2] = (float) yabo_i16(&packet[14]) * GYRO_SCALE;
        imu->mag[0] = (float) yabo_i16(&packet[16]) * (800.0f / 32767.0f);
        imu->mag[1] = (float) yabo_i16(&packet[18]) * (800.0f / 32767.0f);
        imu->mag[2] = (float) yabo_i16(&packet[20]) * (800.0f / 32767.0f);
        break;

    case IMU_TYPE_EULER:
        imu->roll = yabo_float(&packet[4]) * RAD2DEG;
        imu->pitch = yabo_float(&packet[8]) * RAD2DEG;
        imu->yaw = yabo_float(&packet[12]) * RAD2DEG;
        break;

    case IMU_TYPE_QUAT:
        imu->quat[0] = yabo_float(&packet[4]);
        imu->quat[1] = yabo_float(&packet[8]);
        imu->quat[2] = yabo_float(&packet[12]);
        imu->quat[3] = yabo_float(&packet[16]);
        break;

    case IMU_TYPE_BARO:
        imu->baro_alt = yabo_float(&packet[4]);
        imu->baro_temp = yabo_float(&packet[8]);
        imu->baro_pressure = yabo_float(&packet[12]);
        imu->baro_pressure_ref = yabo_float(&packet[16]);
        break;

    default:
        break;
    }
}

void imu_yabo_init(imu_yabo_data_t *imu)
{
    yabo_state = 0U;
    yabo_index = 0U;
    memset(yabo_packet, 0, sizeof(yabo_packet));

    if (imu != NULL) {
        memset(imu, 0, sizeof(*imu));
    }
}

void imu_yabo_process(imu_yabo_data_t *imu)
{
    uint8_t byte;

    if (imu == NULL) {
        return;
    }

    while (uart_imu_rxbuf_pop(&byte)) {
        if (yabo_state == 0U) {
            if (byte == IMU_SYNC_BYTE) {
                yabo_packet[0] = byte;
                yabo_index = 1U;
                yabo_state = 1U;
            }
            continue;
        }

        yabo_packet[yabo_index++] = byte;
        if (yabo_index >= 3U) {
            uint8_t frame_length = yabo_packet[2];

            if ((frame_length < 5U) || (frame_length > IMU_PACKET_SIZE)) {
                yabo_state = 0U;
                yabo_index = 0U;
                if (byte == IMU_SYNC_BYTE) {
                    yabo_packet[0] = byte;
                    yabo_index = 1U;
                    yabo_state = 1U;
                }
            } else if (yabo_index >= frame_length) {
                yabo_state = 0U;
                yabo_index = 0U;
                if ((yabo_packet[1] == IMU_DEV_ADDR) &&
                    (yabo_checksum(yabo_packet, frame_length) ==
                     yabo_packet[frame_length - 1U])) {
                    yabo_decode(yabo_packet, imu);
                }
            }
        }
    }
}
