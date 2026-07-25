#include "bsp_jy901s.h"
#include "bsp_uart.h"

#include <string.h>

#define JY901S_FRAME_HEAD 0x55U
#define JY901S_FRAME_SIZE 11U
#define JY901S_TYPE_ACCEL 0x51U
#define JY901S_TYPE_GYRO  0x52U
#define JY901S_TYPE_ANGLE 0x53U
#define JY901S_TYPE_MAG   0x54U

static int16_t jy901s_i16(const uint8_t *data)
{
    return (int16_t) ((uint16_t) data[0] | ((uint16_t) data[1] << 8));
}

static bool jy901s_checksum_valid(const uint8_t *frame)
{
    uint8_t sum = 0U;
    uint8_t i;

    for (i = 0U; i < (JY901S_FRAME_SIZE - 1U); i++) {
        sum = (uint8_t) (sum + frame[i]);
    }
    return sum == frame[JY901S_FRAME_SIZE - 1U];
}

static void jy901s_decode(const uint8_t *frame, jy901s_data_t *imu)
{
    int16_t x = jy901s_i16(&frame[2]);
    int16_t y = jy901s_i16(&frame[4]);
    int16_t z = jy901s_i16(&frame[6]);

    switch (frame[1]) {
    case JY901S_TYPE_ACCEL:
        imu->accel[0] = (float) x * (16.0f / 32768.0f);
        imu->accel[1] = (float) y * (16.0f / 32768.0f);
        imu->accel[2] = (float) z * (16.0f / 32768.0f);
        imu->temperature =
            (float) jy901s_i16(&frame[8]) / 100.0f;
        break;
    case JY901S_TYPE_GYRO:
        imu->gyro[0] = (float) x * (2000.0f / 32768.0f);
        imu->gyro[1] = (float) y * (2000.0f / 32768.0f);
        imu->gyro[2] = (float) z * (2000.0f / 32768.0f);
        break;
    case JY901S_TYPE_ANGLE:
        imu->angle[0] = (float) x * (180.0f / 32768.0f);
        imu->angle[1] = (float) y * (180.0f / 32768.0f);
        imu->angle[2] = (float) z * (180.0f / 32768.0f);
        imu->roll = imu->angle[0];
        imu->pitch = imu->angle[1];
        imu->yaw = imu->angle[2];
        break;
    case JY901S_TYPE_MAG:
        imu->mag[0] = (float) x;
        imu->mag[1] = (float) y;
        imu->mag[2] = (float) z;
        break;
    default:
        break;
    }
}

void bsp_jy901s_init(jy901s_data_t *imu)
{
    if (imu != NULL) {
        memset(imu, 0, sizeof(*imu));
    }
}

void bsp_jy901s_process(jy901s_data_t *imu)
{
    static uint8_t frame[JY901S_FRAME_SIZE];
    static uint8_t index = 0U;
    uint8_t byte;

    if (imu == NULL) {
        return;
    }

    while (bsp_uart_jy901s_rxbuf_pop(&byte)) {
        if (index == 0U) {
            if (byte == JY901S_FRAME_HEAD) {
                frame[index++] = byte;
            }
            continue;
        }

        frame[index++] = byte;
        if (index == JY901S_FRAME_SIZE) {
            if (jy901s_checksum_valid(frame)) {
                jy901s_decode(frame, imu);
            }
            index = 0U;
        }
    }
}
