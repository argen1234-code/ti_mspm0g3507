#include "bsp_jy61s.h"
#include "bsp_uart.h"

#include <stdbool.h>
#include <string.h>

#define JY61S_FRAME_HEAD 0x55U
#define JY61S_FRAME_SIZE 11U
#define JY61S_TYPE_TIME  0x50U
#define JY61S_TYPE_ACCEL 0x51U
#define JY61S_TYPE_GYRO  0x52U
#define JY61S_TYPE_ANGLE 0x53U
#define JY61S_TYPE_MAG   0x54U

static uint8_t jy61s_frame[JY61S_FRAME_SIZE];
static uint8_t jy61s_index;

static int16_t jy61s_i16(const uint8_t *data)
{
    return (int16_t) ((uint16_t) data[0] | ((uint16_t) data[1] << 8));
}

static bool jy61s_checksum_valid(const uint8_t *frame)
{
    uint8_t sum = 0U;
    uint8_t i;

    for (i = 0U; i < (JY61S_FRAME_SIZE - 1U); i++) {
        sum = (uint8_t) (sum + frame[i]);
    }
    return sum == frame[JY61S_FRAME_SIZE - 1U];
}

static bool jy61s_decode(const uint8_t *frame, jy61s_data_t *imu)
{
    int16_t x = jy61s_i16(&frame[2]);
    int16_t y = jy61s_i16(&frame[4]);
    int16_t z = jy61s_i16(&frame[6]);

    switch (frame[1]) {
    case JY61S_TYPE_TIME:
        imu->time.year = frame[2];
        imu->time.month = frame[3];
        imu->time.day = frame[4];
        imu->time.hour = frame[5];
        imu->time.minute = frame[6];
        imu->time.second = frame[7];
        imu->time.millisecond =
            (uint16_t) frame[8] | ((uint16_t) frame[9] << 8);
        imu->valid_mask |= JY61S_VALID_TIME;
        break;

    case JY61S_TYPE_ACCEL:
        imu->accel[0] = (float) x * (16.0f / 32768.0f);
        imu->accel[1] = (float) y * (16.0f / 32768.0f);
        imu->accel[2] = (float) z * (16.0f / 32768.0f);
        imu->temperature = (float) jy61s_i16(&frame[8]) / 100.0f;
        imu->valid_mask |= JY61S_VALID_ACCEL;
        break;

    case JY61S_TYPE_GYRO:
        imu->gyro[0] = (float) x * (2000.0f / 32768.0f);
        imu->gyro[1] = (float) y * (2000.0f / 32768.0f);
        imu->gyro[2] = (float) z * (2000.0f / 32768.0f);
        imu->temperature = (float) jy61s_i16(&frame[8]) / 100.0f;
        imu->valid_mask |= JY61S_VALID_GYRO;
        break;

    case JY61S_TYPE_ANGLE:
        imu->angle[0] = (float) x * (180.0f / 32768.0f);
        imu->angle[1] = (float) y * (180.0f / 32768.0f);
        imu->angle[2] = (float) z * (180.0f / 32768.0f);
        imu->temperature = (float) jy61s_i16(&frame[8]) / 100.0f;
        imu->roll = imu->angle[0];
        imu->pitch = imu->angle[1];
        imu->yaw = imu->angle[2];
        imu->valid_mask |= JY61S_VALID_ANGLE;
        break;

    case JY61S_TYPE_MAG:
        imu->mag[0] = (float) x;
        imu->mag[1] = (float) y;
        imu->mag[2] = (float) z;
        imu->temperature = (float) jy61s_i16(&frame[8]) / 100.0f;
        imu->valid_mask |= JY61S_VALID_MAG;
        break;

    default:
        return false;
    }

    return true;
}

static void jy61s_resync(void)
{
    uint8_t offset;

    for (offset = 1U; offset < JY61S_FRAME_SIZE; offset++) {
        if (jy61s_frame[offset] == JY61S_FRAME_HEAD) {
            jy61s_index = (uint8_t) (JY61S_FRAME_SIZE - offset);
            memmove(jy61s_frame, &jy61s_frame[offset], jy61s_index);
            return;
        }
    }
    jy61s_index = 0U;
}

void bsp_jy61s_init(jy61s_data_t *imu)
{
    jy61s_index = 0U;
    memset(jy61s_frame, 0, sizeof(jy61s_frame));

    if (imu != NULL) {
        memset(imu, 0, sizeof(*imu));
    }
}

void bsp_jy61s_process(jy61s_data_t *imu)
{
    uint8_t byte;

    if (imu == NULL) {
        return;
    }

    while (bsp_uart_imu_rxbuf_pop(&byte)) {
        if (jy61s_index == 0U) {
            if (byte == JY61S_FRAME_HEAD) {
                jy61s_frame[jy61s_index++] = byte;
            }
            continue;
        }

        jy61s_frame[jy61s_index++] = byte;
        if (jy61s_index == JY61S_FRAME_SIZE) {
            if (jy61s_checksum_valid(jy61s_frame)) {
                if (jy61s_decode(jy61s_frame, imu)) {
                    imu->valid_frames++;
                } else {
                    imu->unsupported_frames++;
                }
                jy61s_index = 0U;
            } else {
                imu->checksum_errors++;
                jy61s_resync();
            }
        }
    }
}
