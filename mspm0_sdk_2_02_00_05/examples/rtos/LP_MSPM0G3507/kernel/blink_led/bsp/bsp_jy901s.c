#include "bsp_jy901s.h"
#include "bsp_uart.h"

#include <stdbool.h>
#include <string.h>

#define JY901S_FRAME_HEAD    0x55U
#define JY901S_FRAME_SIZE    11U
#define JY901S_TYPE_TIME     0x50U
#define JY901S_TYPE_ACCEL    0x51U
#define JY901S_TYPE_GYRO     0x52U
#define JY901S_TYPE_ANGLE    0x53U
#define JY901S_TYPE_MAG      0x54U
#define JY901S_TYPE_DSTATUS  0x55U
#define JY901S_TYPE_PRESSURE 0x56U
#define JY901S_TYPE_LON_LAT  0x57U
#define JY901S_TYPE_GPS      0x58U

static uint8_t jy901s_frame[JY901S_FRAME_SIZE];
static uint8_t jy901s_index;

static int16_t jy901s_i16(const uint8_t *data)
{
    return (int16_t) ((uint16_t) data[0] | ((uint16_t) data[1] << 8));
}

static int32_t jy901s_i32(const uint8_t *data)
{
    uint32_t value = (uint32_t) data[0] |
                     ((uint32_t) data[1] << 8) |
                     ((uint32_t) data[2] << 16) |
                     ((uint32_t) data[3] << 24);
    return (int32_t) value;
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

static bool jy901s_decode(const uint8_t *frame, jy901s_data_t *imu)
{
    int16_t x = jy901s_i16(&frame[2]);
    int16_t y = jy901s_i16(&frame[4]);
    int16_t z = jy901s_i16(&frame[6]);

    switch (frame[1]) {
    case JY901S_TYPE_TIME:
        imu->time.year = frame[2];
        imu->time.month = frame[3];
        imu->time.day = frame[4];
        imu->time.hour = frame[5];
        imu->time.minute = frame[6];
        imu->time.second = frame[7];
        imu->time.millisecond =
            (uint16_t) frame[8] | ((uint16_t) frame[9] << 8);
        imu->valid_mask |= JY901S_VALID_TIME;
        break;

    case JY901S_TYPE_ACCEL:
        imu->accel[0] = (float) x * (16.0f / 32768.0f);
        imu->accel[1] = (float) y * (16.0f / 32768.0f);
        imu->accel[2] = (float) z * (16.0f / 32768.0f);
        imu->temperature = (float) jy901s_i16(&frame[8]) / 100.0f;
        imu->valid_mask |= JY901S_VALID_ACCEL;
        break;

    case JY901S_TYPE_GYRO:
        imu->gyro[0] = (float) x * (2000.0f / 32768.0f);
        imu->gyro[1] = (float) y * (2000.0f / 32768.0f);
        imu->gyro[2] = (float) z * (2000.0f / 32768.0f);
        imu->temperature = (float) jy901s_i16(&frame[8]) / 100.0f;
        imu->valid_mask |= JY901S_VALID_GYRO;
        break;

    case JY901S_TYPE_ANGLE:
        imu->angle[0] = (float) x * (180.0f / 32768.0f);
        imu->angle[1] = (float) y * (180.0f / 32768.0f);
        imu->angle[2] = (float) z * (180.0f / 32768.0f);
        imu->temperature = (float) jy901s_i16(&frame[8]) / 100.0f;
        imu->roll = imu->angle[0];
        imu->pitch = imu->angle[1];
        imu->yaw = imu->angle[2];
        imu->valid_mask |= JY901S_VALID_ANGLE;
        break;

    case JY901S_TYPE_MAG:
        imu->mag[0] = (float) x;
        imu->mag[1] = (float) y;
        imu->mag[2] = (float) z;
        imu->temperature = (float) jy901s_i16(&frame[8]) / 100.0f;
        imu->valid_mask |= JY901S_VALID_MAG;
        break;

    case JY901S_TYPE_DSTATUS:
        imu->digital_status[0] = x;
        imu->digital_status[1] = y;
        imu->digital_status[2] = z;
        imu->digital_status[3] = jy901s_i16(&frame[8]);
        imu->valid_mask |= JY901S_VALID_DSTATUS;
        break;

    case JY901S_TYPE_PRESSURE:
        imu->pressure = jy901s_i32(&frame[2]);
        imu->altitude = jy901s_i32(&frame[6]);
        imu->valid_mask |= JY901S_VALID_PRESSURE;
        break;

    case JY901S_TYPE_LON_LAT:
        imu->longitude = jy901s_i32(&frame[2]);
        imu->latitude = jy901s_i32(&frame[6]);
        imu->valid_mask |= JY901S_VALID_LONGITUDE;
        break;

    case JY901S_TYPE_GPS:
        imu->gps_height = x;
        imu->gps_yaw = y;
        imu->gps_velocity = jy901s_i32(&frame[6]);
        imu->valid_mask |= JY901S_VALID_GPS;
        break;

    default:
        return false;
    }

    return true;
}

static void jy901s_resync(void)
{
    uint8_t offset;

    for (offset = 1U; offset < JY901S_FRAME_SIZE; offset++) {
        if (jy901s_frame[offset] == JY901S_FRAME_HEAD) {
            jy901s_index = (uint8_t) (JY901S_FRAME_SIZE - offset);
            memmove(jy901s_frame, &jy901s_frame[offset], jy901s_index);
            return;
        }
    }
    jy901s_index = 0U;
}

void bsp_jy901s_init(jy901s_data_t *imu)
{
    jy901s_index = 0U;
    memset(jy901s_frame, 0, sizeof(jy901s_frame));

    if (imu != NULL) {
        memset(imu, 0, sizeof(*imu));
    }
}

void bsp_jy901s_process(jy901s_data_t *imu)
{
    uint8_t byte;

    if (imu == NULL) {
        return;
    }

    while (bsp_uart_jy901s_rxbuf_pop(&byte)) {
        if (jy901s_index == 0U) {
            if (byte == JY901S_FRAME_HEAD) {
                jy901s_frame[jy901s_index++] = byte;
            }
            continue;
        }

        jy901s_frame[jy901s_index++] = byte;
        if (jy901s_index == JY901S_FRAME_SIZE) {
            if (jy901s_checksum_valid(jy901s_frame)) {
                if (jy901s_decode(jy901s_frame, imu)) {
                    imu->valid_frames++;
                } else {
                    imu->unsupported_frames++;
                }
                jy901s_index = 0U;
            } else {
                imu->checksum_errors++;
                jy901s_resync();
            }
        }
    }
}
