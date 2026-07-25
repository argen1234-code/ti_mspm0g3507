#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H

#include <stdint.h>

#define ENCODER_LINES   13
#define MULTIPLY_FACTOR 2
#define GEAR_RATIO      30

extern volatile int32_t g_encoderA_cnt;
extern volatile int32_t g_encoderB_cnt;

void  bsp_encoder_init(void);
float bsp_encoder_calc_rpm(int32_t encoder_count, uint32_t sample_time_ms);

#endif
