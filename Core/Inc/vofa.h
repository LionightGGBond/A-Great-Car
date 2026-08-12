#ifndef __VOFA_H
#define __VOFA_H

#include "main.h"

/* 定义VOFA支持的最大通道数，可根据需要修改 */
#define VOFA_MAX_CHANNELS 4

/* 函数声明 */
void VOFA_SendJustFloat(float *data_array, uint8_t channel_num);

#endif
