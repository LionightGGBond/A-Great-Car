#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"

/* ================== 编码器参数配置 ================== */

/* 编码器线数 (脉冲数/转)，MG53X 电机编码器 */
#define ENCODER_PPR               11

/* PID 控制周期 (ms)，需与 TIM1 中断间隔保持一致 */
#define ENCODER_CONTROL_PERIOD_MS 10

/* ================== 编码器定时器映射 ================== */
/* 左轮编码器 TIM4 (PB6/PB7)，右轮编码器 TIM3 (PA6/PA7) */

#define ENCODER_LEFT_TIM   htim4
#define ENCODER_RIGHT_TIM  htim3

extern TIM_HandleTypeDef ENCODER_LEFT_TIM;
extern TIM_HandleTypeDef ENCODER_RIGHT_TIM;

/* ================== 数据结构 ================== */

typedef struct {
    float     speed;       /* 当前速度 (脉冲数/控制周期)     */
    long long position;    /* 累计位置 (待开发者自行实现)    */
} Encoder_DataTypeDef;

extern Encoder_DataTypeDef Encoder_Left;
extern Encoder_DataTypeDef Encoder_Right;

/* ================== 函数声明 ================== */

void      Encoder_Init(void);
void      Encoder_UpdateSpeed(void);
long long Encoder_GetPosition(uint8_t motor_id);

#endif
