#ifndef __TB6612_H
#define __TB6612_H

#include "main.h"
#include "tim.h"

/* ================== 引脚映射 ================== */
/* 物理接线 (依据 CubeMX 引脚标签)：
 *   TIM2_CH1(PA0) = PWMA → TB6612 A通道 → 右电机
 *   TIM2_CH2(PA1) = PWMB → TB6612 B通道 → 左电机
 *   AIN1=PA5, AIN2=PA4  (A通道方向，接右电机)
 *   BIN1=PB0, BIN2=PB1  (B通道方向，接左电机)
 *   TIM3(PA6/PA7) = 右轮编码器   TIM4(PB6/PB7) = 左轮编码器  */

/* 右电机 (Motor A) */
#define TB6612_AIN1_PORT   GPIOA
#define TB6612_AIN1_PIN    GPIO_PIN_5
#define TB6612_AIN2_PORT   GPIOA
#define TB6612_AIN2_PIN    GPIO_PIN_4
#define TB6612_PWMA_TIM    htim2
#define TB6612_PWMA_CH     TIM_CHANNEL_1

/* 左电机 (Motor B) */
#define TB6612_BIN1_PORT   GPIOB
#define TB6612_BIN1_PIN    GPIO_PIN_0
#define TB6612_BIN2_PORT   GPIOB
#define TB6612_BIN2_PIN    GPIO_PIN_1
#define TB6612_PWMB_TIM    htim2
#define TB6612_PWMB_CH     TIM_CHANNEL_2

/* ================== TB6612 驱动参数配置 ================== */
/* 以下参数集中定义，修改后全工程生效，无需深入函数体查找            */

/* TIM2 PWM 自动重载值 ARR，需与 CubeMX 中 TIM2 配置保持一致    */
#define TB6612_PWM_ARR         1439

/* 速度量程上限，速度输入范围为 0 ~ TB6612_SPEED_MAX           */
#define TB6612_SPEED_MAX       100

/* 死区阈值：速度低于此值时强制进入短路刹车，消除零PWM惯性滑行问题 */
/* MG53X齿轮电机在speed≤5时不足以克服静摩擦，直接刹车更可靠       */
#define TB6612_DEADBAND        5

/* 电机方向修正开关：若某轮电机线反接导致转向相反，置1即可软件反转  */
/* 注意：宏值绑定物理轮子，与上方通道映射配套，勿单独调换          */
#define TB6612_LEFT_DIR_INVERT   0
#define TB6612_RIGHT_DIR_INVERT  0

/* ================== 枚举类型定义 ================== */

/* 电机ID */
typedef enum {
    MOTOR_LEFT = 0,
    MOTOR_RIGHT
} Motor_ID_t;

/* 电机转向：控制 TB6612 的 IN1/IN2 引脚电平组合 */
/* 注意：实际电平以 tb6612.c 实现为准，方向反转由上述宏统一修正  */
typedef enum {
    MOTOR_FORWARD = 0,   /* 正转 */
    MOTOR_REVERSE        /* 反转 */
} Motor_Dir_t;

/* ================== 函数声明 ================== */

void TB6612_Init(void);
void TB6612_SetMotorSpeed(Motor_ID_t motor, uint8_t speed, Motor_Dir_t dir);
void TB6612_StopAll(void);

#endif
