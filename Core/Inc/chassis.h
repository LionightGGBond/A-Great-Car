#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "tb6612.h"
#include "pid.h"

/* ================== MG53X 电机 PID 参数配置 ================== */
/* 所有 PID 调参在此处一键修改，无需深入函数体查找               */

/* 左轮 PID 系数 (物理左轮个体偏快，使用低增益收敛) */
#define PID_LEFT_KP          1.413f     /* 比例系数：左轮收敛   */
#define PID_LEFT_KI          0.79f    /* 积分系数：降低防左轮过冲 */
#define PID_LEFT_KD          0.03f    /* 微分系数：不变 */









/* 右轮 PID 系数 (物理右轮机械阻力大/偏慢，使用高增益加力) */
#define PID_RIGHT_KP         1.40f     /* 比例系数：右轮加力   */
#define PID_RIGHT_KI         0.81f     /* 积分系数：加速消除右轮静差 */
#define PID_RIGHT_KD         0.06f    /* 微分系数：不变       */

/* PID 输出限幅 (对应 TB6612_SPEED_MAX = 100) */
#define PID_MAX_OUT          100.0f

/* 积分限幅 (Anti-Windup，约为 PID_MAX_OUT 的 30%) */
#define PID_MAX_INTEGRAL     30.0f

/* ================== 全局变量声明 ================== */

extern PID_TypeDef PID_Left;
extern PID_TypeDef PID_Right;
extern float target_speed_left;
extern float target_speed_right;

/* ================== 函数声明 ================== */

void Chassis_Init(void);
void Chassis_SetVelocity(float linear_vel, float angular_vel);
void Chassis_UpdateTask(float current_left_speed, float current_right_speed);

#endif
