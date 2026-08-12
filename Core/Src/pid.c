/**
 * @file    pid.c
 * @brief   位置式 PID 控制器实现
 * @note    特性：积分分离 + 积分抗饱和 (Anti-Windup) + 输出限幅
 *          公式：Output = Kp*e + Ki*∫e + Kd*(e - e_last)
 */

#include "pid.h"

/**
 * @brief  初始化 PID 结构体，清零所有状态变量
 */
void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd,
              float max_out, float max_i)
{
    pid->Kp           = kp;
    pid->Ki           = ki;
    pid->Kd           = kd;
    pid->target       = 0;
    pid->current      = 0;
    pid->error        = 0;
    pid->last_error   = 0;
    pid->integral     = 0;
    pid->max_out      = max_out;
    pid->max_integral = max_i;
}

/**
 * @brief  单次 PID 计算 (位置式)
 * @param  pid     : PID 控制器实例指针
 * @param  target  : 目标值
 * @param  current : 当前测量值 (编码器反馈)
 * @return PID 输出值 (已限幅，范围 -max_out ~ +max_out)
 */
float PID_Calculate(PID_TypeDef *pid, float target, float current)
{
    float output;

    /* ---- 更新目标值与偏差 ---- */
    pid->target  = target;
    pid->current = current;
    pid->error   = pid->target - pid->current;

    /* ---- 积分分离：大幅偏差时清零积分，抑制阶跃尖峰 ---- */
    if (fabsf(pid->error) > PID_I_SEPARATE_THRESHOLD) {
        pid->integral = 0;
    }

    /* ---- 积分累加 ---- */
    pid->integral += pid->error;

    /* ---- 积分抗饱和限幅 (Anti-Windup) ---- */
    if (pid->integral > pid->max_integral) {
        pid->integral = pid->max_integral;
    } else if (pid->integral < -pid->max_integral) {
        pid->integral = -pid->max_integral;
    }

    /* ---- 位置式 PID 核心公式 ---- */
    output = (pid->Kp * pid->error)
           + (pid->Ki * pid->integral)
           + (pid->Kd * (pid->error - pid->last_error));

    /* 保存本次偏差供下次微分计算使用 */
    pid->last_error = pid->error;

    /* ---- 输出限幅 ---- */
    if (output > pid->max_out) {
        output = pid->max_out;
    } else if (output < -pid->max_out) {
        output = -pid->max_out;
    }

    return output;
}
