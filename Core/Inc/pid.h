#ifndef __PID_H
#define __PID_H

#include <stdint.h>
#include <math.h>

/* ================== PID 控制参数配置 ================== */

/* 积分分离阈值：偏差绝对值超过此值时清零积分项，抑制阶跃响应过冲 */
#define PID_I_SEPARATE_THRESHOLD  30.0f

/* ================== 结构体定义 ================== */

typedef struct {
    float Kp;            /* 比例系数      */
    float Ki;            /* 积分系数      */
    float Kd;            /* 微分系数      */

    float target;        /* 目标值        */
    float current;       /* 当前测量值    */

    float error;         /* 本次偏差      */
    float last_error;    /* 上次偏差 (微分)*/
    float integral;      /* 积分累加值    */

    float max_out;       /* 输出限幅 (防止PWM溢出)     */
    float max_integral;  /* 积分限幅 (防止Windup饱和)  */
} PID_TypeDef;

/* ================== 函数声明 ================== */

void  PID_Init(PID_TypeDef *pid, float kp, float ki, float kd,
               float max_out, float max_i);
float PID_Calculate(PID_TypeDef *pid, float target, float current);

#endif
