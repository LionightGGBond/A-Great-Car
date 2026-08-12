/**
 * @file    chassis.c
 * @brief   双轮差速底盘运动控制模块
 * @note    PID输出可为正(正转)或负(反转)，本模块负责将符号拆解为方向+速度绝对值
 *          再下发给 TB6612 驱动层，TB6612层只接收 0~100 的速度和方向枚举
 */

#include "chassis.h"

PID_TypeDef PID_Left;
PID_TypeDef PID_Right;

/* 目标速度 (编码器脉冲数/控制周期) */
float target_speed_left  = 0;
float target_speed_right = 0;

/**
 * @brief  底盘初始化：配置左右轮 PID 控制器参数
 * @note   所有 PID 参数通过 chassis.h 头文件顶部的宏定义统一管理，一键修改
 */
void Chassis_Init(void)
{
    /* 注意：TB6612_Init() 已在 main.c 中调用，此处不重复初始化 PWM */

    /* 使用 chassis.h 中定义的宏初始化左右轮 PID */
    PID_Init(&PID_Left,  PID_LEFT_KP,  PID_LEFT_KI,  PID_LEFT_KD,
             PID_MAX_OUT, PID_MAX_INTEGRAL);
    PID_Init(&PID_Right, PID_RIGHT_KP, PID_RIGHT_KI, PID_RIGHT_KD,
             PID_MAX_OUT, PID_MAX_INTEGRAL);
}

/**
 * @brief  运动学解算：线速度 + 角速度 → 左右轮目标速度
 * @param  linear_vel : 线速度 (编码器脉冲数/控制周期)，正值前进
 * @param  angular_vel: 角速度，正值右转，负值左转
 * @note   差速模型：左轮 = 线速度 + 角速度，右轮 = 线速度 - 角速度
 */
void Chassis_SetVelocity(float linear_vel, float angular_vel)
{
    target_speed_left  = linear_vel + angular_vel;
    target_speed_right = linear_vel - angular_vel;
}

/**
 * @brief  底盘 PID 闭环控制更新 (每 10ms 由 TIM1 中断调用)
 * @param  current_left_speed : 左轮编码器实测速度
 * @param  current_right_speed: 右轮编码器实测速度
 * @note   PID 输出可正可负，本函数拆解符号后以绝对值+方向的方式下发给 TB6612
 */
void Chassis_UpdateTask(float current_left_speed, float current_right_speed)
{
    float       pwm_left  = PID_Calculate(&PID_Left,  target_speed_left,
                                          current_left_speed);
    float       pwm_right = PID_Calculate(&PID_Right, target_speed_right,
                                          current_right_speed);
    uint8_t     speed_left, speed_right;
    Motor_Dir_t dir_left,  dir_right;

    /* ---- 左轮：目标为零时强制刹车，不经过PID ---- */
    if (target_speed_left == 0.0f) {
        dir_left   = MOTOR_FORWARD;  /* 方向无所谓，speed=0触发刹车 */
        speed_left = 0;
    } else {
        /* 分离PID输出符号 → 方向 + 速度绝对值 */
        if (pwm_left >= 0.0f) {
            dir_left   = MOTOR_FORWARD;
            speed_left = (uint8_t)pwm_left;
        } else {
            dir_left   = MOTOR_REVERSE;
            speed_left = (uint8_t)(-pwm_left);
        }
    }

    /* ---- 右轮：目标为零时强制刹车，不经过PID ---- */
    if (target_speed_right == 0.0f) {
        dir_right   = MOTOR_FORWARD;
        speed_right = 0;
    } else {
        if (pwm_right >= 0.0f) {
            dir_right   = MOTOR_FORWARD;
            speed_right = (uint8_t)pwm_right;
        } else {
            dir_right   = MOTOR_REVERSE;
            speed_right = (uint8_t)(-pwm_right);
        }
    }

    /* ---- 下发驱动指令到 TB6612 ---- */
    TB6612_SetMotorSpeed(MOTOR_LEFT,  speed_left,  dir_left);
    TB6612_SetMotorSpeed(MOTOR_RIGHT, speed_right, dir_right);
}
