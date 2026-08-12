/**
 * @file    tb6612.c
 * @brief   TB6612FNG 双路直流电机驱动模块
 * @note    STBY 引脚已硬件接 3.3V，无需软件控制
 *          速度统一使用 0 ~ TB6612_SPEED_MAX (100)，正反转通过方向参数控制
 *          PWM 频率 = 72MHz / (PSC+1) / (ARR+1) = 72MHz / 1 / 1440 = 50KHz
 */

#include "tb6612.h"

/**
 * @brief  初始化 TB6612：启动两路 PWM 输出通道
 */
void TB6612_Init(void)
{
    HAL_TIM_PWM_Start(&TB6612_PWMA_TIM, TB6612_PWMA_CH);
    HAL_TIM_PWM_Start(&TB6612_PWMB_TIM, TB6612_PWMB_CH);
}

/**
 * @brief  设置单个电机速度与方向
 * @param  motor : 电机ID (MOTOR_LEFT / MOTOR_RIGHT)
 * @param  speed : 速度值，范围 0 ~ TB6612_SPEED_MAX (100)
 *                 speed ≤ TB6612_DEADBAND 时强制进入短路刹车
 * @param  dir   : 转向 (MOTOR_FORWARD / MOTOR_REVERSE)
 * @note   PWM 占空比 = speed × TB6612_PWM_ARR / TB6612_SPEED_MAX
 *         刹车模式：IN1=IN2=HIGH，H桥下管导通，电机绕组短路实现电磁制动
 */
void TB6612_SetMotorSpeed(Motor_ID_t motor, uint8_t speed, Motor_Dir_t dir)
{
    uint16_t      pwm_val;
    GPIO_PinState pin1_state, pin2_state;

    /* ---- 死区处理：极低速度进入短路刹车，解决零PWM下惯性滑行问题 ---- */
    if (speed <= TB6612_DEADBAND) {
        /* 短路刹车：AIN1=AIN2=HIGH → H桥下管导通，绕组短路制动 */
        pin1_state = GPIO_PIN_SET;
        pin2_state = GPIO_PIN_SET;
        pwm_val    = 0;
    } else {
        /* 速度上限保护 */
        if (speed > TB6612_SPEED_MAX) {
            speed = TB6612_SPEED_MAX;
        }

        /* ---- 方向反接修正：某轮电机线反接时，软件反转其方向 ---- */
        if (TB6612_LEFT_DIR_INVERT && motor == MOTOR_LEFT) {
            dir = (dir == MOTOR_FORWARD) ? MOTOR_REVERSE : MOTOR_FORWARD;
        }
        if (TB6612_RIGHT_DIR_INVERT && motor == MOTOR_RIGHT) {
            dir = (dir == MOTOR_FORWARD) ? MOTOR_REVERSE : MOTOR_FORWARD;
        }

        /* 方向 → IN1/IN2 电平组合 */
        if (dir == MOTOR_FORWARD) {
            pin1_state = GPIO_PIN_RESET;
            pin2_state = GPIO_PIN_SET;
        } else { /* MOTOR_REVERSE */
            pin1_state = GPIO_PIN_SET;
            pin2_state = GPIO_PIN_RESET;
        }

        /* 速度映射到 PWM 比较值：speed × ARR / SPEED_MAX            */
        /* 例：speed=100 → 100×1439/100=1439 (满占空比)              */
        /*     speed=1   → 1×1439/100=14   (最低有效占空比，不丢步)   */
        pwm_val = (uint16_t)((uint32_t)speed * TB6612_PWM_ARR
                             / TB6612_SPEED_MAX);
    }

    /* ---- 写入 GPIO 方向和 PWM 比较寄存器 ---- */
    /* 注意：MOTOR_LEFT 对应物理左轮 = TB6612 B通道 (BIN+PWMB)     */
    /*       MOTOR_RIGHT 对应物理右轮 = TB6612 A通道 (AIN+PWMA)    */
    if (motor == MOTOR_LEFT) {
        HAL_GPIO_WritePin(TB6612_BIN1_PORT, TB6612_BIN1_PIN, pin1_state);
        HAL_GPIO_WritePin(TB6612_BIN2_PORT, TB6612_BIN2_PIN, pin2_state);
        __HAL_TIM_SET_COMPARE(&TB6612_PWMB_TIM, TB6612_PWMB_CH, pwm_val);
    } else { /* MOTOR_RIGHT */
        HAL_GPIO_WritePin(TB6612_AIN1_PORT, TB6612_AIN1_PIN, pin1_state);
        HAL_GPIO_WritePin(TB6612_AIN2_PORT, TB6612_AIN2_PIN, pin2_state);
        __HAL_TIM_SET_COMPARE(&TB6612_PWMA_TIM, TB6612_PWMA_CH, pwm_val);
    }
}

/**
 * @brief  紧急停止所有电机 (双轮短路刹车)
 */
void TB6612_StopAll(void)
{
    TB6612_SetMotorSpeed(MOTOR_LEFT,  0, MOTOR_FORWARD);
    TB6612_SetMotorSpeed(MOTOR_RIGHT, 0, MOTOR_FORWARD);
}
