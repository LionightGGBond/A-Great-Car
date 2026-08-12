#include "encoder.h"

Encoder_DataTypeDef Encoder_Left = {0, 0};
Encoder_DataTypeDef Encoder_Right = {0, 0};

/**
 * @brief  初始化编码器定时器
 * @note   开启定时器的 Encoder 模式
 */
void Encoder_Init(void)
{
    // 启动左轮编码器定时器
    HAL_TIM_Encoder_Start(&ENCODER_LEFT_TIM, TIM_CHANNEL_ALL);
    // 启动右轮编码器定时器
    HAL_TIM_Encoder_Start(&ENCODER_RIGHT_TIM, TIM_CHANNEL_ALL);
}

/**
 * @brief  更新编码器速度 (需在固定的定时器中断中调用，如10ms)
 * @note   利用 (int16_t) 强转处理 16位计数器 0-65535 的溢出问题。
 *         例如：当计数器从 0 减到 65535 时，(int16_t)65535 会被直接解析为 -1。
 */
void Encoder_UpdateSpeed(void)
{
    int16_t left_pulse = 0;
    int16_t right_pulse = 0;

    // 1. 读取当前周期内的脉冲变化量
    left_pulse = (int16_t)__HAL_TIM_GET_COUNTER(&ENCODER_LEFT_TIM);
    right_pulse = (int16_t)__HAL_TIM_GET_COUNTER(&ENCODER_RIGHT_TIM);

    // 2. 清零计数器，为下一个周期做准备
    __HAL_TIM_SET_COUNTER(&ENCODER_LEFT_TIM, 0);
    __HAL_TIM_SET_COUNTER(&ENCODER_RIGHT_TIM, 0);

    // 3. 赋值给结构体 (此处单位为：脉冲数/控制周期，你可以在此处乘以系数转换为 m/s)
    Encoder_Left.speed = (float)left_pulse;
    // 注意：如果左右电机安装对称，某一个电机的编码器极性可能相反，可以在这里加负号反转
    Encoder_Right.speed = (float)right_pulse;
}

/**
 * @brief  获取电机位置接口 (由开发者自行完成)
 * @param  motor_id: 电机ID (例如 0 代表左轮，1 代表右轮)
 * @retval 累计位置值
 */
long long Encoder_GetPosition(uint8_t motor_id)
{
    // TODO: 请在此处填写你的位置计算逻辑
    // 提示：可以利用全局变量累加 left_pulse 和 right_pulse 来计算

    return 0;
}
