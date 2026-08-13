/* 五路循迹模块头文件，包含传感器引脚和接口声明 */
#include "Close_line.h"
/* TB6612 电机驱动接口，用于直接驱动左右电机 */
#include "tb6612.h"

/*
 * 左右轮速度查表，内容和参考五路循迹程序保持一致。
 * 每一项为：{ 左轮速度, 右轮速度 }，速度范围是 TB6612 的 0~100。
 */
static const uint8_t CLOSE_LINE_SPEED_TABLE[][2] = {
    {0,  28}, /* 只有最左侧传感器压线 */

    {10, 28}, /* 最左侧和左侧第二路同时压线 */
    {25, 28}, /* 只有左侧第二路压线 */

    {37, 40}, /* 左侧第二路和中间传感器同时压线 */


    {40, 40}, /* 只有中间传感器压线，直线行驶 */


    {40, 37}, /* 中间传感器和偏右侧传感器同时压线 */

    {28, 25}, /* 只有偏右侧传感器压线 */
    {28, 10}, /* 偏右侧和最右侧传感器同时压线 */

    {28, 0},  /* 只有最右侧传感器压线 */
};

/*
 * 读取指定一路传感器的电平。
 * index 从 0 开始，0 对应最左侧，4 对应最右侧。
 */
static uint8_t CloseLine_ReadSensor(uint8_t index)
{
    /* 当前要读取的 GPIO 端口和引脚 */
    GPIO_TypeDef *port;
    uint16_t pin;
    /* 归一化后的传感器电平：1 表示压线，0 表示未压线 */
    uint8_t level;

    /* 根据传感器序号选择对应的端口和引脚 */
    switch (index)
    {
        case 0:
            port = CLOSE_LINE_SENSOR_1_PORT;
            pin  = CLOSE_LINE_SENSOR_1_PIN;
            break;
        case 1:
            port = CLOSE_LINE_SENSOR_2_PORT;
            pin  = CLOSE_LINE_SENSOR_2_PIN;
            break;
        case 2:
            port = CLOSE_LINE_SENSOR_3_PORT;
            pin  = CLOSE_LINE_SENSOR_3_PIN;
            break;
        case 3:
            port = CLOSE_LINE_SENSOR_4_PORT;
            pin  = CLOSE_LINE_SENSOR_4_PIN;
            break;
        case 4:
            port = CLOSE_LINE_SENSOR_5_PORT;
            pin  = CLOSE_LINE_SENSOR_5_PIN;
            break;
        default:
            return 0;
    }

    /* 读取原始 GPIO 电平，并统一转换成 1/0 */
    level = (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET) ? 1U : 0U;

    /* 如果传感器是低电平有效，则对电平取反 */
    if (CLOSE_LINE_ACTIVE_LEVEL == 0)
    {
        level = !level;
    }

    return level;
}

uint8_t CloseLine_ReadState(void)
{
    /* 5 位状态位图，bit0 对应最左侧，bit4 对应最右侧 */
    uint8_t state = 0;
    uint8_t i;

    /* 依次读取五路传感器，并写入对应位 */
    for (i = 0; i < 5; i++)
    {
        if (CloseLine_ReadSensor(i) != 0U)
        {
            state |= (uint8_t)(1U << i);
        }
    }

    return state;
}

/*
 * 根据五路传感器状态查表，得到左右轮速度。
 * 该函数不直接驱动电机，便于 main 中结合底盘 PID 使用。
 */
void CloseLine_GetSpeedPair(uint8_t state, uint8_t *left_speed, uint8_t *right_speed)
{
    /* 查表索引，默认值会在 switch 中处理 */
    uint8_t index = 0;

    /* 将传感器状态位图映射到速度表 */
    switch (state)
    {
        case 0x01: index = 0; break;
        case 0x03: index = 1; break;
        case 0x02: index = 2; break;
        case 0x06: index = 3; break;
        case 0x04: index = 4; break;
        case 0x0C: index = 5; break;
        case 0x08: index = 6; break;
        case 0x18: index = 7; break;
        case 0x10: index = 8; break;
        default:
            /* 未识别状态按低速直线兜底，避免电机失控 */
            *left_speed  = 20;
            *right_speed = 20;
            return;
    }

    /* 输出查表得到的左右轮速度 */
    *left_speed  = CLOSE_LINE_SPEED_TABLE[index][0];
    *right_speed = CLOSE_LINE_SPEED_TABLE[index][1];
}

/* 直接通过 TB6612 设置左右电机速度，主要用于开环循迹模式 */
static void CloseLine_SetMotorSpeed(uint8_t left_speed, uint8_t right_speed)
{
    TB6612_SetMotorSpeed(MOTOR_LEFT,  left_speed,  MOTOR_FORWARD);
    TB6612_SetMotorSpeed(MOTOR_RIGHT, right_speed, MOTOR_FORWARD);
}

void CloseLine_Update(void)
{
    /* 读取当前五路传感器状态 */
    uint8_t state = CloseLine_ReadState();
    uint8_t left_speed;
    uint8_t right_speed;

    /* 根据状态查表得到左右轮速度，并直接输出到 TB6612 */
    CloseLine_GetSpeedPair(state, &left_speed, &right_speed);
    CloseLine_SetMotorSpeed(left_speed, right_speed);
}
