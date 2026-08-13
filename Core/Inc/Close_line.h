/* 防止头文件被重复包含 */
#ifndef __CLOSE_LINE_H
#define __CLOSE_LINE_H

/* CubeIDE 工程公共头文件，提供 HAL 类型和引脚宏 */
#include "main.h"

/* 五路循迹传感器按从左到右的顺序排列 */

/* 第 1 路：最左侧传感器 */
#define CLOSE_LINE_SENSOR_1_PORT   M_Left_GPIO_Port
#define CLOSE_LINE_SENSOR_1_PIN    M_Left_Pin

/* 第 2 路：左侧第二路传感器 */
#define CLOSE_LINE_SENSOR_2_PORT   Left_GPIO_Port
#define CLOSE_LINE_SENSOR_2_PIN    Left_Pin

/* 第 3 路：中间传感器 */
#define CLOSE_LINE_SENSOR_3_PORT   Middle_GPIO_Port
#define CLOSE_LINE_SENSOR_3_PIN    Middle_Pin

/* 第 4 路：偏右侧传感器 */
#define CLOSE_LINE_SENSOR_4_PORT   Right_GPIO_Port
#define CLOSE_LINE_SENSOR_4_PIN    Right_Pin

/*
 * 第 5 路：最右侧传感器。
 * 当前 CubeIDE 工程中最右侧一路是 PB5。
 * PA5 已经被 TB6612 的 AIN1 占用，不能重复用于循迹输入。
 */
#define CLOSE_LINE_SENSOR_5_PORT   M_Right_GPIO_Port
#define CLOSE_LINE_SENSOR_5_PIN    M_Right_Pin

/* 传感器输出为 1 时表示检测到黑线；如果硬件是低电平有效，请改成 0 */
#define CLOSE_LINE_ACTIVE_LEVEL    1

/* 读取一次五路传感器并直接驱动电机，适合开环循迹模式 */
void CloseLine_Update(void);
/* 读取当前五路循迹状态，返回一个 5 位位图 */
uint8_t CloseLine_ReadState(void);

/* 根据传感器状态查表，输出左右轮原始速度值 */
void CloseLine_GetSpeedPair(uint8_t state, uint8_t *left_speed, uint8_t *right_speed);

#endif
