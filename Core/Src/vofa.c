#include "vofa.h"
#include "usart.h"
#include <string.h>

/**
 * @brief  使用 VOFA+ 的 JustFloat 协议发送浮点数据
 * @param  data_array: 包含需要发送的浮点数据的数组指针
 * @param  channel_num: 需要发送的数据通道数量 (不能超过 VOFA_MAX_CHANNELS)
 * @note   协议格式：(ch_0) (ch_1) ... (ch_n) (Tail)
 *         Tail 帧尾固定为：0x00 0x00 0x80 0x7F
 */
void VOFA_SendJustFloat(float *data_array, uint8_t channel_num)
{
    // 限制最大通道数，防止数组越界
    if (channel_num > VOFA_MAX_CHANNELS) {
        channel_num = VOFA_MAX_CHANNELS;
    }

    // 计算总字节数：浮点数数量 * 4字节 + 尾帧 4字节
    uint16_t send_length = (channel_num * 4) + 4;

    // 分配发送缓冲区
    uint8_t send_buffer[VOFA_MAX_CHANNELS * 4 + 4];

    // 1. 将浮点数组拷贝进发送缓冲区
    memcpy(send_buffer, data_array, channel_num * 4);

    // 2. 填充 JustFloat 协议的固定尾部帧 (0x00, 0x00, 0x80, 0x7F)
    send_buffer[channel_num * 4 + 0] = 0x00;
    send_buffer[channel_num * 4 + 1] = 0x00;
    send_buffer[channel_num * 4 + 2] = 0x80;
    send_buffer[channel_num * 4 + 3] = 0x7F;

    // 3. 通过 USART1 轮询发送 (如果在中断中调用，建议后续优化为 DMA 发送)
    HAL_UART_Transmit(&huart1, send_buffer, send_length, 100);
}
