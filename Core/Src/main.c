/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tb6612.h"
#include "chassis.h"
#include "encoder.h"
#include "vofa.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ================== PID 调参及测试参数配置 ================== */

/* 1. 调试轮子选择 (一键切换测试模式)
 * 1 - 仅测试左轮 (右轮目标速度强制为 0，专门用于调左轮 PID)
 * 2 - 仅测试右轮 (左轮目标速度强制为 0，专门用于调右轮 PID)
 * 0 - 左右双轮同时联动测试
 */
#define TEST_MOTOR_MODE       1

/* 2. 默认运动测试目标速度 (编码器脉冲数/10ms控制周期) */
#define DEFAULT_TARGET_SPEED  10.0f

/* 3. 运动过程各阶段时间配置 (单位: ms) */
#define WAIT_TRIGGER_DELAY_MS 1000   /* 上位机发送速度后延迟 1s 响应 */
#define MOTION_BUFFER_MS      2000   /* 上电/触发后的缓冲静止时间 2s */
#define MOTION_RUN_MS         2000   /* 电机转动持续时间 2s */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

static char rx_buffer[64];             /* DMA 串口数据接收缓冲区 */


static float current_set_speed = DEFAULT_TARGET_SPEED; /* 当前运动的目标速度 */
static volatile uint8_t trigger_flag = 0;              /* 串口指令触发运动标志位 */
static volatile uint8_t trigger_auto_on_boot = 1;      /* 上电自动触发一次运动标志位 */
extern DMA_HandleTypeDef hdma_usart1_rx;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Set_Target_Speed_By_Mode(float speed);
static void Parse_VOFA_Command(char *cmd);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief 根据测试模式 (TEST_MOTOR_MODE) 赋予左右轮目标速度
  * @param speed: 设定目标速度
  */
static void Set_Target_Speed_By_Mode(float speed)
{
#if (TEST_MOTOR_MODE == 1)
    target_speed_left  = speed;
    target_speed_right = 0.0f;
#elif (TEST_MOTOR_MODE == 2)
    target_speed_left  = 0.0f;
    target_speed_right = speed;
#else
    target_speed_left  = speed;
    target_speed_right = speed;
#endif
}

/**
  * @brief 解析 VOFA / 串口上位机发送的调参及控制指令
  * @param cmd: 包含完整指令信息的字符串
  * @note  支持指令类型:
  *        1. "LP%f\n" : 动态更新左轮 Kp 项 (如 LP1.5\n)
  *        2. "RP%f\n" : 动态更新右轮 Kp 项 (如 RP2.0\n)
  *        3. "V%f\n" 或直接发送数字 "%f\n" : 更改运动速度并触发整套运动过程
  */
static void Parse_VOFA_Command(char *cmd)
{
    float val = 0.0f;

    /* 匹配 LP%f 格式: 修改左轮 Kp */
    if (sscanf(cmd, "LP%f", &val) == 1)
    {
        PID_Left.Kp = val;
    }
    /* 匹配 RP%f 格式: 修改右轮 Kp (备用) */
    else if (sscanf(cmd, "RP%f", &val) == 1)
    {
        PID_Right.Kp = val;
    }
    /* 匹配 V%f 格式 (例如 V8.0) 或直接发送数字 (例如 8.0) */
    else if (sscanf(cmd, "V%f", &val) == 1 || sscanf(cmd, "%f", &val) == 1)
    {
        current_set_speed = val;
        trigger_flag = 1; /* 置位标志位，通知主循环触发整套运动过程 */
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_I2C2_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  /* 1. 初始化电机驱动与控制算法模块 */
  TB6612_Init();      /* 初始化电机 PWM 驱动 */
  Chassis_Init();     /* 初始化底盘 PID 参数 */
  Encoder_Init();     /* 启动编码器计数定时器 */

  /* 启动 DMA + 空闲中断接收机制 */
    /* 当收到一帧完整数据（由空闲总线判定）或缓冲区满时，会触发回调 */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *)rx_buffer, sizeof(rx_buffer));

    /* 可选优化：关闭 DMA 的过半中断 (Half Transfer)，防止变长指令触发误判 */
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

    Set_Target_Speed_By_Mode(0.0f);
    HAL_TIM_Base_Start_IT(&htim1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      /* 判断是否需要执行运动过程：上电首次自动触发 或 串口收到速度值后触发 */
      if (trigger_auto_on_boot || trigger_flag)
      {
          if (!trigger_auto_on_boot && trigger_flag)
          {
              /* 在上位机发送速度值后，单片机延迟 1s 后响应 */
              HAL_Delay(WAIT_TRIGGER_DELAY_MS);
              trigger_flag = 0;
          }
          trigger_auto_on_boot = 0; /* 上电触发仅生效一次 */

          /* [阶段 1] 缓冲 2s (目标速度为 0，电机制动) */
          Set_Target_Speed_By_Mode(0.0f);
          HAL_Delay(MOTION_BUFFER_MS);

          /* [阶段 2] 电机转动 2s (达到设定的目标速度) */
          Set_Target_Speed_By_Mode(current_set_speed);
          HAL_Delay(MOTION_RUN_MS);

          /* [阶段 3] 停止转动 */
          Set_Target_Speed_By_Mode(0.0f);
      }

      /* 空闲等待 (TIM1 中断一直在后台持续运行，编码器数据不会中断) */
      HAL_Delay(10);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
  * @brief  TIM1 定时器中断周期回调函数 (10ms 触发一次)
  * @note   核心数据调度与控制程序：
  *         1. 读取当前编码器脉冲数 (实际速度)
  *         2. 进行 PID 控制解算并输出到电机驱动器
  *         3. 将目标速度与实际速度打包通过 VOFA+ 发送
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
        /* 【修改1】正确声明一个包含4个元素的浮点型数组 */
        float vofa_data[4];

        /* 1. 更新编码器反馈实际速度 */
        Encoder_UpdateSpeed();

        /* 2. PID 计算并驱动电机 */
        Chassis_UpdateTask(Encoder_Left.speed, Encoder_Right.speed);

        /* 3. 打包 VOFA 4 通道数据 */
        /* 【修改2】为每个通道指定正确的数组下标 */
        vofa_data[0] = target_speed_left;    /* 通道 0: 左轮目标速度 */
        vofa_data[1] = Encoder_Left.speed;   /* 通道 1: 左轮实际速度 (编码器) */
        vofa_data[2] = target_speed_right;   /* 通道 2: 右轮目标速度 */
        vofa_data[3] = Encoder_Right.speed;  /* 通道 3: 右轮实际速度 (编码器) */

        /* 4. 通过串口发送 4 通道波形数据 */
        /* 此时传入 vofa_data(数组首地址，即 float *)，语法完全正确 */
        VOFA_SendJustFloat(vofa_data, 4);
    }
}
/**
  * @brief  串口中断接收完成回调函数
  * @note   解析来自 VOFA 上位机的指令以实现 PID 实时调参及运动触发
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1)
    {
        /* 1. 为接收到的字符串添加结束符，防止越界解析 */
        rx_buffer[Size] = '\0';

        /* 2. 直接将完整的一帧数据交给解析函数 */
        Parse_VOFA_Command(rx_buffer);

        /* 3. 必须重新开启 DMA 接收，准备迎接下一帧指令 */
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *)rx_buffer, sizeof(rx_buffer));
        __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
