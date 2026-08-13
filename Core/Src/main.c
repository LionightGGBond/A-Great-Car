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
#include "Close_line.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 循迹时直线行驶的基础目标速度，保持原 main.c 中的 20 */
#define CLOSE_LINE_BASE_SPEED   20.0f
/* 循迹速度表的最大值，用于把原始查表速度等比例缩放到基础速度附近 */
#define CLOSE_LINE_TABLE_MAX    40.0f
/* 上电后先保持静止缓冲的时间，单位：毫秒 */
#define CLOSE_LINE_BUFFER_MS    2000U
/* 循迹运动持续时间，单位：毫秒 */
#define CLOSE_LINE_RUN_MS       100000U
/* 循迹状态采集和控制周期，单位：毫秒 */
#define CLOSE_LINE_PERIOD_MS    10U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  /* 初始化 TB6612 电机驱动，启动左右电机 PWM */
  TB6612_Init();

  /* 初始化左右轮 PID 控制器参数 */
  Chassis_Init();

  /* 启动左右轮编码器测速定时器 */
  Encoder_Init();

  /* 初始阶段让左右轮目标速度都为 0，保证小车静止 */
  target_speed_left  = 0.0f;
  target_speed_right = 0.0f;

  /* 启动 TIM1 中断，周期执行编码器更新和底盘 PID 控制 */
  HAL_TIM_Base_Start_IT(&htim1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      /* 按控制周期计算整个运动过程中需要执行的控制次数 */
      uint32_t run_cycles = CLOSE_LINE_RUN_MS / CLOSE_LINE_PERIOD_MS;
      uint32_t i;
      /* 当前五路循迹传感器的状态位图 */
      uint8_t state;
      /* 查表得到的左右轮原始速度 */
      uint8_t left_speed;
      uint8_t right_speed;

      /* 上电缓冲阶段：保持左右轮停止 */
      target_speed_left  = 0.0f;
      target_speed_right = 0.0f;
      HAL_Delay(CLOSE_LINE_BUFFER_MS);

      /* 循迹运动阶段，每个控制周期根据传感器状态调整一次小车姿态 */
      for (i = 0; i < run_cycles; i++)
      {
          /* 读取五路传感器，得到当前压线状态 */
          state = CloseLine_ReadState();

          /* 根据压线状态查表，得到原始左右轮差速值 */
          CloseLine_GetSpeedPair(state, &left_speed, &right_speed);

          /* 将原始速度表按基础速度等比例缩放后，写入左右轮目标速度 */
          target_speed_left  = CLOSE_LINE_BASE_SPEED *
                               ((float)left_speed / CLOSE_LINE_TABLE_MAX);
          target_speed_right = CLOSE_LINE_BASE_SPEED *
                               ((float)right_speed / CLOSE_LINE_TABLE_MAX);

          /* 等待一个控制周期，期间 TIM1 中断会执行底盘 PID 控制 */
          HAL_Delay(CLOSE_LINE_PERIOD_MS);
      }

      /* 循迹运动结束后停止小车 */
      target_speed_left  = 0.0f;
      target_speed_right = 0.0f;

      /* 保持停车状态，防止主循环再次触发运动流程 */
      while (1)
      {
          HAL_Delay(10);
      }
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
  * @brief  TIM1 periodic control callback.
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
        /* 更新左右轮编码器实测速度 */
        Encoder_UpdateSpeed();

        /* 根据左右轮目标速度和实测速度执行 PID，并驱动 TB6612 */
        Chassis_UpdateTask(Encoder_Left.speed, Encoder_Right.speed);

        /* 上位机 VOFA 波形发送功能已关闭，保留原代码供以后恢复使用。
        {
            float vofa_data[4];
            vofa_data[0] = target_speed_left;
            vofa_data[1] = Encoder_Left.speed;
            vofa_data[2] = target_speed_right;
            vofa_data[3] = Encoder_Right.speed;
            VOFA_SendJustFloat(vofa_data, 4);
        }
        */
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
