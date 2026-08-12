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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ================== 测试参数配置 ================== */
/* 以下参数集中定义，修改后一键生效                            */

/* 测试速度：线速度与角速度 (单位：编码器脉冲数/控制周期)    */
/* TEST_LINEAR_VEL = 10 = PID输出上限(100)的10%              */
#define TEST_LINEAR_VEL       6.0f   /* 直行目标速度 (Max的10%) */
#define TEST_ANGULAR_VEL      0.0f     /* 转向角速度 (0 = 直行)  */

/* 测试阶段延时 (ms) */
#define TEST_STOP_DELAY_MS    3000     /* 上电后静止等待时间     */
#define TEST_RUN_DELAY_MS     2000     /* 阶跃响应持续时间       */

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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_I2C2_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  // 1. 初始化外设模块
    TB6612_Init();      // 初始化电机PWM驱动
    Chassis_Init();     // 初始化底盘与PID参数
    Encoder_Init();     // 启动编码器定时器 (TIM2, TIM3)

  // 2. 确保上电时电机是静止的
    Chassis_SetVelocity(0.0f, 0.0f);

  // 3. 启动系统中断 (TIM1)，PID控制和波形发送开始运行
    HAL_TIM_Base_Start_IT(&htim1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      /* 状态1：停止等待，让系统和电机完全静止，便于观察起点         */
      /* 这段时间内 TIM1 中断依然在每10ms周期维持电机速度为0          */
      Chassis_SetVelocity(0.0f, 0.0f);
      HAL_Delay(TEST_STOP_DELAY_MS);

      /* 状态2：双轮同速直行，观察电机响应曲线                        */
      /* 通过 VOFA+ 软件观察目标速度 vs 实际速度波形                   */
      Chassis_SetVelocity(TEST_LINEAR_VEL, TEST_ANGULAR_VEL);
      HAL_Delay(TEST_RUN_DELAY_MS);

      /* 状态3：停止，回到状态1开始下一轮测试                         */
      Chassis_SetVelocity(0.0f, 0.0f);
      HAL_Delay(TEST_STOP_DELAY_MS);
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
  * @brief  TIM1定时器中断周期回调 (每10ms调用一次)
  * @note   整个PID闭环控制的核心调度入口：
  *         1. 读取编码器当前速度
  *         2. PID计算并驱动电机
  *         3. 向VOFA发送目标速度与实际速度波形数据
  * @param  htim: 触发回调的定时器句柄
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
        float vofa_data[4];  // VOFA JustFloat协议：4通道浮点数据帧

        // 第一步：读取编码器测量到的当前速度 (单位：脉冲数/10ms控制周期)
        Encoder_UpdateSpeed();

        // 第二步：将目标速度与实际速度送入PID控制器，输出PWM并驱动TB6612
        Chassis_UpdateTask(Encoder_Left.speed, Encoder_Right.speed);

        // 第三步：组装VOFA数据帧，通过USART1发送目标速度与实际速度曲线
        vofa_data[0] = target_speed_left;    // CH0：左轮目标速度
        vofa_data[1] = Encoder_Left.speed;   // CH1：左轮实际速度 (编码器反馈)
        vofa_data[2] = target_speed_right;   // CH2：右轮目标速度
        vofa_data[3] = Encoder_Right.speed;  // CH3：右轮实际速度 (编码器反馈)

        VOFA_SendJustFloat(vofa_data, 4);
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
