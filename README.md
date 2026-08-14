# TB6612 电机控制工程（PID 调参版）

基于 STM32F103C8T6 + TB6612FNG 的双轮差速底盘运动控制系统，本分支（PID）为 V1.0.1 ~ V1.0.2 的 PID 参数调试版本，不含循迹模块，聚焦底盘运动控制与 PID 闭环整定。

## 工程结构

```
TB6612_motor/
├── Core/
│   ├── Inc/                    # 头文件
│   │   ├── chassis.h           # 底盘控制 + PID参数宏定义
│   │   ├── encoder.h           # 编码器接口
│   │   ├── pid.h               # PID控制器
│   │   ├── tb6612.h            # TB6612驱动 + 引脚映射
│   │   ├── tim.h               # 定时器配置
│   │   ├── usart.h             # 串口配置
│   │   └── vofa.h              # VOFA+ 波形调试接口
│   └── Src/                    # 源文件
│       ├── chassis.c           # 底盘运动学解算 + PID闭环
│       ├── encoder.c           # 编码器测速
│       ├── main.c              # 主程序 + 测试参数配置 + VOFA指令解析
│       ├── pid.c               # PID算法实现
│       ├── tb6612.c            # TB6612 PWM驱动
│       └── vofa.c              # VOFA+ 串口波形发送
├── Drivers/                    # ST HAL库 + CMSIS
├── STM32F103C8TX_FLASH.ld      # 链接脚本
└── TB6612_motor.ioc            # STM32CubeMX 工程配置
```

## 硬件平台与资源分配

| 外设 | 配置 | 用途 |
|---|---|---|
| 时钟 | HSE 8MHz，PLL×9 = 72MHz | 系统主频 |
| TIM1 | 72MHz/(72)/(10000) = 10ms 周期中断 | PID 控制调度入口 |
| TIM2 | ARR=1439，PWM 50kHz | 电机 PWM 输出 |
| TIM3 | 编码器模式 (TI12)，PA6/PA7 | 右轮编码器测速 |
| TIM4 | 编码器模式 (TI12)，PB6/PB7 | 左轮编码器测速 |
| USART1 | 115200，DMA + 空闲中断 RX | VOFA+ 波形输出 + 在线调参 |

### 电机与编码器引脚映射（见 [tb6612.h](file:///d:/CubeIDE/Project/TB6612_motor/Core/Inc/tb6612.h)）

| 通道 | PWM | IN1 | IN2 | 编码器 |
|---|---|---|---|---|
| 右电机 (A) | PWMA=PA0 (TIM2_CH1) | AIN1=PA5 | AIN2=PA4 | TIM3: PA6/PA7 |
| 左电机 (B) | PWMB=PA1 (TIM2_CH2) | BIN1=PB0 | BIN2=PB1 | TIM4: PB6/PB7 |

STBY 引脚硬件接 3.3V。速度统一使用 0~100（`TB6612_SPEED_MAX`），`speed ≤ TB6612_DEADBAND(5)` 时强制短路刹车消除惯性滑行。

## 软件架构与实现方案

控制链路：`上位机VOFA/串口 → main.c(目标速度) → chassis.c(PID闭环) → tb6612.c(PWM/方向) → 电机`，编码器反馈经 `encoder.c` 每 10ms 回读。

### 1. 主程序（[main.c](file:///d:/CubeIDE/Project/TB6612_motor/Core/Src/main.c)）

- 初始化顺序：`TB6612_Init()` → `Chassis_Init()` → `Encoder_Init()` → 启动 DMA 空闲中断接收 → 启动 TIM1 中断
- 测试模式宏 `TEST_MOTOR_MODE`（[main.c](file:///d:/CubeIDE/Project/TB6612_motor/Core/Src/main.c) 第 51 行）：
  - `1` = 只测左轮（右轮目标强制 0），`2` = 只测右轮（左轮目标强制 0），`0` = 双轮联动
- TIM1 10ms 中断回调 `HAL_TIM_PeriodElapsedCallback`：`Encoder_UpdateSpeed()` → `Chassis_UpdateTask()` → `VOFA_SendJustFloat()`（4 通道：左右目标/实际速度）
- 串口接收：`HAL_UARTEx_ReceiveToIdle_DMA` + `HAL_UARTEx_RxEventCallback`，每帧 `\0` 截断后交给 `Parse_VOFA_Command()`

### 2. VOFA 在线调参指令（[main.c](file:///d:/CubeIDE/Project/TB6612_motor/Core/Src/main.c) 第 117 行 `Parse_VOFA_Command`）

| 上位机发送 | 功能 |
|---|---|
| `LP1.5` | 在线修改左轮 Kp |
| `RP2.0` | 在线修改右轮 Kp |
| `V8.0` 或直接 `8.0` | 设置目标速度并触发运动流程 |

运动流程时序（宏定义见 [main.c](file:///d:/CubeIDE/Project/TB6612_motor/Core/Src/main.c) 第 56-59 行）：接收速度后延迟 1s（`WAIT_TRIGGER_DELAY_MS`）→ 缓冲静止 2s（`MOTION_BUFFER_MS`）→ 按目标速度运行 7s（`MOTION_RUN_MS`）→ 停止。上电自动触发一次。

### 3. 底盘控制（[chassis.c](file:///d:/CubeIDE/Project/TB6612_motor/Core/Src/chassis.c) / [chassis.h](file:///d:/CubeIDE/Project/TB6612_motor/Core/Inc/chassis.h)）

- 差速模型：`左轮 = 线速度 + 角速度`，`右轮 = 线速度 - 角速度`（`Chassis_SetVelocity`）
- PID 参数宏统一管理在 [chassis.h](file:///d:/CubeIDE/Project/TB6612_motor/Core/Inc/chassis.h) 顶部，一键修改全工程生效
- `Chassis_UpdateTask()`：PID 计算 → PID 输出符号拆解为方向枚举 + 速度绝对值 → 下发 TB6612；目标为 0 时强制刹车不经过 PID

### 4. PID 控制器（[pid.c](file:///d:/CubeIDE/Project/TB6612_motor/Core/Src/pid.c) / [pid.h](file:///d:/CubeIDE/Project/TB6612_motor/Core/Inc/pid.h)）

- 位置式 PID：`Output = Kp*e + Ki*∫e + Kd*(e - e_last)`
- 特性：积分分离（偏差 >30 清零积分，抑制阶跃尖峰）+ 积分抗饱和（限幅 ±30）+ 输出限幅（±100）
- 参数可在运行时通过串口 `LP/RP` 指令动态修改（不中断控制）

### 5. 编码器测速（[encoder.c](file:///d:/CubeIDE/Project/TB6612_motor/Core/Src/encoder.c) / [encoder.h](file:///d:/CubeIDE/Project/TB6612_motor/Core/Inc/encoder.h)）

- TIM3/TIM4 编码器模式 TI12 双边沿计数，`ENCODER_PPR = 11`（MG53X 电机）
- `Encoder_UpdateSpeed()` 每 10ms 读取 16 位计数器并清零，`(int16_t)` 强转处理溢出回绕，速度单位 = 脉冲数/10ms，符号表示转向
- 目标速度 `DEFAULT_TARGET_SPEED`（[main.c](file:///d:/CubeIDE/Project/TB6612_motor/Core/Src/main.c) 第 54 行）即为该单位

### 6. TB6612 驱动（[tb6612.c](file:///d:/CubeIDE/Project/TB6612_motor/Core/Src/tb6612.c)）

- `TB6612_SetMotorSpeed()`：速度映射 `pwm = speed×1439/100`，方向由 IN1/IN2 电平组合控制；`speed ≤ 5` 进入短路刹车（IN1=IN2=HIGH，H 桥下管导通电磁制动）
- 方向反接修正开关 `TB6612_LEFT/RIGHT_DIR_INVERT` 位于 [tb6612.h](file:///d:/CubeIDE/Project/TB6612_motor/Core/Inc/tb6612.h)

### 7. VOFA+ 波形输出（[vofa.c](file:///d:/CubeIDE/Project/TB6612_motor/Core/Src/vofa.c)）

- JustFloat 协议：n×4 字节 float 数据 + 帧尾 `0x00 0x00 0x80 0x7F`
- 4 通道：ch0 左轮目标 / ch1 左轮实际 / ch2 右轮目标 / ch3 右轮实际

## 版本记录

### V1.0.2 (2026-08-13)

**已完成：**
- 完成左右轮 PID 调节
- 左轮最终参数：Kp=1.413, Ki=0.79, Kd=0.03（左轮收敛，降低过冲）
- 右轮最终参数：Kp=1.40, Ki=0.81, Kd=0.06（右轮加力，消除静差）

**下阶段目标：**
- 完善循迹模块

### V1.0.1 (2026-08-10)

**已完成：**
- 完成底盘双轮差速运动控制框架搭建
- 完成 TB6612FNG 驱动层封装（方向控制 + PWM调速 + 死区刹车）
- 完成编码器测速接口
- 完成 PID 闭环控制链路（目标速度 -> PID计算 -> PWM下发）
- 完成 VOFA+ JustFloat 串口波形调试输出
- **完成 V20 目标速度下 Left（左轮）PID 参数调节**
  - Kp=1.8, Ki=0.5, Kd=0.04
  - 阶跃响应测试稳定，无严重超调

**下阶段目标：**
- 完成各个速度阶段（低速/中速/高速）的跟踪特性调试
- 右轮 PID 参数精细调节
- 线速度-角速度差速转向验证
