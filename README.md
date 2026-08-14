# TB6612 电机控制工程（循迹版）

基于 STM32F103C8T6 + TB6612FNG 的双轮差速底盘运动控制系统，本分支（main）为 V1.1.0 循迹主线版本，在 V1.0.0 底盘框架上装配了五路循迹模块。

## 工程结构

```
TB6612_motor/
├── Core/
│   ├── Inc/                    # 头文件
│   │   ├── chassis.h           # 底盘控制 + PID参数宏定义
│   │   ├── Close_line.h        # 五路循迹模块接口 + 传感器引脚映射
│   │   ├── encoder.h           # 编码器接口
│   │   ├── pid.h               # PID控制器
│   │   ├── tb6612.h            # TB6612驱动 + 引脚映射
│   │   ├── tim.h               # 定时器配置
│   │   ├── usart.h             # 串口配置
│   │   └── vofa.h              # VOFA+ 波形调试接口
│   └── Src/                    # 源文件
│       ├── chassis.c           # 底盘运动学解算 + PID闭环
│       ├── Close_line.c        # 循迹传感器读取 + 速度查表
│       ├── encoder.c           # 编码器测速
│       ├── main.c              # 主程序 + 循迹流程控制
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
| USART1 | 115200，DMA RX | VOFA+ 波形输出 |
| 循迹传感器 | 5 路数字 GPIO | 五路循迹检测 |

### 电机与编码器引脚映射（见 [tb6612.h](file:///d:/CubeIDE/Project/TB6612_motor/Core/Inc/tb6612.h)）

| 通道 | PWM | IN1 | IN2 | 编码器 |
|---|---|---|---|---|
| 右电机 (A) | PWMA=PA0 (TIM2_CH1) | AIN1=PA5 | AIN2=PA4 | TIM3: PA6/PA7 |
| 左电机 (B) | PWMB=PA1 (TIM2_CH2) | BIN1=PB0 | BIN2=PB1 | TIM4: PB6/PB7 |

STBY 引脚硬件接 3.3V。速度统一使用 0~100（`TB6612_SPEED_MAX`），`speed ≤ TB6612_DEADBAND(5)` 时强制短路刹车消除惯性滑行。

### 五路循迹传感器引脚映射（见 [Close_line.h](file:///d:/CubeIDE/Project/TB6612_motor/Core/Inc/Close_line.h)）

| 序号 | 位置 | 引脚 | 状态位 |
|---|---|---|---|
| 1 | 最左侧 | PB14 (M_Left) | bit0 |
| 2 | 左侧第二路 | PB13 (Left) | bit1 |
| 3 | 中间 | PA15 (Middle) | bit2 |
| 4 | 偏右侧 | PB4 (Right) | bit3 |
| 5 | 最右侧 | PB3 (M_Right) | bit4 |

传感器高电平有效（`CLOSE_LINE_ACTIVE_LEVEL = 1`）。

## 软件架构与实现方案

分层调用链：`main.c(循迹流程) → Close_line.c(状态读取/查表) → chassis.c(PID闭环) → tb6612.c(PWM/方向) → 硬件`

### 1. 主程序（[main.c](file:///d:/CubeIDE/Project/TB6612_motor/Core/Src/main.c)）

- 初始化顺序：`TB6612_Init()` → `Chassis_Init()` → `Encoder_Init()` → 启动 TIM1 中断
- 主循环循迹流程：
  1. 上电缓冲 2s（`CLOSE_LINE_BUFFER_MS`），目标速度置 0 保持静止
  2. 循迹运行 `CLOSE_LINE_RUN_MS = 100s`，每 10ms（`CLOSE_LINE_PERIOD_MS`）一个控制周期：
     - 读取五路状态 → 查表得原始左右速度 → 按 `CLOSE_LINE_BASE_SPEED(20) / CLOSE_LINE_TABLE_MAX(40)` 等比例缩放后写入左右轮目标速度
  3. 运行结束停止并死循环保持停车
- TIM1 10ms 中断回调 `HAL_TIM_PeriodElapsedCallback`：`Encoder_UpdateSpeed()` → `Chassis_UpdateTask()` → `VOFA_SendJustFloat()`（4 通道：左右目标/实际速度）

### 2. 循迹模块（[Close_line.c](file:///d:/CubeIDE/Project/TB6612_motor/Core/Src/Close_line.c)）

- `CloseLine_ReadState()`：依次读 5 路 GPIO，组合成 5 位状态位图（bit0~bit4 对应左→右）
- `CloseLine_GetSpeedPair()`：状态→查表得到 `{左速, 右速}`，共 9 种有效状态，未识别状态兜底 `{20,20}` 低速直线
- 速度表设计（开环思想）：中间压线 `{40,40}` 直线；偏离中线时一侧减速一侧保持，形成差速转向；极限状态（只压最外侧）一侧速度降为 0

### 3. 底盘控制（[chassis.c](file:///d:/CubeIDE/Project/TB6612_motor/Core/Src/chassis.c) / [chassis.h](file:///d:/CubeIDE/Project/TB6612_motor/Core/Inc/chassis.h)）

- 差速模型：`左轮 = 线速度 + 角速度`，`右轮 = 线速度 - 角速度`（`Chassis_SetVelocity`）
- PID 参数宏统一管理在 [chassis.h](file:///d:/CubeIDE/Project/TB6612_motor/Core/Inc/chassis.h) 顶部：左轮 Kp=1.413/Ki=0.79/Kd=0.03，右轮 Kp=1.40/Ki=0.81/Kd=0.06，输出限幅 ±100，积分限幅 ±30
- `Chassis_UpdateTask()`：PID 计算 → PID 输出符号拆解为方向枚举 + 速度绝对值 → 下发 TB6612；目标为 0 时强制刹车不经过 PID

### 4. PID 控制器（[pid.c](file:///d:/CubeIDE/Project/TB6612_motor/Core/Src/pid.c) / [pid.h](file:///d:/CubeIDE/Project/TB6612_motor/Core/Inc/pid.h)）

- 位置式 PID：`Output = Kp*e + Ki*∫e + Kd*(e - e_last)`
- 特性：积分分离（偏差 >30 清零积分，抑制阶跃尖峰）+ 积分抗饱和（限幅 ±30）+ 输出限幅（±100）

### 5. 编码器测速（[encoder.c](file:///d:/CubeIDE/Project/TB6612_motor/Core/Src/encoder.c) / [encoder.h](file:///d:/CubeIDE/Project/TB6612_motor/Core/Inc/encoder.h)）

- TIM3/TIM4 编码器模式 TI12 双边沿计数，`ENCODER_PPR = 11`（MG53X 电机）
- `Encoder_UpdateSpeed()` 每 10ms 读取 16 位计数器并清零，`(int16_t)` 强转处理溢出回绕，速度单位 = 脉冲数/10ms，符号表示转向

### 6. TB6612 驱动（[tb6612.c](file:///d:/CubeIDE/Project/TB6612_motor/Core/Src/tb6612.c)）

- `TB6612_SetMotorSpeed()`：速度映射 `pwm = speed×1439/100`，方向由 IN1/IN2 电平组合控制；`speed ≤ 5` 进入短路刹车（IN1=IN2=HIGH，H 桥下管导通电磁制动）
- 方向反接修正开关 `TB6612_LEFT/RIGHT_DIR_INVERT` 位于 [tb6612.h](file:///d:/CubeIDE/Project/TB6612_motor/Core/Inc/tb6612.h)

### 7. VOFA+ 调试输出（[vofa.c](file:///d:/CubeIDE/Project/TB6612_motor/Core/Src/vofa.c)）

- JustFloat 协议：n×4 字节 float 数据 + 帧尾 `0x00 0x00 0x80 0x7F`
- 4 通道：ch0 左轮目标 / ch1 左轮实际 / ch2 右轮目标 / ch3 右轮实际

## 版本记录

### V1.1.0 (2026-08-13)

**已完成：**
- 完成循迹模块装配，能够完整跟踪直线
- 五路循迹传感器读取 + 9 状态速度查表差速转向

**已知问题：**
- 曲线部分权重过高，导致另一侧轮子停转卡顿、造成抖动（未解决）
  - 根因分析：速度表中极限状态（如 `{0,28}`、`{28,0}`）一侧目标速度直接为 0，速度差骤变引发一侧停转；转向权重分配不连续导致抖动
- 运行速度 20%（`CLOSE_LINE_BASE_SPEED = 20`），有待提升
- PID 参数需要跟随修改（曲线转向时建议降低单轮目标速度骤变、引入平滑过渡）

**下阶段目标：**
- 优化速度查表：限制最小目标速度不为 0、转向权重线性化
- 提升循迹基础速度
- 跟随转向工况重新整定 PID

### V1.0.0
- 初始工程创建，STM32CubeMX 生成基础配置
