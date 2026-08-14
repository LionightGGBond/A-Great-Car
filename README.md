# TB6612 电机控制工程

基于 STM32F103C8T6 + TB6612FNG 的双轮差速底盘运动控制系统。

## 工程结构

```
TB6612_motor/
├── Core/
│   ├── Inc/                # 头文件
│   │   ├── chassis.h       # 底盘控制 + PID参数宏定义
│   │   ├── encoder.h       # 编码器接口
│   │   ├── pid.h           # PID控制器
│   │   ├── tb6612.h        # TB6612驱动 + 引脚映射
│   │   ├── tim.h           # 定时器配置
│   │   ├── usart.h         # 串口配置
│   │   └── vofa.h          # VOFA+ 波形调试接口
│   └── Src/                # 源文件
│       ├── chassis.c       # 底盘运动学解算 + PID闭环
│       ├── encoder.c       # 编码器测速
│       ├── main.c          # 主程序 + 测试参数配置
│       ├── pid.c           # PID算法实现
│       ├── tb6612.c        # TB6612 PWM驱动
│       └── vofa.c          # VOFA+ 串口波形发送
├── Drivers/                # ST HAL库 + CMSIS
├── STM32F103C8TX_FLASH.ld  # 链接脚本
└── TB6612_motor.ioc        # STM32CubeMX 工程配置
```

## 版本记录

### V1.1.0 (2026-08-13)

**已完成：**
- 完成循迹模块装配，能够完整跟踪直线

**已知问题：**
- 曲线部分权重过高，导致另一侧轮子停转卡顿、造成抖动（未解决）
- 运行速度 20%，有待提升
- PID 参数需要跟随修改

### V1.0.0
- 初始工程创建，STM32CubeMX 生成基础配置

