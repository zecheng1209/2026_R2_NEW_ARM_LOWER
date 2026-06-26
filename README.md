# 五自由度机械臂控制系统 (Robotic Arm Control System)

## 项目简介

本项目是一个基于 **STM32F407** 微控制器的五自由度（5-DOF）机械臂控制系统。系统通过 **CAN 总线**控制 RobStride 系列电机，实现机械臂的关节位置、速度和力矩控制。通过 **USB CDC** 与上位机（PC）进行实时通信，接收目标位置并反馈实际状态。

---

## 系统架构

### 硬件平台
- **MCU**: STM32F407xx (ARM Cortex-M4, 168MHz)
- **通信接口**: 
  - 双路 CAN 总线（CAN1 & CAN2）用于电机驱动
  - USB 虚拟串口（CDC）用于与上位机通信
  - UART4 用于遥控器接收（DMA 空闲中断）
- **GPIO 控制**: 气泵（PC5）和电磁阀（PC4）控制

### 电机配置（5个关节）

| 关节 | 名称 | 电机型号 | 连接 CAN | 电机 ID | 方向 | 初始偏移 |
|------|------|----------|----------|---------|------|----------|
| 0 | 云台（底座） | RobStride_06 | CAN1 | 0x01 | 正 | 2.2696 rad |
| 1 | 大臂 | RobStride_03 | CAN1 | 0x02 | 正 | 1.1827 rad |
| 2 | 小臂 | RobStride_01 | CAN2 | 0x03 | 反 | 2.1369 rad |
| 3 | 手腕 | RobStride_02 | CAN2 | 0x04 | 正 | -0.0107 rad |
| 4 | 末端（夹爪） | RobStride_EL05 | CAN2 | 0x05 | 正 | 5.3768 rad |

---

## 控制算法

### 双闭环 PID 控制
每个关节均采用 **位置环 + 速度环** 双闭环 PID 控制：

```
上位机目标位置 -> 位置环 PID -> 速度环 PID -> 电机力矩指令
```

### 各关节 PID 参数

| 关节 | 位置环 Kp | 位置环 Kd | 速度环 Kp | 速度环 Ki | 速度环输出限幅 |
|------|-----------|-----------|-----------|-----------|--------------|
| 云台（0） | 30.0 | 0.0 | 8.0 | 0.8 | 50.0 |
| 大臂（1） | 5.0 | 2.0 | 15.0 | 0.1 | 100.0 |
| 小臂（2） | 5.0 | 0.0 | 2.7 | 0.1 | 70.0 |
| 手腕（3） | 60.0 | 0.0 | 3.5 | 0.8 | 20.0 |
| 末端（4） | 60.0 | 0.0 | 3.0 | 0.3 | 10.0 |

---

## 项目结构

```
Arm_Baisc_Control/
├── Core/                      # STM32 核心代码（由 CubeMX 生成）
│   ├── Inc/                   # 头文件：main.h, can.h, usart.h, gpio.h 等
│   └── Src/                   # 源文件：main.c, freertos.c, can.c, usart.c 等
├── MyTask/                    # 应用层任务（核心控制逻辑）
│   ├── Task_Init.c/h          # 电机初始化、PID 参数配置、FreeRTOS 任务创建
│   ├── Run.c/h                # 电机驱动控制、USB 数据收发、气泵控制
├── MyLib/                     # 自定义库
│   ├── RobStride2.c/h         # RobStride 电机驱动库（CAN 通信协议）
│   ├── PID.c/h                # PID 控制器实现
│   ├── motor.c/h              # 传统电机驱动（RM3508/GM6020/M2006 等）
│   ├── motorEx.c/h            # 电机扩展封装
│   ├── CANDrive.c/h           # CAN 底层驱动封装
│   ├── usb_trans.c/h          # USB 通信传输
│   └── infrared_host.c/h      # 红外遥控主机
├── USB_DEVICE/                # USB 设备配置（CDC 虚拟串口）
├── Drivers/                   # HAL 驱动和 CMSIS
│   ├── STM32F4xx_HAL_Driver/  # STM32 HAL 库
│   └── CMSIS/                 # CMSIS 标准库
├── Middlewares/               # 中间件
│   └── Third_Party/FreeRTOS/  # FreeRTOS 实时操作系统
└── MDK-ARM/                   # Keil MDK 工程文件
```

---

## 通信协议

### USB 数据包格式（与上位机通信）

#### 1. 下行数据（PC -> 机械臂）
`ArmTarget_t` 结构体（1字节对齐）：

```c
#pragma pack(1)
typedef struct {
    int pack_type;          // 数据包类型：0x01
    Motor_t joints[6];      // 6个关节目标值（实际使用5个）
    unsigned char air_pump; // 气泵开关：1=开启，0=关闭
    uint8_t worked;         // 任务结束标志
} ArmTarget_t;

typedef struct {
    float rad;      // 目标关节角度（弧度）
    float omega;    // 目标角速度
    float torque;   // 目标力矩
} Motor_t;
```

#### 2. 上行数据（机械臂 -> PC）
`ArmState_t` 结构体（1字节对齐）：

```c
#pragma pack(1)
typedef struct {
    int pack_type;      // 数据包类型：0x01
    Motor_t joints[6];  // 6个关节实际状态（实际使用5个）
    uint8_t message;    // 状态消息（如抓取成功等）
} ArmState_t;
```

---

## FreeRTOS 任务

| 任务名称 | 优先级 | 周期 | 功能说明 |
|----------|--------|------|----------|
| Motor_Drive | 4 | 1ms | 电机驱动核心：执行双闭环 PID 控制，向所有电机发送控制指令 |
| Motor_reset | 4 | 5ms | 电机复位：启动时将所有关节缓慢归零 |
| MotorSendTask | 4 | 20ms | 状态上报：通过 USB 向上位机发送关节实际状态 |
| MotorRecTask | 4 | 事件触发 | 指令接收：通过 USB 接收上位机目标位置，并控制气泵/电磁阀 |

---

## 末端执行器（气泵 & 夹爪）

系统支持真空吸附末端执行器：

- **气泵控制**：`GPIOC_PIN_5` — 高电平开启，低电平关闭
- **电磁阀控制**：`GPIOC_PIN_4` — 用于快速释放吸附
  - 气泵关闭时，电磁阀自动开启 **5秒** 后关闭，释放残余负压

---

## 开发环境

- **IDE**: Keil MDK-ARM (ARM Compiler 5/6)
- **框架**: STM32CubeMX + HAL 库
- **RTOS**: FreeRTOS v10.x
- **语言**: C99

---

## 构建与烧录

1. 使用 **STM32CubeMX** 打开 `.ioc` 文件（如有），配置外设后生成代码
2. 在 **Keil MDK** 中打开 `MDK-ARM/Mechanical_Arm.uvprojx`
3. 编译项目并下载到 STM32F407 开发板

---

## 作者

- 作者：Trick Immortal 等
- 日期：2025 - 2026

## 许可证

本项目仅供学习和研究使用。
