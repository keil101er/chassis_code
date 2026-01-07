<!-- -*- coding: utf-8 -*- -->
<!-- 本文件使用 UTF-8 编码，请在 VSCode 中确保使用 UTF-8 打开 -->
<!-- VSCode: 右下角点击编码 -> 选择 "通过编码重新打开" -> 选择 "UTF-8" -->

# CLAUDE.md

本文件为 Claude Code (claude.ai/code) 在此代码仓库中工作时提供指导。

**重要：在此项目中，请始终使用中文进行回答和交流。添加的代码注释为中英双语**

**重要说明**：此项目是在一个框架上编写而成的，有许多函数和代码并没有参与机器人的实际控制，所以读取到相关代码时，不要认为它就被实际使用了。现在实际运行的FreeRTOS任务有 `INS_task`, `ChassisR_task`, `ChassisL_task`, `OBSERVE_task`, `detect_task`, `referee_usart_task`, `led_flow_task`，当你不确定一个函数或什么东西是否真正有用时可以从这些任务函数中寻找。

**重要：当需要了解云台的相关代码时可访问“D:\RM_code\code_rmuc (4)\code_rmuc\yuntai _send”**
---

## 目录

1. [项目概述](#项目概述)
2. [快速开始](#快速开始)
3. [代码架构](#代码架构)
4. [控制逻辑详解](#控制逻辑详解)
5. [双板系统架构](#双板系统架构)
6. [VMC虚拟模型约束算法](#vmc虚拟模型约束算法)
7. [PID调参参考](#pid调参参考)
8. [调试与问题排查](#调试与问题排查)
9. [附录](#附录)

---

## 项目概述

这是一个基于 **STM32F407 的 RoboMaster 比赛机器人控制系统**，用于轮腿底盘（chassis）平台。固件控制底盘平衡、轮腿运动、跟随云台，并实现高机动性的运动控制。项目使用 FreeRTOS 进行任务管理，遵循 DJI RoboMaster 开发板标准。

### 系统定位

**本项目为底盘板（下板）控制代码**，与云台板（上板）协同工作：

- **底盘板职责**（本项目）：轮腿运动控制（VMC）、平衡控制、底盘跟随云台、功率管理、姿态解算
- **云台板职责**：云台姿态稳定、射击控制、视觉自瞄、遥控器解析、裁判系统通信
- **通信方式**：底盘板通过直接读取云台 Yaw 电机（DM4310, CAN ID=9）编码器值实现跟随（⚠️ 备用 CAN ID 0x15 协议已实现但未启用）

### 核心特性

- **VMC（虚拟模型约束）腿部运动学控制**：五连杆机构的正逆运动学、雅可比矩阵、力控制
- **FreeRTOS 实时多任务调度**：1ms IMU融合 + 2ms腿部控制的高频实时系统
- **9DOF IMU 姿态融合**：BMI088 + IST8310，基于四元数EKF的姿态解算
- **DM8009 关节电机驱动**：MIT模式（位置/速度/力矩混合控制）
- **M3508 轮毂电机驱动**：高精度速度闭环控制
- **底盘跟随云台**：通过读取云台 Yaw 电机编码器实现高实时性跟随

### 硬件配置

- **目标硬件**：STM32F407IGHx 微控制器（1MB Flash, 192KB RAM）
- **操作系统**：FreeRTOS with CMSIS-RTOS 封装
- **编译系统**：Keil MDK-ARM (μVision)
- **主要外设**：
  - CAN1/CAN2：腿部关节电机（DM8009）、轮毂电机（M3508）、云台 Yaw 电机编码器读取
  - SPI1：BMI088 IMU（加速度计+陀螺仪）
  - I2C1：IST8310 磁力计
  - USART6：调试串口（115200 bps）

---

## 快速开始

### 项目文件

- `MDK-ARM/chassis_task.uvprojx` - Keil 项目文件
- `chassis_task.ioc` - STM32CubeMX 配置文件（如果存在）

### 编译方式

#### 方式1：Keil μVision 编译（标准方式）

**前提条件**：
- 已安装 Keil MDK-ARM v5（通常位于 `C:\Keil_v5\`）

**编译步骤**：
1. 打开 Keil μVision IDE
2. 打开工程文件：`MDK-ARM/chassis_task.uvprojx`
3. 点击 Build 按钮或按 **F7** 编译
4. 编译输出位于 `chassis_task/` 目录：
   - `chassis_task.axf` - ELF 可执行文件
   - `chassis_task.hex` - HEX 固件（用于烧录）
   - `chassis_task.map` - 内存映射文件

#### 方式2：Keil 命令行编译

**前提条件**：
- 已安装 Keil MDK-ARM v5
- 在命令行中进入 `MDK-ARM/` 目录

**编译命令**：
```bash
# 标准编译
"C:\Keil_v5\UV4\UV4.exe" -b chassis_task.uvprojx -o build_log.txt

# 查看编译结果
type build_log.txt
```

**成功标志**：
```
"chassis_task\chassis_task.axf" - 0 Error(s), 0 Warning(s).
```

### 烧录固件

使用 J-Link 或 ST-Link 烧录：
- 在 Keil 中选择 **Flash → Download (F8)**
- 或使用 JFlash 烧录 `chassis_task.hex`

### 调试

- Keil 内置调试器：**Debug → Start/Stop Debug Session (Ctrl+F5)**
- 支持 JTAG/SWD 接口
- 使用 Watch 窗口监控实时变量

### 编译输出分析

```
Program Size: Code=XXXXX RO-data=XXXX RW-data=XXXX ZI-data=XXXXX
```

- **Flash 使用量** = Code + RO-data + RW-data
- **RAM 使用量** = RW-data + ZI-data

**性能基准**（STM32F407IGHx）：
- Flash 限制：1MB
- RAM 限制：192KB
- 典型使用率：Flash ~20%，RAM ~40%

### 常见编译错误

| 错误类型 | 示例 | 解决方法 |
|---------|------|---------|
| 函数名拼写错误 | `error: #70: incomplete type` | 检查函数声明和定义 |
| 头文件缺失 | `error: #5: cannot open source input file` | 检查 Keil Include Paths 设置 |
| 未定义引用 | `error: L6218E: Undefined symbol` | 检查 .c 文件是否已添加到项目 |
| 内存溢出 | `error: L6407E: Sections could not fit` | 优化代码或检查数组大小 |

---

## 代码架构

### 目录结构

```
├── application/          # 高层应用任务和控制逻辑
│   ├── chassisR_task.c/h # 右腿控制任务 ⭐
│   ├── chassisL_task.c/h # 左腿控制任务 ⭐
│   ├── INS_task.c/h      # 惯导姿态解算 ⭐
│   ├── OBSERVE_task.c/h  # 状态观测器
│   ├── chassis_behaviour.c/h # 底盘行为状态机
│   ├── CAN_receive.c/h   # CAN 接收电机反馈
│   ├── remote_control.c/h # 遥控器接收（未在底盘板使用）
│   ├── detect_task.c/h   # 设备在线检测
│   └── referee_usart_task.c/h # 裁判系统通信
├── bsp/boards/           # 板级支持包 - 硬件抽象层
│   ├── bsp_can.c/h       # CAN 总线初始化
│   ├── bsp_dwt.c/h       # DWT 高精度计时器
│   ├── bsp_usart.c/h     # 串口通信
│   └── CANdata_analysis.c/h # 板间 CAN 数据解析（备用）
├── components/           # 可复用软件组件
│   ├── algorithm/
│   │   ├── VMC_calc.c/h  # VMC 运动学算法 ⭐
│   │   ├── QuaternionEKF.c/h # 四元数 EKF
│   │   ├── kalman_filter.c/h # 卡尔曼滤波
│   │   └── chassis_power_control.c/h # 功率管理
│   ├── controller/
│   │   └── pid.c/h       # PID 控制器 ⭐
│   └── devices/
│       ├── BMI088driver.c/h # IMU 驱动 ⭐
│       ├── ist8310driver.c/h # 磁力计驱动
│       └── dm_8009_drv.c/h # DM8009 电机驱动 ⭐
├── Drivers/              # STM32 HAL 和 CMSIS 库
├── Middlewares/          # FreeRTOS 中间件
├── Src/                  # STM32CubeMX 生成的代码
│   ├── main.c            # 程序入口
│   ├── freertos.c        # FreeRTOS 任务创建 ⭐
│   └── can.c, usart.c... # 外设初始化
├── Inc/                  # 生成的头文件
└── MDK-ARM/              # Keil 项目和编译文件
    └── chassis_task.uvprojx # Keil 工程文件
```

### FreeRTOS 任务架构

系统运行多个并发任务，定义在 `Src/freertos.c` 中：

| 任务名 | 优先级 | 栈大小 | 功能 | 周期 |
|--------|--------|--------|------|------|
| `INS_task` | Realtime | 1024 | IMU 数据融合，姿态解算 | 1ms (1000Hz) |
| `ChassisR_task` | AboveNormal | 512 | 右腿 VMC 控制 | 2ms (500Hz) |
| `ChassisL_task` | AboveNormal | 512 | 左腿 VMC 控制 | 2ms (500Hz) |
| `OBSERVE_task` | High | 512 | 状态观测与估计 | - |
| `detect_task` | Normal | 256 | 故障检测 | - |
| `referee_usart_task` | Normal | 128 | 裁判系统通信 | - |
| `led_flow_task` | Low | 128 | LED 状态指示 | - |

**关键控制周期**：
- **1ms (1000Hz)**：IMU 姿态融合，高精度姿态估计
- **2ms (500Hz)**：腿部和底盘控制的核心频率

### 硬件抽象层注意事项

**重要**：`Src/` 和 `Inc/` 可能由 STM32CubeMX 生成。重新生成时，保留 `/* USER CODE BEGIN */` 和 `/* USER CODE END */` 块之间的代码。

**对于硬件相关的修改，始终修改 BSP 文件**（`bsp/boards/bsp_*.c`），而不是 `Src/` 中 HAL 生成的代码。

---

## 控制逻辑详解

### 双层控制架构

本项目采用 **行为层（Behaviour Layer）+ 任务层（Task Layer）** 的双层控制架构：

```
云台 Yaw 编码器 → 底盘行为状态机 → 左右腿任务控制器 → VMC 运动学 → PID 控制 → 电机输出
         ↑
    IMU 姿态反馈
```

#### 第一层：行为状态机

**文件位置**：`application/chassis_behaviour.c/h`

**底盘行为模式** (`chassis_behaviour_e`)：
- `CHASSIS_ZERO_FORCE` - 零力矩模式（安全模式）
- `CHASSIS_BALANCE` - 平衡模式（站立）
- `CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW` - 跟随云台模式 ⭐（常用）
- `CHASSIS_NO_FOLLOW_YAW` - 小陀螺模式（不跟随云台）
- `CHASSIS_OPEN_LOOP` - 开环控制模式
- `CHASSIS_STOP` - 停止模式

**关键函数**：
- `chassis_behaviour_mode_set()` - 根据遥控器或系统状态设置行为模式
- `chassis_behaviour_control_set()` - 根据当前行为调用对应控制函数

#### 第二层：任务控制器

**文件位置**：
- `application/chassisR_task.c/h` - 右腿控制
- `application/chassisL_task.c/h` - 左腿控制

**控制特点**：
- 运行频率：2ms (500Hz)
- 双腿独立控制，各自的 VMC 运动学
- 每条腿控制 2 个关节电机（DM8009）+ 2 个轮毂电机（M3508）

**控制模式**：
- `CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW`：速度控制 + 云台跟随
- `CHASSIS_VECTOR_NO_FOLLOW_YAW`：速度控制 + 小陀螺
- `CHASSIS_VECTOR_RAW`：原始电流控制

### 底盘跟随云台机制

#### ⭐ 实际使用的跟随机制（通过云台电机编码器）

**当前系统实际采用的底盘跟随云台方式**：底盘板直接通过 CAN 总线读取云台 Yaw 电机（DM4310，ID=9）的编码器值，而不是通过专用的板间通信协议。

**数据流路径**：

1. **云台 Yaw 电机反馈** → CAN 总线（电机控制总线）

2. **底盘板接收电机数据**：
   - 文件：`application/CAN_receive.c:109-127`
   - 接收云台 Yaw 电机（ID=9）的编码器、速度、电流等反馈数据
   - 存储到：`chassis_move_balance.motor_chassis[4].ecd`

3. **底盘板计算云台相对角度**：
   - 文件：`chassisL_task.c:176`（或 `chassisR_task.c` 对应位置）
   ```c
   chassis->yaw_motor_angle = motor_ecd_to_angle_change(chassis->motor_chassis[4].ecd, 0);
   ```
   - 将编码器值转换为相对角度（注意：offset=0，即以编码器零点为参考）

4. **底盘板使用云台角度进行跟随控制**：
   - 文件：`application/chassisR_task.c:653`（或对应位置）
   ```c
   chassis->relative_angle = chassis->yaw_motor_angle;
   ```

**关键特点**：
- ✅ **实时性高**：直接读取电机反馈，无额外通信延迟
- ✅ **可靠性强**：复用电机控制 CAN 总线，无需额外通信协议
- ⚠️ **依赖编码器零点**：底盘跟随的准确性依赖于云台电机 `offset_ecd` 的正确设置（在云台板代码中）
- ⚠️ **无云台姿态信息**：底盘无法获知云台的 IMU 绝对姿态，仅知道电机机械位置

#### 备用通信协议（CAN ID 0x15，当前未启用）

**⚠️ 注意**：以下协议代码已实现（`bsp/boards/CANdata_analysis.c/h`）但**当前未启用**。

**设计目的**：从云台板接收 IMU 的绝对 Yaw 角度，用于更高级的姿态融合控制。

**数据结构**：`c_fbpara_t` 结构体（定义在 `CANdata_analysis.h`）
```c
typedef struct {
    uint8_t  MODE;
    uint8_t  MODE_STOOT;
    int16_t  pitch_ch;
    int16_t  yaw_ch;
    float    received_yaw;
    float    received_upyaw;  // 云台板 IMU Yaw 角度
    uint8_t  received_v;
} c_fbpara_t;
```

**通信 ID**：0x15（CAN1 总线）

**启用方法**：
1. 云台板：取消注释发送函数调用（`gimbal_task.c:286`）
2. 底盘板：修改控制逻辑，使用 `C_data.received_upyaw` 替代 `yaw_motor_angle`
3. 底盘板：添加通信超时检测

### 电机控制系统

#### DM8009 关节电机（腿部）

**驱动文件**：`components/devices/dm_8009_drv.c/h`

**控制模式**：MIT 模式（位置/速度/力矩混合控制）
- 位置控制：设定关节角度
- 速度控制：设定关节速度
- 力矩控制：设定关节力矩

**每条腿配置**：
- 左腿：2 个 DM8009 关节电机
- 右腿：2 个 DM8009 关节电机

**控制命令发送**：
```c
dm_8009_enable(motor);  // 使能电机
dm_8009_control(motor, pos, vel, kp, kd, torque);  // 发送控制命令
```

#### M3508 轮毂电机（驱动轮）

**驱动文件**：`application/CAN_receive.c/h`

**控制模式**：电流闭环（速度 PID）

**每条腿配置**：
- 左腿：2 个 M3508 轮毂电机
- 右腿：2 个 M3508 轮毂电机

**控制命令发送**：
```c
CAN_cmd_chassis(motor1, motor2, motor3, motor4);  // 0x200 ID
```

**转速转换**：
```c
// RPM 转 线速度 (m/s)
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN 0.0004998609952f

// RPM 转 角速度 (rad/s)
#define CHASSIS_MOTOR_RPM_TO_OMG_SEN 0.00664267f
```

### 姿态融合系统

**文件位置**：`application/INS_task.c/h`

**传感器配置**：
- **BMI088**：6轴 IMU（3轴加速度计 + 3轴陀螺仪）
  - SPI1 接口
  - 陀螺仪量程：±2000 °/s
  - 加速度计量程：±6g
- **IST8310**：3轴磁力计
  - I2C1 接口
  - 用于 Yaw 轴绝对角度校正

**算法**：四元数扩展卡尔曼滤波器 (QuaternionEKF)

**输出数据**：
```c
extern fp32 INS_angle[3];      // Pitch, Roll, Yaw (rad)
extern fp32 INS_gyro[3];       // 角速度 (rad/s)
extern fp32 INS_accel[3];      // 加速度 (m/s²)
```

**控制周期**：1ms (1000Hz) - 最高优先级任务

### 关键数据流

```
【感知层】
BMI088 (SPI1) → INS_task → INS_angle[]/INS_gyro[]
                              ↓
云台 Yaw 电机编码器 (CAN) → motor_chassis[4].ecd → yaw_motor_angle
                              ↓
【决策层】
                    chassis_behaviour_mode_set()
                              ↓
                    chassis_behaviour_control_set()
                              ↓
【执行层】
            chassisR_task / chassisL_task
                              ↓
                       VMC_calc (运动学)
                              ↓
                    PID 控制器（腿长、横滚、前倾、轮速）
                              ↓
            DM8009 (关节) + M3508 (轮毂) → CAN 总线

【反馈回路】
电机反馈（编码器、速度、电流） → CAN 中断 → motor_measure_t → PID/VMC
```

---

## 双板系统架构

### 机器人整体架构

本 RoboMaster 机器人采用 **双板分离式架构**，由云台板和底盘板（本项目）两个独立控制器协同工作：

```
┌─────────────────────────────────────────────────────────────────┐
│                    RoboMaster 机器人系统                         │
├──────────────────────────┬──────────────────────────────────────┤
│   云台板（上板）          │         底盘板（本项目）              │
│   STM32F4xx              │         STM32F407IGHx                 │
├──────────────────────────┼──────────────────────────────────────┤
│ 【硬件控制】             │ 【硬件控制】                          │
│ • Yaw 轴电机 (DM4310)    │ • 4个关节电机 DM8009（腿部）          │
│ • Pitch 轴电机 (6020)    │ • 4个轮毂电机 M3508（驱动轮）         │
│ • 摩擦轮电机 (左右各1个) │ • 底盘 IMU (BMI088 + IST8310)        │
│ • 拨弹电机 (2006)        │                                      │
│ • 云台 IMU (BMI088)      │                                      │
├──────────────────────────┼──────────────────────────────────────┤
│ 【控制任务】             │ 【控制任务】                          │
│ • 云台姿态稳定           │ • VMC 虚拟模型约束腿部控制 ⭐         │
│ • 射击控制（热量管理）   │ • 底盘平衡控制 ⭐                     │
│ • 视觉自瞄               │ • 底盘跟随云台 ⭐                     │
│ • 遥控器接收与解析       │ • 功率管理                            │
│ • 裁判系统通信           │ • 姿态估计与观测                      │
├──────────────────────────┼──────────────────────────────────────┤
│ 【对外接口】             │ 【对外接口】                          │
│ • UART3: 遥控器(SBUS)    │ • CAN1/CAN2: 电机控制 + 云台电机读取  │
│ • UART6: 裁判系统        │ • UART6: 调试串口                     │
│ • UART1: 视觉上位机      │ • SPI1: IMU (BMI088)                 │
│ • CAN2: 云台/射击电机    │ • I2C1: 磁力计 (IST8310)             │
└──────────────────────────┴──────────────────────────────────────┘
```

### 底盘板与云台板通信机制

#### ⭐ 实际使用的跟随机制（通过云台电机编码器）

**底盘板通过读取云台 Yaw 电机（DM4310，ID=9）的 CAN 反馈数据获取云台朝向**，而不需要额外的板间通信协议。

**优势**：
- 实时性高（无额外通信协议延迟）
- 可靠性强（复用电机控制 CAN 总线）
- 代码简洁（无需实现板间通信协议栈）

**依赖条件**：
- 云台 Yaw 电机必须正常工作且反馈数据正确
- 云台板必须正确设置 `offset_ecd`（编码器零点）

**数据读取位置**：
- 文件：`application/CAN_receive.c`
- CAN 中断处理函数接收 ID=9 的电机数据
- 存储到：`motor_chassis[4]` 结构体

#### 备用板间通信协议（CAN ID 0x15，未启用）

如果需要更高级的功能（如获取云台板 IMU 绝对姿态），可以启用备用通信协议：

**云台板发送**（`CAN ID 0x15`）：
- 云台 IMU 的绝对 Yaw 角度
- 云台工作模式
- 其他状态信息

**底盘板接收**（`CANdata_analysis.c`）：
- 解析 CAN 数据包
- 存储到 `C_data` 结构体
- 供控制逻辑使用

**启用步骤**：
1. 云台板：取消注释 `CAN_gimbal_send_to_chassis()` 调用
2. 底盘板：修改控制逻辑，使用 `C_data.received_upyaw`
3. 添加通信超时保护机制

---

## VMC虚拟模型约束算法

### VMC 简介

**VMC（Virtual Model Control，虚拟模型约束）** 是一种将笛卡尔空间的力映射到关节空间力矩的控制方法，特别适用于轮腿机器人的腿部控制。

**核心思想**：
- 在笛卡尔空间（腿端坐标系）设定虚拟力/虚拟弹簧
- 通过雅可比矩阵映射到关节空间的力矩
- 实现腿长控制、轮速控制、平衡控制

### VMC 算法实现

**文件位置**：`components/algorithm/VMC_calc.c/h`

**关键数据结构**：

```c
typedef struct {
    fp32 l1;      // 大腿长度 (m)
    fp32 l2;      // 小腿长度 (m)
    fp32 l3;      // 连杆3长度 (m)
    fp32 l4;      // 连杆4长度 (m)
    fp32 l5;      // 连杆5长度 (m)

    fp32 theta1;  // 关节1角度 (rad)
    fp32 theta2;  // 关节2角度 (rad)
    fp32 theta3;  // 关节3角度 (rad)
    fp32 theta4;  // 关节4角度 (rad)

    fp32 x0;      // 腿端 x 坐标 (m)
    fp32 y0;      // 腿端 y 坐标 (m)
    fp32 L0;      // 腿长 (m)
    fp32 phi0;    // 腿与竖直方向夹角 (rad)

    fp32 F;       // 腿端支持力 (N)
    fp32 Tp;      // 腿端扭矩 (N·m)

    fp32 T1;      // 关节1力矩 (N·m)
    fp32 T2;      // 关节2力矩 (N·m)
} VMC_leg_t;
```

### VMC 核心函数

#### 1. 运动学正解

```c
void VMC_kinematics(VMC_leg_t *leg);
```

**功能**：已知关节角度 → 计算腿端位置

**输入**：`theta1`, `theta2`, `theta3`, `theta4`

**输出**：`x0`, `y0`, `L0`, `phi0`

**应用**：
- 腿部状态估计
- 腿端轨迹规划
- 运动学仿真

#### 2. 运动学逆解

```c
void VMC_inversekinematics(VMC_leg_t *leg);
```

**功能**：已知腿端位置 → 计算关节角度

**输入**：`L0`, `phi0`

**输出**：`theta1`, `theta2`, `theta3`, `theta4`

**应用**：
- 腿长控制
- 腿端轨迹跟踪
- 姿态调整

#### 3. 雅可比矩阵

```c
void VMC_Jacobian(VMC_leg_t *leg);
```

**功能**：计算笛卡尔力 → 关节力矩的映射矩阵

**公式**：
```
[T1]   [J11  J12] [F ]
[T2] = [J21  J22] [Tp]
```

**应用**：
- 力控制
- 虚拟弹簧/阻尼器
- VMC 控制核心

#### 4. VMC 力矩计算

```c
void VMC_calc_torque(VMC_leg_t *leg);
```

**功能**：根据虚拟力计算关节力矩

**输入**：`F`（腿端支持力），`Tp`（腿端扭矩）

**输出**：`T1`（关节1力矩），`T2`（关节2力矩）

**实现**：通过雅可比矩阵映射

### VMC 控制流程

```c
// 1. 读取关节编码器
theta1 = motor_joint1.angle;
theta2 = motor_joint2.angle;

// 2. 运动学正解（获取当前腿长）
VMC_kinematics(&vmc_leg);

// 3. 腿长 PID 控制（计算虚拟力）
leg.F = PID_calc(&leg_length_pid, L0_target, leg.L0);

// 4. 横滚角 PID 控制（计算虚拟扭矩）
leg.Tp = PID_calc(&roll_pid, roll_target, INS_angle[1]);

// 5. 雅可比矩阵计算
VMC_Jacobian(&leg);

// 6. 力矩映射（虚拟力 → 关节力矩）
VMC_calc_torque(&leg);

// 7. 输出到电机
dm_8009_control(motor_joint1, ..., leg.T1);
dm_8009_control(motor_joint2, ..., leg.T2);
```

### VMC PID 参数

**文件位置**：`components/algorithm/VMC_calc.h`

```c
// 腿长控制
#define LEG_PID_KP  200.0f
#define LEG_PID_KI  0.0f
#define LEG_PID_KD  500.0f

// 轮速控制
#define WHEEL_PID_KP  18000.0f
#define WHEEL_PID_KI  10.0f
#define WHEEL_PID_KD  0.0f
```

### 底盘平衡控制

**文件位置**：`application/chassisR_task.h` / `chassisL_task.h`

```c
// 横滚角控制（Roll）
#define ROLL_PID_KP 40.0f
#define ROLL_PID_KI 0.0f
#define ROLL_PID_KD 0.1f

// 前倾控制（Pitch，Tp）
#define TP_PID_KP 25.0f
#define TP_PID_KI 0.0f
#define TP_PID_KD 8.0f

// 旋转控制（Yaw，跟随云台）
#define TURN_PID_KP 5.0f
#define TURN_PID_KI 0.0f
#define TURN_PID_KD 0.8f
```

---

## PID调参参考

### VMC 控制 PID

**腿长控制** (`components/algorithm/VMC_calc.h`)：
```c
#define LEG_PID_KP  200.0f    // 比例增益：腿长误差 → 虚拟力
#define LEG_PID_KD  500.0f    // 微分增益：腿长速度 → 阻尼力
#define LEG_PID_MAX_OUT  100.0f
```

**轮速控制** (`components/algorithm/VMC_calc.h`)：
```c
#define WHEEL_PID_KP  18000.0f  // 比例增益：速度误差 → 电流
#define WHEEL_PID_KI  10.0f     // 积分增益：消除稳态误差
#define WHEEL_PID_MAX_OUT  16384.0f
#define WHEEL_PID_MAX_IOUT 2000.0f
```

### 底盘平衡 PID

**横滚角控制（Roll）** (`chassisR_task.h`)：
```c
#define ROLL_PID_KP 40.0f   // 比例增益：横滚角误差 → 虚拟扭矩
#define ROLL_PID_KD 0.1f    // 微分增益：横滚角速度 → 阻尼扭矩
#define ROLL_PID_MAX_OUT  100.0f
```

**前倾控制（Pitch/Tp）** (`chassisR_task.h`)：
```c
#define TP_PID_KP 25.0f     // 比例增益：前倾角误差 → 虚拟扭矩
#define TP_PID_KD 8.0f      // 微分增益：前倾角速度 → 阻尼扭矩
#define TP_PID_MAX_OUT  5.0f
```

**旋转控制（Yaw/Turn）** (`chassisR_task.h`)：
```c
#define TURN_PID_KP 5.0f    // 比例增益：航向角误差 → 旋转力矩
#define TURN_PID_KD 0.8f    // 微分增益：旋转角速度 → 阻尼力矩
#define TURN_PID_MAX_OUT  3.0f
```

### 电机转换系数

**M3508 轮毂电机** (`chassisR_task.h`)：
```c
// 减速比
#define M3508_MOTOR_REDUCATION 15.764705882f

// RPM → 线速度 (m/s)
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN 0.0004998609952f

// RPM → 角速度 (rad/s)
#define CHASSIS_MOTOR_RPM_TO_OMG_SEN 0.00664267f

// 电流 → 力矩 (N·m)
#define CHASSIS_MOTOR_CURRENT_TO_TORQUE_SEN 0.000366211f
```

### 调参建议

#### 1. 腿长控制调参

**目标**：快速响应、无超调、稳定站立

**步骤**：
1. 先设置 `LEG_PID_KD = 0`，逐步增大 `LEG_PID_KP`，直到出现轻微震荡
2. 减小 `LEG_PID_KP` 至震荡消失（约 70%）
3. 逐步增大 `LEG_PID_KD`，增加阻尼，抑制震荡
4. 微调 `LEG_PID_KP` 和 `LEG_PID_KD`，平衡响应速度和稳定性

**典型值**：
- Kp: 150 ~ 250
- Kd: 400 ~ 600

#### 2. 横滚角控制调参

**目标**：身体保持水平、快速恢复平衡

**步骤**：
1. 先设置 `ROLL_PID_KD = 0`，逐步增大 `ROLL_PID_KP`
2. 观察机器人左右摇摆，增加 `ROLL_PID_KD` 抑制震荡
3. 微调至机器人快速恢复水平且无持续摇摆

**典型值**：
- Kp: 30 ~ 50
- Kd: 0.05 ~ 0.2

#### 3. 前倾控制调参

**目标**：前后平衡、快速响应、无震荡

**步骤**：
1. 先设置 `TP_PID_KD = 0`，逐步增大 `TP_PID_KP`
2. 观察机器人前后摇摆，增加 `TP_PID_KD` 抑制震荡
3. 微调至机器人快速恢复平衡且无持续摇摆

**典型值**：
- Kp: 20 ~ 30
- Kd: 5 ~ 10

#### 4. 轮速控制调参

**目标**：快速加速、精确跟踪速度、无震荡

**步骤**：
1. 先设置 `WHEEL_PID_KI = 0` 和 `WHEEL_PID_KD = 0`
2. 逐步增大 `WHEEL_PID_KP`，直到速度响应快速且无明显震荡
3. 如果存在稳态误差，逐步增大 `WHEEL_PID_KI` 消除误差
4. 微调 `WHEEL_PID_KP` 和 `WHEEL_PID_KI`

**典型值**：
- Kp: 15000 ~ 20000
- Ki: 5 ~ 15
- Kd: 0

---

## 调试与问题排查

### 底盘板调试方法

#### 1. 串口调试输出

使用 USART6 (115200 bps) 进行调试输出：

```c
// 在代码中添加调试输出
printf("L0: %.3f, F: %.2f, T1: %.2f\r\n", leg.L0, leg.F, leg.T1);
```

#### 2. LED 状态指示

`led_flow_task.c` 通过不同颜色指示系统状态：
- 绿色：系统正常
- 红色：严重错误（如 IMU 离线）
- 黄色：警告（如电机离线）

#### 3. detect_task 监控

查看哪些设备离线，定位通信问题：
- IMU 离线：检查 SPI1 连接
- 电机离线：检查 CAN 总线
- 云台 Yaw 电机离线：检查云台板是否工作

#### 4. Keil 调试模式

**实时变量监控**：
```c
// Watch 窗口监控
motor_chassis[0].speed_rpm   // 电机0转速
INS_angle[0]                 // Pitch 角度
INS_angle[1]                 // Roll 角度
INS_angle[2]                 // Yaw 角度
vmc_leg_r.L0                 // 右腿长度
vmc_leg_l.L0                 // 左腿长度
```

**性能分析（DWT）**：
```c
#include "bsp_dwt.h"

uint32_t start_time = DWT_GetTimeline_us();
// 执行控制代码...
uint32_t exec_time = DWT_GetTimeline_us() - start_time;

if (exec_time > 2000) {  // 超过 2ms 报警
    printf("WARNING: Task exec time: %d us\r\n", exec_time);
}
```

### 常见问题排查

#### 底盘板问题

| 问题 | 检查项 | 解决方法 |
|------|--------|---------|
| 电机无响应 | CAN ID 映射、接线、电源 | 检查 `CAN_receive.h` 中的 ID 映射 |
| 底盘不平衡 | IMU 校准、PID 参数 | 运行 IMU 校准，调整 ROLL/TP PID |
| 腿部抖动 | VMC PID 参数、关节电机 | 调整 LEG_PID_KP/KD，检查电机 |
| 底盘不跟随云台 | 云台 Yaw 电机反馈 | 检查 `motor_chassis[4].ecd` 是否更新 |
| IMU 数据异常 | SPI 连接、校准 | 检查 SPI1 接线，重新校准 IMU |
| 腿长控制失效 | VMC 运动学、PID | 检查 VMC 参数，验证运动学正解 |

#### 底盘跟随云台问题

**底盘不跟随云台或朝向错误**：
1. 检查云台 Yaw 电机（ID=9）是否正常工作
2. 验证底盘板能否正确接收云台电机反馈数据
   - 在 CAN 中断中添加调试输出，查看 `motor_chassis[4].ecd`
3. 检查云台电机 `offset_ecd` 设置是否正确（云台板代码）
4. **如果云台反装**：需要在云台板修改 `offset_ecd`，加上 180° 偏移（4096）
5. 确认底盘板处于跟随模式（`CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW`）

**底盘跟随方向反了**：
1. 检查云台电机编码器零点设置（`offset_ecd`）
2. 可能是云台反装但未修正编码器零点
3. 解决方法：在云台板 `set_cali_gimbal_hook()` 中调整 `offset_ecd`

**底盘跟随抖动**：
1. 检查云台电机编码器数据稳定性
2. 在底盘板侧添加低通滤波器平滑编码器数据
3. 调整底盘板跟随 PID 参数（减小 Kp，增加 Kd）
4. 检查云台 Yaw 轴机械安装是否松动

**调试建议**：
- 使用 CAN 分析仪监控云台 Yaw 电机（ID=9）的反馈数据
- 在底盘板代码中输出 `yaw_motor_angle`，观察编码器角度变化
- 在云台板代码中输出 `offset_ecd`，验证零点设置
- 使用串口输出 `relative_angle` 和 `INS_angle[2]`，对比云台角度和底盘角度

#### VMC 控制问题

**腿部运动不协调**：
1. 检查左右腿 VMC 参数是否一致
2. 验证运动学正解和逆解是否正确
3. 检查关节电机编码器零点

**腿长控制超调**：
1. 减小 `LEG_PID_KP`
2. 增大 `LEG_PID_KD`
3. 检查 PID 输出限幅

**雅可比矩阵奇异**：
1. 检查腿部是否接近奇异位形（如完全伸直）
2. 添加腿长限制，避免极限位置
3. 在奇异点附近切换控制模式

#### 整体系统问题

**底盘功率超限**：
- 检查功率管理逻辑（`chassis_power_control.c`）
- 验证裁判系统数据接收
- 调整电机输出限幅

**机器人失控**：
1. 检查遥控器连接状态（云台板）
2. 验证 IMU 校准状态
3. 确认安全保护机制是否触发
4. 检查云台板和底盘板是否正常工作

**CAN 总线错误**：
1. 检查 CAN 终端电阻（120Ω）
2. 验证 CAN 波特率（1Mbps）
3. 检查 CAN_H 和 CAN_L 接线
4. 减少 CAN 总线负载

### 双板系统调试建议

**分板调试流程**：
1. **先单独调试云台板**：确保云台稳定、射击正常、IMU 数据准确
2. **再单独调试底盘板**：确保站立平衡、腿部运动学正确
3. **最后联调**：测试底盘跟随功能

**联调测试步骤**：
1. 上电后检查两块板 LED 状态
2. 验证底盘板能接收云台 Yaw 电机数据
3. 手动旋转云台，观察底盘是否跟随
4. 测试不同行为模式切换
5. 测试高速运动和急停

**通信调试**（如果启用 CAN ID 0x15 协议）：
- 使用 CAN 分析仪监控 0x15 数据帧
- 在底盘板串口输出 `C_data.received_upyaw`
- 检查通信延迟和丢包率

**性能优化**：
- 底盘跟随抖动：检查 PID 参数（`TURN_PID_KP/KD`）
- 跟随响应慢：提高通信频率（取消注释云台板发送函数）
- CAN 总线负载高：降低板间通信频率或减少数据量

---

## 附录

### 云台板项目信息

**项目路径**：`D:\RM_code\code_rmuc (4)\code_rmuc\yuntai _send\`

**核心特性**：
- **芯片**：STM32F4xx
- **控制任务**：云台稳定（1ms 周期）、射击控制、视觉自瞄
- **通信接口**：UART3（遥控器）、UART6（裁判系统）、UART1（视觉）、CAN2（云台/射击电机）

**云台板行为模式**：
- `GIMBAL_ZERO_FORCE`：零力矩模式
- `GIMBAL_INIT`：初始化归中
- `GIMBAL_ABSOLUTE_ANGLE`：陀螺仪角度控制（常用）
- `GIMBAL_RELATIVE_ANGLE`：编码器角度控制
- `GIMBAL_AUTO`：视觉自瞄

**关键文件**：
- `application/gimbal_task.c/h`：云台控制
- `application/shoot.c/h`：射击控制
- `application/CAN_receive.c`：CAN 通信（第613-626行：备用板间通信协议）
- `MDK-ARM/AutoGimbal.c/h`：视觉自瞄接口

### 备用板间通信协议代码参考（当前未使用）

**⚠️ 重要提示**：以下代码是关于 CAN ID 0x15 通信协议的，该协议当前未启用。实际系统使用直接读取云台电机编码器的方式。

#### 云台板发送数据（云台板代码）

```c
// 文件：云台板 application/CAN_receive.c:613-626
void CAN_gimbal_send_to_chassis(CAN_HandleTypeDef *hcan, float UP_INS_YAW)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8] = {0};
    uint32_t send_mail_box;

    tx_header.StdId = 0x15;  // 板间通信 ID
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;

    // 压缩 Yaw 角度：float (-3.5~3.5 rad) → int16 (-32767~32767)
    int16_t YAW_NAW = (int16_t)((UP_INS_YAW / 3.5f) * 32767);

    tx_data[0] = (YAW_NAW >> 8) & 0xFF;  // 高字节
    tx_data[1] = YAW_NAW & 0xFF;         // 低字节
    // tx_data[2-7] 保留

    HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &send_mail_box);
}
```

**调用位置**（云台板 `gimbal_task.c:286`，已注释）：
```c
// CAN_gimbal_send_to_chassis(&hcan1, gimbal_INS_angle[2]);
```

#### 底盘板接收数据（底盘板代码）

**CAN 中断处理**（`application/CAN_receive.c`）：
```c
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    if(hcan == &hcan1) {
        switch (rx_header.StdId) {
            case 0x15:  // 云台板数据
                C_fbdata1(&C_data, rx_data, 8);
                break;
        }
    }
}
```

**数据解析**（`bsp/boards/CANdata_analysis.c`）：
```c
void C_fbdata1(c_fbpara_t *motor, uint8_t *rx_data, uint32_t data_len)
{
    if(data_len == 8) {
        // 解压缩 Yaw 角度
        int16_t compressed_data = (int16_t)((rx_data[0] << 8) | rx_data[1]);
        motor->received_upyaw = ((float)compressed_data / 32767.0f) * 3.5f;
    }
}
```

**使用数据**（底盘控制逻辑）：
```c
// 伪代码示例
void chassis_follow_gimbal_control(chassis_t *chassis)
{
    // 获取底盘自身 Yaw（来自底盘 IMU）
    float chassis_yaw = INS_angle[2];

    // 获取云台 Yaw（来自云台板 CAN）
    float gimbal_yaw = C_data.received_upyaw;

    // 计算相对角度误差
    float angle_error = gimbal_yaw - chassis_yaw;

    // 角度归一化到 [-PI, PI]
    while (angle_error > PI) angle_error -= 2*PI;
    while (angle_error < -PI) angle_error += 2*PI;

    // PID 控制底盘旋转
    chassis->Wz_set = PID_calc(&chassis->turn_pid, angle_error, 0);
}
```

### 如何添加新的底盘行为模式

参考 `chassis_behaviour.h` 头部注释，步骤如下：

1. **在 `chassis_behaviour.h` 中添加新行为枚举**：
```c
typedef enum {
    CHASSIS_ZERO_FORCE,
    CHASSIS_BALANCE,
    CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW,
    CHASSIS_NO_FOLLOW_YAW,
    CHASSIS_YOUR_NEW_MODE,  // 新添加的
} chassis_behaviour_e;
```

2. **实现新的控制函数**：
```c
void chassis_your_new_mode_control(fp32 *vx, fp32 *vy, fp32 *wz, chassis_move_t *chassis)
{
    // vx: 前后速度（m/s），正值前进，负值后退
    // vy: 左右速度（m/s），正值左移，负值右移
    // wz: 旋转速度（rad/s）或角度（rad）

    *vx = 0.5f;  // 示例：设置前进速度
    *vy = 0.0f;
    *wz = 0.0f;
}
```

3. **在 `chassis_behaviour_mode_set()` 中添加模式切换逻辑**：
```c
void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode)
{
    // ... 现有逻辑 ...

    // 添加新的判断条件
    if (/* 某个条件触发新模式 */) {
        chassis_move_mode->chassis_behaviour_mode = CHASSIS_YOUR_NEW_MODE;
    }
}
```

4. **在 `chassis_behaviour_control_set()` 中调用新函数**：
```c
void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
    // ... 现有逻辑 ...

    else if(chassis_behaviour_mode == CHASSIS_YOUR_NEW_MODE) {
        chassis_your_new_mode_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
}
```

### Git 分支策略

- `master`: 主分支，稳定版本
- `test`: 当前活跃开发分支

提交前确保代码能够成功编译且不破坏现有功能。

---

## 总结

本项目是 RoboMaster 轮腿机器人的 **底盘板控制系统**，负责底盘平衡、腿部运动和跟随云台。关键特点：

1. **双板架构**：底盘板通过读取云台 Yaw 电机编码器实现跟随（备用协议 CAN ID 0x15 未启用）
2. **高频控制**：腿部控制任务运行在 2ms 周期（500Hz），IMU 融合 1ms 周期（1000Hz）
3. **VMC 算法**：虚拟模型约束实现腿部力控制，支持平衡和高机动性运动
4. **模块化设计**：行为层 + 任务层双层架构，便于扩展新的控制模式

**重要文件索引**：
- 腿部控制：`application/chassisR_task.c` / `chassisL_task.c`
- VMC 算法：`components/algorithm/VMC_calc.c` (核心运动学)
- 板间通信：`bsp/boards/CANdata_analysis.c` (备用通信协议)
- IMU 融合：`application/INS_task.c`
- PID 参数：`application/chassisR_task.h` / `components/algorithm/VMC_calc.h`
- 云台电机读取：`application/CAN_receive.c` (ID=9 电机反馈处理)
