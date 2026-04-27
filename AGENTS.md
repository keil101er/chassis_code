# AGENTS.md

本文件约束 `/mnt/d/RM_code/code_rmuc/chassis` 及其子目录中的代理工作方式。

## 交流与目标

- 默认使用中文交流。
- 先读代码再下结论，这个仓库里存在历史代码、备用实现和未启用模块，不能只靠文件名判断是否生效。
- 做修改时优先保持现有控制链路和硬件行为稳定，避免无关重构。

## 项目定位

- 这是一个基于 `STM32F407` 的 RoboMaster 轮腿底盘控制工程。
- RTOS 为 `FreeRTOS + CMSIS-RTOS` 封装。
- 主编译入口是 `MDK-ARM/chassis_task.uvprojx`。
- `chassis_task.ioc` 表明工程含有 STM32CubeMX 生成部分。

## 先看哪里

在分析功能是否真实生效时，优先按下面顺序阅读：

1. `Src/main.c`
2. `Src/freertos.c`
3. `MDK-ARM/chassis_task.uvprojx`
4. 具体任务函数及其调用链

不要默认 `application/` 下所有代码都在运行。这个项目的真实行为必须同时以“任务是否被创建”和“源文件是否被 Keil 工程收录”为准。

## 当前实际运行任务

以当前 `Src/freertos.c` 为准，实际创建的任务包括：

- `test_task`
- `ChassisR_Task`
- `INS_Task`
- `ChassisL_Task`
- `OBSERVE_Task`
- `referee_usart_task`
- `detect_task`

注意：

- `led_flow_task` 源码存在，但当前线程创建代码被注释掉了，不能按“正在运行”处理。
- 文档和历史说明里如果出现与上述列表冲突的说法，以当前代码为准。

## 编译单元与目录边界

### 1. 应用层

- `application/` 放高层任务、状态机、通信和业务逻辑。
- 但左腿任务当前编译用的是 `MDK-ARM/chassisL_task.c`，不是 `application/` 下的同名文件。

### 2. CubeMX 生成层

- `Src/` 和 `Inc/` 中大量文件为 CubeMX/HAL 生成代码。
- 修改这些文件时，只在 `/* USER CODE BEGIN */` / `/* USER CODE END */` 区块内动手。
- 外设初始化优先看 `Src/main.c`、`Src/freertos.c` 与各 `Src/*.c` 初始化函数。

### 3. 板级支持与算法层

- `bsp/boards/` 是板级封装，处理 CAN、SPI、USART、DWT、PWM 等硬件接口。
- `components/algorithm/` 放 VMC、卡尔曼、姿态解算等算法。
- `components/controller/` 放 PID 和底盘功率控制等控制器。

### 4. Keil 工程私有文件

以下文件位于 `MDK-ARM/`，但属于实际编译源，不要当成纯生成物忽略：

- `MDK-ARM/chassisL_task.c`
- `MDK-ARM/test_task.c`
- `MDK-ARM/ui_interface.c`
- `MDK-ARM/ui_g.c`

`MDK-ARM/build_log.txt`、`JLinkLog.txt`、`DebugConfig/` 等更接近构建产物或调试输出，除非任务明确要求，否则不要随意改动。

## 关键运行链路

- 启动入口：`Src/main.c`
- 任务创建：`Src/freertos.c`
- IMU 姿态链路：`application/INS_task.c`
- 左右腿控制：
  - 右腿：`application/chassisR_task.c`
  - 左腿：`MDK-ARM/chassisL_task.c`
- 状态观测：`application/observe_task.c`
- CAN 收发与电机反馈：`application/CAN_receive.c`、`bsp/boards/bsp_can.c`

分析控制行为时，建议从任务函数反向追到：

- 数据来源
- 共享状态
- 控制器参数
- CAN 输出

不要只盯某个局部函数。

## 高频共享状态

这个项目跨文件 `extern` 很多，改动前先确认读写关系。高频共享对象包括但不限于：

- `INS`
- `chassis_move_balance`
- `left`
- `right`
- `robot_state`
- `shoot_control`

任何对这些结构体字段、控制周期、滤波参数、CAN ID、PID/LQR 参数的改动，都可能同时影响多个任务。

## 修改规则

- 优先复用现有结构和命名，不引入新依赖。
- 优先在已有模块内修复问题，不新建抽象层包装现有逻辑。
- 除非任务明确要求，不修改：
  - `Drivers/`
  - `Middlewares/`
  - `components/algorithm/*.lib`
- 涉及硬件外设行为时，优先修改 `bsp/boards/` 或用户代码区，而不是直接改 HAL 生成模板。
- 若发现文档描述和代码不一致，以当前编译配置和当前任务创建代码为准，并在结论里明确指出差异。

## 验证方式

优先级从高到低：

1. 检查 `MDK-ARM/chassis_task.uvprojx` 是否真正收录了修改文件。
2. 检查任务创建链路和头文件引用是否闭合。
3. 如果环境有 Keil，可在 Windows 下执行：

```bash
"C:\\Keil_v5\\UV4\\UV4.exe" -b MDK-ARM\\chassis_task.uvprojx -o MDK-ARM\\build_log.txt
```

4. 查看 `MDK-ARM/build_log.txt` 中是否为 `0 Error(s), 0 Warning(s)`。

如果当前环境无法使用 Keil 编译，要在结论中明确说明“已完成静态检查，但未执行整工程编译验证”。

## 常见陷阱

- 文档中写“在跑”的任务，不一定真的在当前固件里创建。
- `application/` 下的实现，不一定就是 Keil 当前编译用的实现。
- 该工程大量使用全局状态和定时 `osDelay()`，控制周期改动会带来连锁影响。
- CAN 电机映射和零点处理写在业务代码里，修改前必须追踪接收回调和发送路径。

## 输出要求

完成工作后，结论至少说明：

- 改了哪些文件
- 这些文件为什么是正确的修改点
- 是否做了编译或静态验证
- 还有哪些未验证风险
