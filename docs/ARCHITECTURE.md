# StarFlight（StarFlight）架构

入口：`firmware/platform/stm32/startup_stm32f407xx.S: Reset_Handler` → `Core/Src/main.c: main` → `MX_FREERTOS_Init` → `firmware/app/startup.c: app_tasks_init` → `osKernelStart`。
`APP_RTOS_EXTERNAL_TASKS=1` 排除生成模板中的旧任务，实际创建配置以 `startup.c` 为准。

`Core/Drivers/Middlewares/.ioc` 保持 CubeMX 结构。手写代码位于 `firmware/app/services/algorithms/drivers/platform/boards/os`，分别负责业务与任务、服务、独立计算、设备、芯片适配、板级资源和必要同步。
CubeMX 开启 Keep User Code，重新生成后核查初始化桥接、DMA 回调、内核配置、时基和 CMake 源清单，完整构建后再上板。

传感器 → `Sensor_Data_Task_Proc` → 数据处理/独立姿态算法 → 姿态快照 → `Motor_Task_Proc` → 控制/PWM。SBUS 任务发布遥控快照，控制入口判定运行状态；显示和存储与控制任务分开。独立状态机、诊断及带校验双副本标定记录位于 `dev`，未纳入此基线。

## 任务与资源所有权

| 任务 | 优先级 | 配置栈 | 周期/等待 | 资源与失败策略 |
|---|---|---|---|---|
| Sensor | Realtime | 768 words | 1ms 基准、2ms读取惯性量、5ms解算、20ms磁场 | SPI2独占、算法上下文、标定与姿态快照；过期不补算积分 |
| Control | High | 512 words | 5ms固定周期 | 飞行状态、PID/混控；失联/姿态旧于100ms停止 |
| SBUS | AboveNormal | 256 words | 队列事件，100ms失联判定 | UART6 RX4×25，校准通道快照 |
| PC | Normal | 768 words | RX事件/5ms超时检查，遥测20–1000ms | UART1，RX4×100；应答、遥测与 LOG 统一发送，不远程解锁 |
| OLED | Idle | 1024 words | 50ms目标；曲线100ms采样 | U8g2静态帧缓冲、7页与中文菜单；OLED独立SPI分频/单线TX DMA，事务后恢复；2s完整重绘 |
| Storage | Idle | 768 words | 写请求事件 | 队列2个按值副本；原标定存储；独占4K scratch，busy超时5s |
| Flow | Idle | 128 words | 队列事件，100ms旧数据失效 | RX4×14；ISR不解析，不计算浮点 |
| Key/RGB | Idle | 各128 words | 5ms按键/低速LED | 按键消抖与600ms长按；只提交GUI输入，校准由菜单请求服务 |
| Log queue | 无独立任务 | 1024字节队列 + 8192字节启动缓存 | PC每轮最多64字节 | UART1 LOG事件；启动参数附设备毫秒时间戳，生产者不等待 |

配置栈为 FreeRTOS 的 32 位项数，不是实测余量。厂商 SDK/内核保持原版；GCC 使用匹配内核端口。具体版本见 SOURCES.md。

显示实现位于 `firmware/app/tasks/display_task.c`，通过 `gui_model_t` 向
`firmware/gui` 提供快照。图形核心无 HAL/FreeRTOS 依赖；`gui_presenter_t`
比较帧缓冲与上一帧，通过板级 `gui_display_port_write` 发送变化区域。
启动默认进入三行 12px 中文主菜单，菜单状态和有序键事件都在显示任务处理。
旧 `legacy/src/oled_proc.c` 不再编入目标；原OLED库源码保留，GUI改用板级初始化与传输接口。
页面、按键、移植方式与预览工具见 [GUI.md](GUI.md)。
