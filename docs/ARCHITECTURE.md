# StarFlight无人机架构

## 启动入口

`firmware/platform/stm32/startup_stm32f407xx.S: Reset_Handler` 初始化数据段后进入 `Core/Src/main.c: main`。
`main` 初始化 HAL、时钟、外设及 DMA，再调用 `MX_FREERTOS_Init` 和 `osKernelStart`。
`Core/Src/freertos.c` 的 USER CODE 桥接到 `firmware/app/startup.c: app_tasks_init`。
`APP_RTOS_EXTERNAL_TASKS=1` 排除生成模板里的旧任务；实际任务表以 `startup.c` 为准。

## 目录与生成代码

`Core/`、`Drivers/`、`Middlewares/`、`.ioc` 保留 CubeMX 生成结构。手写代码位于 `firmware/`：

| 目录 | 职责 |
|---|---|
| `app` | 启动、任务、业务流程及状态机 |
| `services` | 控制、采集、协议、显示和参数管理 |
| `algorithms` | 独立 PID、滤波、运动学和姿态计算 |
| `drivers` | 电机、传感器、显示和存储设备 |
| `platform` | UART、SPI、PWM、时间等芯片实现 |
| `boards` | 引脚资源、板卡参数及链接脚本 |
| `os` | 消息、快照、互斥、故障钩子及同版本 GCC 移植适配 |

依赖方向：应用 → 服务 → 设备 → 平台 → SDK；服务调用独立算法，任务及共享资源使用必要的 RTOS 适配。`services/legacy` 保留旧流程与适配接口，目录名不代表所有旧模块已彻底拆分。

CubeMX 调整外设后开启 Keep User Code，核查 USER CODE 桥接、DMA 回调、FreeRTOS 配置、HAL 时基和 `cmake/sources.cmake`，再完整构建。厂商源码及 RTOS 内核不随手写层重排。

## 采集与控制数据流

传感器驱动 → `services/legacy/src/AHRS.c: Sensor_Data_Task_Proc` → 标定/滤波 → `algorithms/attitude/attitude.c` → `os/flight_snapshot.c` → `app/tasks/flight_control_task.c: Motor_Task_Proc` → 状态机 → PID/混控 → `drivers/motor/uav_actuator.c` → `platform/stm32/uav_pwm_hal.c`。

UART6 中断向 SBUS 队列交帧，由 `sbus_proc.c` 校验、换算通道、判断失联并发布快照。控制任务读取遥控和姿态快照；姿态存 rad/rad/s，旧控制入口显式换成度/度每秒。状态守卫在 PWM 输出之前执行。

`AHRS.c` 仍承担采集和旧标定适配，不是纯算法模块。当前 yaw PID 输出已计算，但四路混控中的 yaw 项仍被注释；CH5 高档仍选择自稳，不启用定高。六面标定和校准温度/有效期政策仍待完成。

应用启动先释放 SPI1/2 的所有片选，记录实际驱动读取的传感器 ID。
BMI088 失败阻断启动陀螺仪校准和解算；磁力计失败保留错误标志，允许惯性校准与六轴解算。
传感器任务开始正常磁转换后，每隔至少 500ms 重查一次 ID，最多四次；恢复后重新启用九轴输入。
外部 Flash 初始化失败仍继续诊断启动，参数服务拒绝未识别器件的记录读写。

## 任务与所有权

| 任务 | 优先级 | 栈（32 位项） | 周期 / 资源 |
|---|---|---|---|
| Sensor | Realtime | 768 | 2ms 惯性整块采集；5ms 目标区间融合；20ms 磁场、50ms 气压、100ms 温度；SPI2 独占 |
| Control | High | 512 | 5 ms；状态机、PID、混控和输出 |
| SBUS | AboveNormal | 256 | 队列 4×25 字节；100 ms 失联判定 |
| PC | Normal | 768 | RX 4×100 字节，等待 5 ms；UART1 查询/遥测/LOG，TX DMA |
| OLED | Idle | 1024 | 50 ms 目标；中文菜单、曲线与三维姿态；2s 完整重绘 |
| Flash | Idle | 768 | 按值写请求队列 2；校验、双副本提交、读回 |
| Flow | Idle | 128 | RX 4×14 字节；100 ms 旧数据失效 |
| Key / RGB | Idle | 各 128 | 5ms 按键消抖、600ms 长按、菜单事件和指示灯 |
| Log queue | 无独立任务 | 8192B 启动缓存 + 1024B 队列 | PC 每轮发送最多 64 字节 LOG |

配置栈是 FreeRTOS 项数，实际余量需测量。Sensor 发布姿态和有效性，Control 发布状态和故障，其他任务复制快照。OLED 与 Flash 共享 SPI1，`os/spi1_mutex.c` 保护完整片选事务。

显示任务通过只读 `gui_model_t` 驱动 U8g2 静态画布、三行中文菜单、二维曲线和三维姿态。
OLED 使用独立 /64 分频和单线 TX DMA；事务结束恢复 Flash 的 SPI 配置。
复位后关屏清空显存，完整首帧提交后开屏；原 OLED 库源码保留，旧 `oled_proc.c` 不再编入。
校准从菜单确认窗口提交，显示与服务均检查锁定状态。页面和移植方式见 [GUI.md](GUI.md)。
详细启动参数与时间戳、USART1 LOG 复用见 [UART_LOG.md](UART_LOG.md)。

## 飞行状态与标定存储


状态沿用0锁定、1解锁怠速、2自稳、3紧急停止。所有守卫在本控制tick输出PWM之前评估。
CH1/2/3<=1050、CH4>=1950、CH5<=1100必须连续保持1000ms，且遥控/姿态有效、未校准/存储，才可在锁定与解锁间切换。
长时间持续保持同一手势只触发一次；松开后才允许下一次。自稳模式不能用该手势直接切换。
只有低油门<=1100时，CH5高档>1400才从解锁进入自稳；CH5低档回到解锁。
CH5最高档目前仍为自稳，定高模式没有启用。
CH8>1400立即急停；遥控丢失、无效通道、姿态无效或旧于100ms、校准/存储冲突均在已解锁时锁存急停。
故障后必须恢复健康数据、油门<=1050、CH5低档、CH8低档，才回到锁定，随后重新保持解锁手势；链路恢复不会自动恢复输出。
每个非自稳tick清空控制积分/滤波状态，避免再次进入控制时沿用旧历史。
故障位：bit0遥控/通道、bit1姿态无效/陈旧、bit2校准或存储、bit3急停开关、bit4控制延迟/非法状态。控制迟于当前计划唤醒超过10ms时跳过追赶并触发保护。
0x2001为只读诊断：成功数据u8故障位+LE u32状态转换次数+LE u32拒绝/失败写入次数+3个LE u32标定记录sequence（IMU/遥控/PID）+u8存储忙。
原状态和姿态查询包长度保持不变；没有新增远程解锁或电机控制命令。

## 标定存储

记录schema1，显式LE字段，CRC32和提交标记；每类两个独立4KiB扇区。
IMU地址0x1000/0x2000，遥控0x3000/0x4000，PID0x5000/0x6000；旧裸数据所在sector0保留。
不自动载入旧裸数据，因为无法验证来源、版本或CRC；原数据不会被新版写入覆盖。
IMU记录72字节：acc offset/scale、gyro offset/scale、mag offset/scale共18个f32。
所有值必须有限并有数值边界；mag scale必须0.1..10。当前只恢复磁标定；gyro 每次启动重新校准，加速度沿用未标定采样值，六面流程和其应用仍待完成。
遥控记录32字节：8对max/min u16，每项<=2047、max>min、跨度>=100。无有效记录时通道无有效连接，禁止解锁；从中文菜单的“校准管理 / 遥控校准”重新校准所有8通道，切换各通道覆盖完整范围后“保存退出”。
校准开始重置极值，只有新鲜有效帧更新极值；非法范围不能保存，保存失败保留校准状态以便重试。
PID记录60字节：roll/pitch/yaw/roll-rate/pitch-rate的Kp/Ki/Kd，共15个f32；有限非负<=2000且不能全0。读失败保留编译默认增益，不载入随机Flash值。
写入只允许锁定状态，按值队列深度2；有待写记录时状态机禁止解锁。所有Flash操作由存储任务互斥执行，SPI1事务与显示通过既有互斥适配隔离。
未识别的Flash ID拒绝记录；队列满/非法记录/存储验证失败计入诊断。底层SPI超时仍进入安全停机。
每次更新写另一个副本，最后写提交标记，读回验证；上次记录保留。相同内容不擦写。
物理地址分配只涵盖参数记录；未来记录/回放的Flash环形日志必须另划分地址，不可复用这些扇区。

### 待完善的标定与控制

六面数据采集/拟合/应用、完整标定温度/有效期政策、定高控制、机载高频日志及其解算回放仍待后续依赖完成。
旧加速度占位采样/未执行的拟合已移除，不再以阻塞延时模拟六面标定；原磁标定统计仍需台架回归。

### 启动陀螺仪质量守卫

`algorithms/calibration/gyro_calibration.c` 只依赖 C 标准库；Sensor 任务拥有上下文。
独立 gyro 5Hz Butterworth 校准通道保留绝对零偏，acc 用 30Hz 通道；稳定500ms后累计500个连续姿态帧样本。
控制 gyro 使用独立40Hz带宽；区间角增量按原始采样和实际微秒dt积分，姿态融合目标200Hz。
运动、非有限值、重力或方差超限时重采；断采/滤波重配置重置窗口，但不延长总期限。
仅完整通过时更新 gyrooffsetbias；30秒失败锁存“校准失败”，姿态无效、红灯常亮，静置后RESET重试。
质量边界位于 `boards/stm32/imu_calibration_config.h`：重力9.80665±0.8m/s²、每轴角速度≤0.15rad/s、
校准gyro标准差≤0.015rad/s、acc标准差≤0.15m/s²。门限仍需本板实测，不表示温度已经稳定。
新融合以重力/可用磁场初始化，按实际dt更新四元数；重力修正按模长加权，磁场按年龄、模长和航向创新检查。
磁场异常保持惯性融合，恢复要十个新样本及一秒权重过渡；界面显示实际磁场参与状态。
数据链路、配置、官方参考与验证见 [IMU_PIPELINE.md](IMU_PIPELINE.md)。
需要先在无桨台架完成遥控全通道标定、状态矩阵、Flash掉电、时序/栈及控制增益验收，再进行独立飞行验收。


相关实现：`app/flight_machine.c`、`services/legacy/src/flash_proc.c`、`services/parameters/calibration_record.c`、`services/parameters/param_journal.c`。协议接口见 [PROTOCOL.md](PROTOCOL.md)，验证方法见 [BUILD.md](BUILD.md)。
