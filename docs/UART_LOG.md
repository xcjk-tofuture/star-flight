# USART1 遥测与日志

USART1（PA9/PA10）使用 115200、8N1。遥测和日志均使用原协议 v1 的
`A5 5A | version | flags | sequence | command | length | payload | CRC16`。
共享协议编解码源码没有修改。

| 类型 | flags | command | payload |
|---|---|---|---|
| 遥测 | EVENT=2 | 0x0004 | 原状态数据：status、state、roll/pitch/yaw，角度单位 rad |
| 日志 | EVENT=2 | 0x20F0 | 1 至 64 字节 UTF-8 文本，无额外状态字节 |

0x20F0 是本工程的 LOG 事件，避免占用 dev 分支的 0x2001 诊断命令。
遥测和日志分别维护序号；原遥测结构、序号、查询和周期设置保持兼容。
一条日志可以分成多个帧，接收端按日志字节流重组换行，不应逐帧单独解码 UTF-8。

`printf` 经现有 `fputc` 接口进入日志队列。启动阶段最多保留 8192 字节，
运行队列容量为 1024 字节。调度器启动后先发送启动缓存，再发送运行日志。
`uav_logf` 格式化整行后一次入队，空间不足时整行丢弃，生产者不等待；
`printf` 仍按字节入队。`log_drop_bytes` 记录未进入缓存或队列的字节数。
PC 通信任务独占 USART1 发送，先处理应答及到期遥测，再发送最多一个 64 字节日志块。
发送使用已配置的 DMA2_Stream7，板级接口复制一份帧数据，并等待发送完成回调；
等待时让出 CPU，完成超时为 50ms。超时或 DMA 错误时只中止 TX，保留 RX。
LOG 帧失败时保留当前数据块和序号，下轮重试，成功后才读取下一块。
`UART_TX` 区分成功帧、启动错误、完成超时和 DMA 错误；`log_drop_bytes` 只统计入队丢弃。
监视脚本检查 LOG 序号，丢帧后丢弃不完整行并报告 `[WARN] LOG sequence gap`，避免把半行误当完整日志。
原 UART3 日志任务不再创建，避免两个消费者分走同一队列里的字节。

## 查看

关闭占用 COM7 的串口助手，在工程根目录运行：

VS Code 可以直接执行任务 `USART1: telemetry + LOG`，在终端分别显示日志和遥测。

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\uart-monitor.ps1 -Port COM7
```

默认只显示 `[LOG]` 日志。加 `-ShowTelemetry` 同时每秒显示一次 `[TEL]`，
脚本仍会处理和校验全部遥测帧。Ctrl+C 退出并释放串口。

## 详细启动参数与时间戳

启动日志通过 `uav_logf(level, module, format, ...)` 生成。例如（格式示例）：

```text
2026-10-07 15:20:01.123 +08:00 [LOG] [0000000012 ms][INFO][UART] USART1 baud=115200 data=8 parity=NONE stop=1 ...
```

最前面的日期时间是电脑收到完整日志行的时间；`[0000000012 ms]` 是设备生成该行时
距离启动的毫秒数，来自 HAL tick。缓存延迟发送不会改变设备时间戳。
设备没有设置日历时间，因此电脑时间和设备运行时间分别保留。
重放文件时，电脑时间表示重放时刻。

记录内容：

- 固件 Debug/Release、编译时间和编译器版本，MCU ID、修订号、Flash 容量及复位标志。
- SYSCLK/HCLK/PCLK1/PCLK2，PLL 时钟源、M/N/P/Q 和 RCC 寄存器。
- 六路串口的波特率、有效数据位、校验、停止位、BRR 和引脚；SPI1/2 的实际总线频率、分频、模式和引脚。
- 外部 Flash 的读取 ID、预期 ID、初始化返回值，以及继续诊断启动的提示。
- `EXT_FLASH`为可选W25诊断；`STORAGE backend=INTERNAL ready=1`表示片内参数后端已初始化，启动不擦除。
- 已保存磁椭球参数会打印`MAG_CAL loaded full_matrix=1`、各行矩阵（×1000）、偏置、场强和RMS。
  手动校准只有`COMPLETE saved=1 readback=1`才表示已保存并用于融合；等待写入时不提前报成功。
- 六面加速度系数启动打印`ACC_CAL loaded faces=6`及零偏/比例；采集中记录每面完成、方向/静置/噪声原因及偏斜重采。
  保存成功为`ACC_CAL COMPLETE saved=1 readback=1 applied_before_LPF=1`；诊断首个sequence涵盖三类IMU校准记录。
- 每个资源初始化的开始、返回值和剩余堆；每个任务的句柄、优先级、配置栈大小。
- 调度器启动情况、遥测周期、LOG 命令和缓存大小。
- PC 任务运行约两秒和八秒后：传感器初始化标志、姿态有效标志、实际 ID、传感器数据、UART 发送统计、OLED/PC/Sensor 历史最小栈余量、堆余量及日志丢弃数。

先烧录新固件，打开监视任务，再按板子的 RESET，才能看到完整启动日志。
两秒时传感器可能仍在校准；八秒记录便于查看校准后的状态。

## 传感器检查与解算模式

应用启动时先将 Flash、OLED 和 SPI2 的四路片选拉高，再执行器件初始化。
`SPI_CS` 的 SPI2 `high_mask` 按 ACC/陀螺仪/磁力计/气压计映射至 bit0..bit3，空闲期应为 `0xF`。
板级接口观察现有驱动的实际 SPI 事务，`SENSOR_ID` 不额外发起读命令：

| 器件 | 片选 | 预期 ID |
|---|---|---|
| BMI088 加速度计 | PE15 | 0x1E |
| BMI088 陀螺仪 | PD8 | 0x0F |
| 磁力计 | PD10 | 0x48 |
| SPL06 | PD9 | 0x10 |

`transfer=OK` 表示 HAL SPI 传输返回成功，器件识别还需比较实际 ID。
汇总记录的 `seen_mask` 同样使用上述 bit 顺序；未观察到 ID 时，对应的 `0x00` 只是占位值。

2026-10-02 的 `a5a3768` 提交增加了 `SensorError` 对整个姿态解算的阻断。
现在按器件用途处理：BMI088 初始化失败时为 `BLOCKED`；BMI088 正常但磁力计检查失败时
为 `6AXIS`，坏磁力数据不参与滤波，航向缺少磁场参考，可能漂移；两者正常时为 `9AXIS`。
气压计失败仍报告故障，不阻断姿态滤波。故障标志不会因采用六轴模式而清零。
磁力计失败时拒绝磁校准请求，避免六轴模式陷入无法完成的磁校准。

磁力计启动检查失败后，传感器任务在正常读取和触发转换已开始的情况下，
每隔至少 500ms 重查一次 ID，最多四次，输出 `MAG_RECHECK`。
读到预期 `0x48` 后，清除磁力计初始化失败标志，重新计算 `SensorError` 并恢复磁场输入。
四次均失败时保留六轴模式。初次 ID 为 0x00 并不证明后续磁力数据不存在，
应结合 `SENSOR_DATA` 和重查结果判断。

当前采集/滤波/融合链路见 [IMU_PIPELINE.md](IMU_PIPELINE.md)。`GYRO_CAL` 每秒显示
滤波后校准进度和重采原因，30秒超时输出 `FAILED ... retry=RESET`。
`IMU_PIPE` 报告实际Hz、间隔范围、读错误、重置及重力/磁场参与状态；
`MAG_READY` 表示初始化ID正常，是否实际参与融合以 `mag_used` 为准。

`SENSOR_DATA` 为当前发布的滤波后采样值，`scale=1000` 表示每项数值除以 1000 得到浮点值。
`bad_fields` 的 bit0..8 按 acc xyz、gyro roll/pitch/yaw、mag xyz 排列；非有限或超出日志转换范围的字段用 0 占位并置位。

重放保存的原始串口数据：

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\uart-monitor.ps1 -ReplayFile capture.bin -ShowTelemetry
```

这是运行期日志通道；CPU 进入停机钩子之后，通信任务无法继续发送排队日志。
此时仍需调试器查看调用现场。OLED 任务栈已在应用配置中调整为 4 KiB。
