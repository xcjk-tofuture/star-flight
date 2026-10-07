# StarFlight（StarFlight）串口协议

USART1 的遥测/日志复用见 [UART_LOG.md](UART_LOG.md)。本工程增加 `0x20F0`
LOG 事件，沿用原 v1 帧格式和 CRC；原遥测与查询结构保持兼容。

`services/protocol` 与 `algorithms` 不依赖芯片、RTOS、业务全局变量。STM32、TM4C 和星璇各自包含相同版本的协议源码；修改公共源码须运行主机检查并核对三处 SHA256。星璇只复用协议，不复用底盘业务。

协议 v1：`A5 5A | version:u8 | flags:u8 | sequence:u16 | command:u16 | length:u16 | payload | CRC16:u16`。所有多字节字段小端，CRC16/CCITT-FALSE（poly=0x1021, init=0xFFFF）覆盖 version 到 payload。长度最大 128，帧间接收超时 100ms。请求 flags=0，应答=1，主动遥测=2。响应第一字节为错误码；异步遥测使用相同 status 数据结构并保留第一字节状态码 0。

命令：1版本、2设备ID、3能力位、4状态、5参数读、6参数写，0x1000底盘速度，0x2000无人机姿态。参数读 payload=id:u16，参数写=id:u16+value:u16。参数1为遥测周期 ms，仅 RAM 生效，范围由设备最小周期至1000ms。底盘速度为三个 IEEE754 binary32，单位 m/s、m/s、rad/s；NaN、Inf、越界、欠压和队列满必须拒绝。UAV 不开放串口解锁或执行器输出，扩展写命令返回 unsupported。

状态 payload：status:u8 + project_state:u8 + 三个 float32；底盘为vx/vy/wz，无人机为roll/pitch/yaw（rad）；底盘其后追加 wheel_count:u8 + voltage:f32 + IMU 六个原始 i16 小端（acc±2g与gyro±500deg/s，量程待上板核对）。能力位0=状态、1=遥测周期读写、2=底盘速度。

帧编解码与命令分发均不直接接触硬件。ISR只提交字节；通信任务负责解码和命令提交；控制任务拥有闭环状态，通过受保护快照供遥测使用。队列满不覆盖未处理命令，返回 busy；接收丢包后重置解析器，旧目标由控制超时归零。

BREAKING CHANGE：上位机默认改用协议v1；ROS配套源码同步迁移。旧ROS/匿名ANO客户端需保留旧固件或使用旧提交，协议版本不自动猜测。

补充：底盘状态0=idle，1=active，2=undervoltage，3=command-timeout，4=uncalibrated。TM4C默认能力不含底盘运动，标定完成之前拒绝速度命令；其状态帧的IMU原始字段当前填0，表示没有接入统一遥测，不可作为有效IMU测量。STM32 IMU配置源码为acc±2g/gyro±500deg/s（仍需上板核对）。星璇状态角度使用rad，0x2000为查询，协议不提供远程解锁或飞控目标写入。
