# StarFlight（StarFlight）串口协议

USART1 的遥测/日志复用见 [UART_LOG.md](UART_LOG.md)。本工程增加 `0x20F0`
LOG 事件，沿用原 v1 帧格式和 CRC；原遥测与查询结构保持兼容。

## 帧格式

`A5 5A | version:u8 | flags:u8 | sequence:u16 | command:u16 | length:u16 | payload | CRC16:u16`

版本 1；所有多字节字段小端。payload 最大 128 字节，接收超时 100 ms。CRC16/CCITT-FALSE：poly=0x1021、init=0xFFFF，覆盖 version 到 payload。
flags：0 请求、1 应答、2 主动事件。应答/状态事件 payload 首字节为状态码：0 成功、1 长度错误、2 不支持、3 范围错误、4 状态拒绝、5 忙、6 内部错误。
`0x20F0` LOG 事件 payload 是 1..64 字节原始 UTF-8 流，没有状态字节；其序号与遥测独立。

基础命令：1 版本、2 设备 ID、3 能力、4 状态、5 参数读、6 参数写。参数 ID 1 为遥测周期：读取 payload 为 `id:u16`，写入为 `id:u16 + value:u16`。各设备限制不同，先查能力。

编解码与分发在 `services/protocol`，不直接调用硬件或修改控制状态。中断交接数据，通信任务拥有解析器和发送；项目 `business` 回调通过业务接口提交或查询。分包、粘包、异常帧与队列丢包恢复由解析/传输层处理。旧客户端不兼容 v1，需配套固件和客户端。

## 查询接口

UART1，115200。遥测周期 20..1000 ms。状态 payload 共 14 字节：`status:u8 + state:u8 + roll:f32 + pitch:f32 + yaw:f32`，角度为 rad。
`0x2000` 姿态查询当前与状态命令返回相同的 14 字节，不返回角速度。

`0x2001` 只读诊断的成功数据：`fault:u8 + transitions:u32 + rejected_writes:u32 + sequence[3]:u32 + storage_busy:u8`，加上首字节状态码共 23 字节。sequence 顺序是 IMU、遥控、PID。
片内存储版本中，首个sequence取加载/保存的IMU兼容、磁椭球、加速度六面块的最新序号，满矩阵磁块加载优先；
该诊断不改变帧长度，也不新增串口参数写接口。详细校准系数与保存完成结果见MAG_CAL/STORAGE日志。

状态：0 锁定、1 解锁怠速、2 自稳、3 急停。故障位：bit0 遥控/通道、bit1 姿态、bit2 标定/存储、bit3 急停开关、bit4 时序/非法状态。
不提供串口解锁、飞控目标或电机输出写入。标定记录和状态转换见 [ARCHITECTURE.md](ARCHITECTURE.md)。
