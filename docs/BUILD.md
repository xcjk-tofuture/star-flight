# StarFlight构建与验证

依赖：ARM GNU 13.3.Rel1、CMake ≥3.21、Ninja、OpenOCD；VS Code 扩展 C/C++、CMake Tools、Cortex-Debug。`ARM_GCC_PATH` 可指向工具链根目录，相关 bin 目录加入 PATH。

```sh
cmake --preset debug
cmake --build --preset debug
cmake --preset release
cmake --build --preset release
```

输出为 `build/debug` 或 `build/release` 下的 `xingxuan_uav.elf/.hex/.bin/.map` 和 GCC `.su`。
本机 VS Code 的任务配置已设置 ARM GCC/Ninja 路径；`Ctrl+Shift+B` 执行构建，
`VLLINK: flash HEX` 手动烧录，F5 用 VLLINK CMSIS-DAP 启动调试并停在 main。
移植到其他电脑时需调整 `.vscode/tasks.json` / `launch.json` 中的本机工具链和探针路径。

连接匹配的板卡与调试器后，可运行：

```sh
cmake --build build/debug --target flash
```

下载配置使用 `program ... verify reset exit`。ELF 由启动汇编复制 `.data`、清零 `.bss`；堆、栈和 `.bss` 不作为烧录数据。Flash/RAM 分区见链接脚本，`.su` 不替代 RTOS 实测栈余量。

合并保留 master 原链接脚本，未引入 dev 的段布局改动。普通 CMake 构建可完成，
仍有原 `.data` 段分配和 `.heap/.stack` LMA 调整诊断。
`tools/validate.py --preset ...` 和固件 CI 会严格拒绝这些诊断；主机验证可独立执行。
这一限制不通过跳过检查或修改库来消除，后续链接脚本决策应单独处理。

## 软件验证与串口工具

```sh
python tools/validate.py --cc gcc
python tools/validate.py --preset debug
python tools/validate.py --preset release
python -m pip install -r tools/requirements.txt
python tools/serial_cli.py --port COM7 identify
python tools/serial_cli.py --port COM7 status
python tools/serial_cli.py --port COM7 diagnostics
python tools/serial_cli.py --port COM7 record capture.jsonl --seconds 10
python tools/replay_log.py capture.jsonl
```

使用实际串口，Linux 例如 `/dev/ttyUSB0`。录包文件必须不存在。回放用于帧与状态时间线检查，不是高频传感器解算或 HIL。主机测试使用本机 gcc；构建报告输出到 `build/reports`。CI 在 dev/master push 与 PR 上运行主机测试、Debug/Release 并保存固件/map/报告。

### 串口诊断与回放

工具先读取设备 ID 和能力位，再发送操作命令。状态/诊断输出带字段名称和单位，同时保留 `data_hex`。
STM32/TM4C 底盘状态与 StarFlight 状态均可解码；TM4C 不支持底盘参数保存/诊断。
遥测周期参数 ID 1 可在三种设备读取和写入：StarFlight 为 20..1000 ms，底盘为 50..1000 ms。
STM32 底盘扩展参数的范围见 PROTOCOL.md；写入 ACK 表示受理，仍需再次读取确认应用。

```sh
python tools/serial_cli.py --port COM7 capabilities
python tools/serial_cli.py --port COM7 read 1
python tools/serial_cli.py --port COM7 write 1 100
```

新录包使用从开始计时的单调秒数；回放兼容旧版绝对单调秒数。
回放检查行长度、字段类型、时间顺序、状态/诊断长度与有限数值；报告最后的解码状态/诊断、状态时间线、错误应答、最长状态事件间隔与事件序号间断。
`estimated_missing_status_events` 只统计前进距离小于 32768 的序号缺口；重复或倒序单独计为间断。
间断可能来自设备发送失败、链路丢失或主机未及时接收；接收时间间隔不是控制任务抖动实测。
原始 IMU 字段保持 counts，不用未经验证的量程换算。回放不向设备发送命令。

StarFlight LOG 事件 `0x20F0` 单独处理，没有响应状态字节。
回放按 LOG 序号重组跨帧 UTF-8，去除重试重复帧，丢帧后丢弃不完整行；
输出 LOG 帧数、行数、缺口、重复数和最后一行，同时继续生成遥测状态时间线。
实时查看可执行 VS Code 的 `USART1: telemetry + LOG` 任务，详见 [UART_LOG.md](UART_LOG.md)。

## 硬件验证

先在无桨台架验证八通道遥控标定、状态守卫、失联/旧姿态/急停、传感器量程、控制增益及混控方向、总线时序、任务栈和 Flash 掉电；完成这些检查后再独立验收飞行。CubeMX 重生成回归、六面标定、yaw 混控和定高仍待完善，见 TODO。

Debug/Release 软件构建及主机测试已有通过记录；探针烧录、实时任务水位和硬件行为仍待实际板卡验证。发布固件、map、报告应关联源码版本保存，构建目录不提交。

## 版本兼容与回滚

旧源码提交 `977d5b068c73dccd3e709bc8414d04f70d11acac` 可在独立 checkout 中检查；协议/参数格式按对应固件版本使用。新参数记录不会自动载入旧裸记录，遥控需重新校准。TM4C 仅同步适用的基础变更。不要以旧参数格式解释新版记录。
