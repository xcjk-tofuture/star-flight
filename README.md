# StarFlight（星璇）

STM32F407 / FreeRTOS 无人机固件。

`master` 为集成固件，包含中文 GUI、USART1 遥测/日志、飞行状态守卫和校准参数存储；`dev` 用于功能开发。

- [入口、架构与任务](docs/ARCHITECTURE.md)
- [构建与验证](docs/BUILD.md)
- [串口协议与接口](docs/PROTOCOL.md)
- [中文菜单、图表与显示移植](docs/GUI.md)
- [遥控接收与未校准状态排查](docs/RC_LINK.md)
- [启动参数与 USART1 日志](docs/UART_LOG.md)
- [IMU采集、滤波、姿态与校准](docs/IMU_PIPELINE.md)
- [手动校准与数据存储现状](docs/CALIBRATION.md)
- [F407片内Flash分区与参数扩展表](docs/FLASH_LAYOUT.md)
- [未完成工作](docs/TODO.md)
- [依赖与来源](docs/SOURCES.md)

项目合集：[Starbase](https://github.com/xcjk-tofuture/starbase)。
