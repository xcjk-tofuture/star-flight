# OLED 图形显示框架

128×64 单色 OLED，现有 SPI1 与片选接线。字体和二维图元使用 U8g2；
控制器配置与上电时序位于显示板级接口。目标刷新频率 20Hz，曲线按 10Hz 在后台采样。
此频率是调度目标，实际表现取决于高优先级任务负载及 SPI1 资源等待。

## 中文菜单与按键

启动默认进入一级主菜单，九个入口为飞行总览、三维姿态、趋势曲线、传感数据、
遥控通道、光流测距、系统诊断、校准管理、按键说明。
列表每屏三行、12 像素中文，反色圆角焦点平滑移动；滚动条和索引显示列表位置。
标题、状态、帮助和校准操作提示使用 12 像素中文，重要提示使用 16 像素中文。
参数、单位、图例及 6 像素高的快捷提示使用英文缩写。

| 操作 | 菜单 / 确认窗口 | 数据页面 |
|---|---|---|
| K1（PD0）短按 | 下一项，循环选择 | 快速翻到下一个主页面 |
| K2（PD1）短按 | 确认选择 / 进入 | 切换当前页面子视图 |
| K1 长按 ≥600ms 后松开 | 返回上一级；主菜单返回之前页面 | 返回主菜单；校准页返回校准管理 |
| K2 长按 ≥600ms 后松开 | 打开校准管理快捷入口 | 打开校准管理快捷入口 |

短按、长按在松开时各触发一次，不会长按后再额外触发短按。
按键任务每 5ms 扫描，四次一致读数消抖；时长使用 HAL 毫秒计时，
输入通过静态有序事件缓冲传给显示任务，不分配新 RTOS 队列。

校准管理包含遥控校准（校准中为“保存退出”）、磁力校准、返回菜单。
进入修改类操作后有确认/取消窗口，默认选中取消；K1 选择确认再按 K2 执行。
菜单只发出已有服务的校准请求，不直接读写传感器、Flash 或飞行控制参数。
校准开始和保存要求锁定状态，确认前与执行时均检查；解锁或飞行中显示“等待锁定 / 请先锁定飞控”。
已知外部 Flash 初始化失败时，写入类校准操作显示“闪存异常 / 校准数据无法保存”；
遥控无有效原始帧、磁力计未就绪或惯导仍在校准时，显示相应中文说明。
原业务请求的页面 19（遥控校准）和 20（磁校准）继续有效。

菜单组织和反色焦点参考 [OLED_UI](https://github.com/bdth-7777777/OLED_UI) 的 MenuPage/MenuItem 设计，
本工程使用独立静态菜单管理器与已有 U8g2 渲染，不依赖该参考项目的 GPIO、四键和定时器。

## 数据页面

| 页 | 内容 | K2 子视图 |
|---|---|---|
| 1 总览 | 大字横滚/俯仰、航向、温度、九轴/六轴、气压计与 Flash 状态 | 单一视图 |
| 2 姿态 | 真实姿态驱动的透视三维无人机线框、方向箭头、R/P/Y 数值 | 三维 / 人工地平仪 |
| 3 趋势 | 64 点滚动曲线；不同线型区分通道，缺失数据不连线 | 横滚/俯仰、陀螺仪、磁场、温度、气压、光流 XY |
| 4 传感器 | 三轴加速度、角速度、磁场与校准参数 | 当前采样 / 校准偏置 / 磁刻度、温度、气压 |
| 5 遥控 | 通道条形图和数值 | PWM 1–4 / PWM 5–8 / RAW 1–4 / RAW 5–8 |
| 6 光流 | XY 速度方向、测距和质量原值；失效显示 NO FLOW | 向量仪表 / XY 速度曲线 |
| 7 系统 | 器件与 RC/光流状态、Flash 启动结果、内存、日志、显示性能 | 状态 / 堆与通信 / 显示耗时及 SPI 错误 |
| 19 遥控校准 | MIN/NOW/MAX；校准期间仍显示有效原始帧的数据 | 通道 1–4 / 5–8 |
| 20 磁校准 | 转轴步骤提示、进度条、偏置 | 校准结束后返回总览 |

所有页面共用中文状态栏：校准中、磁校中、遥控校准、惯导异常、数据过期、待遥控、紧急、飞行、解锁、锁定。
缺失或无效数字显示 `--`。“待遥控”对应未得到有效的遥控快照，便于理解状态灯。
姿态数值显示角度（deg），后端模型使用 rad；系统状态不会因页面变化而改变。
中文使用 WenQuanYi Bitmap Song 的 12px / 16px 字形子集，UTF-8 解码。
只保留本界面用字，原始字形和许可证随子集 BDF 一同保存。

## 分层和扩展

```mermaid
flowchart LR
    A[传感器/姿态/遥控快照] --> B[display_task: gui_model_t]
    B --> C[gui_dashboard: 页面与历史数据]
    C --> D[gui_canvas / U8g2: 1024B 帧缓冲]
    D --> E[gui_presenter: 比较上一帧]
    E --> F[gui_display_port: SPI1 批量发送]
```

| 文件 | 职责 |
|---|---|
| `firmware/app/tasks/display_task.c` | FreeRTOS 调度、按键请求、只读数据快照 |
| `firmware/gui/gui_dashboard.*` | 页面、状态提示、曲线数据与绘制；无 HAL/RTOS 依赖 |
| `firmware/gui/gui_menu.*` | 静态菜单项表、三行列表、焦点动画、滚动位置；无 HAL/RTOS 依赖 |
| `firmware/gui/fonts` | 12px / 16px 中文子集、字形清单和可重建 BDF 源 |
| `firmware/gui/gui_canvas.*` | 字体、线/虚线、圆、矩形、条形图、定点文本、变化帧提交 |
| `firmware/gui/gui_plot.*` | 带有效标记的 64 点三通道历史、自适应量程、曲线绘制 |
| `firmware/gui/gui_scene.*` | Rz(yaw)Ry(pitch)Rx(roll) 旋转、固定相机透视投影、人工地平仪 |
| `firmware/platform/stm32/gui_display_port.*` | 上电/复位/首帧、OLED独立SPI时钟、DMA页跨度、互斥 |
| `firmware/boards/stm32/gui_oled_config.h` | 控制器类型、列偏移、时序、SPI分频、整帧刷新周期 |
| `firmware/third_party/u8g2` | 固定版本的原始绘图核心和字体，未修改上游源码 |

添加页面时，在 `gui_dashboard` 添加一个绘制函数、页编号、`view_count` 和 `root_items` 菜单项，
增加 `gui_model_t` 的只读字段，并在 `read_model` 复制数据。
其他任务通过 `display_service.h` 的请求接口操作页面，不能直接修改页面或访问 OLED。
绘图可组合已有控件，也可对 `canvas.graphics` 调用选入的 U8g2 绘图图元，例如圆角框、三角形和 XBM 位图。

## 性能与资源

- 绘图缓冲 1024 字节，上一帧缓存 1024 字节；均为静态 SRAM。
- 六组历史缓冲在后台采样，切换曲线时已有近期数据；间隔过大时插入断点，不补画假采样。
- 每个 8 像素高的页只传输首个至最后一个变化字节；未变化页不发送。
  整帧最多八个跨度，每个最多 128 字节；短命令轮询发送，页数据用 DMA2_Stream3，避免逐字节发送和任务抢占造成数据时钟停顿。
- SPI1 互斥覆盖页地址和数据，跨度结束释放；发送失败将该页标为未知，下一帧整页重试，避免半次传输留下残影。
- 绘制完毕再提交，页面切换不额外清屏；跨 MCU 移植不需要改图形核心。
- OLED事务使用独立 /64 分频（当前1.3125MHz）、单线TX和CS/DC建立/保持间隔。
  SPI1互斥锁内切换，事务结束恢复原SPI1时钟、SPE和方向，供外部Flash继续使用。
- 初始化等待电源300ms，复位低电平20ms、释放后等待30ms；关屏配置和清空显存，完整GUI首帧写入后才发AF开屏。
- 每两秒重新设置起始行、偏移、方向和地址模式并完整重绘。四线SPI不提供显存读回/ACK，定期刷新用于修复软件差分缓存无法察觉的漏写。
- 底部不再绘制贯穿128像素的分隔线，避免与故障白条混淆。
- 数字格式化走整数 `snprintf`，不调用浮点 printf；OLED 任务保留 1024 words 栈，RTOS 堆维持 24576 字节。
- 系统页显示上一帧耗时与实际发送字节数；启动日志的 `GUI` 记录显示累计帧、跨度、字节和错误。
- 三维是单色线框透视视图，采用真实姿态快照。没有深度缓冲、光照或实体面遮挡。

## 移植

同尺寸单色屏只需提供初始化和 `gui_write_span_fn`：
`page` 表示 y=page×8，`x` 是起始列，`bytes` 为垂直八像素、低位在上的字节数组。
传输回调在成功后返回 0，失败返回非零；调用返回后不得继续使用传入指针。
异步 DMA 后端应复制数据或等待完成，以保证缓冲区生命周期。

```c
static gui_canvas_t canvas;
static gui_dashboard_t dashboard;
static gui_presenter_t presenter;

gui_canvas_init(&canvas);
gui_dashboard_init(&dashboard);
gui_presenter_init(&presenter, my_screen_write_span, my_context);
/* 每轮取得只读 model 后： */
gui_dashboard_update(&dashboard, &model);
gui_dashboard_render(&dashboard, &canvas, &model);
gui_present(&presenter, &canvas);
```

更大或彩色屏需要调整画布尺寸、布局和呈现格式；当前预设专用于 128×64。
板级默认 `GUI_OLED_CONTROLLER=GUI_OLED_SSD1306`，沿用工程原来的SSD1306配置假定。
确认是SH1106后设为 `GUI_OLED_SH1106`，其默认列偏移为2；可以另定义 `GUI_OLED_COLUMN_OFFSET`。
OLED不能通过该四线SPI连接自动识别型号，选择值不代表探测结果。
GPIO、SPI 模式和芯片复位留在板级接口，不进入图表或三维代码。

## 显示异常诊断

先观察 `[OLED] controller_profile=... SCK=1312500Hz` 和
`display_on=1 complete_first_frame=1`。两秒和八秒的OLED日志还报告
configured/on/init、DMA错误、完成超时及最后HAL状态；这些仍是MCU发送状态，不能证明屏幕接收正确。

此前直接使用原SPI1的42MHz：SSD1306四线SPI最短周期100ns（10MHz），
SH1106四线SPI最短周期随逻辑电源范围为250/500ns（4/2MHz），原频率超过这两类控制器的时序范围。
参考芯片原厂手册：[SSD1306 Table13-4](https://cdn-shop.adafruit.com/datasheets/SSD1306.pdf)、
[SH1106 SPI时序](https://www.pololu.com/file/0J1813/SH1106.pdf)。

5V接到整板输入与直接接OLED模块VCC是不同情况，需要结合模块稳压、实际逻辑电源和控制器型号确认。
软件时序修正不能替代对供电接法和电平的确认；本次修改是否消除5V空白需上板观察。

## 桌面预览

预览使用与固件相同的渲染代码和模拟数据，不连接设备，不代表上板刷新效果。
需主机 GCC；PNG/GIF 转换额外使用 Python Pillow：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\gui-preview.ps1
python .\tools\gui-preview-images.py .\build\gui-preview .\build\gui-preview\images
```

PBM、PNG 页面和三维动画可独立查看。固定的 U8g2 来源和字体许可证见 `docs/SOURCES.md`。
新增中文文案后，先用 `tools/gui-fonts.py` 对固定 U8g2 源码目录重新生成字体；
该工具扫描 `firmware/gui` 中的文本和 `fonts/characters.txt`，并输出字形清单。
固件正常用 VS Code 的 `CMake: build` / `VLLINK: flash HEX` 任务构建和手动烧录。
