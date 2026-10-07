# 依赖与来源

原文件版权、许可证和版本注释保留；第三方版本不能确认时不补造版本或许可证。

- STM32Cube FW_F4 V1.26.2；FreeRTOS 10.3.1 原内核。
- U8g2 绘图核心：[官方仓库](https://github.com/olikraus/u8g2)，固定提交
  `d6c8499c5f2707cac8eccd09fd8f677d12b17977`，位于 `firmware/third_party/u8g2`。
  选用源码与 Misc Fixed 字体原样保留；来源和许可证见该目录的 `UPSTREAM.md` / `LICENSE`。
- GUI 中文字体：同一固定 U8g2 版本携带的 WenQuanYi Bitmap Song 0.9.9.8，
  按实际界面用字生成 12px / 16px 子集；原始声明为 GPL v2 with font embedding exception。
  字形源、生成数组、版权说明位于 `firmware/gui/fonts`；与 U8g2 绘图库的 BSD 许可证分别保留。
- GCC CM4F 端口：[匹配内核的官方源码](https://github.com/FreeRTOS/FreeRTOS-Kernel/tree/V10.3.1-kernel-only/portable/GCC/ARM_CM4F)，位于 `firmware/os/ports`。
- GNU 启动汇编：[ST CMSIS Device F4 v2.6.8](https://github.com/STMicroelectronics/cmsis-device-f4/tree/v2.6.8/Source/Templates/gcc)，未升级原 HAL。
- ARM GNU 13.3.Rel1：Windows 包 SHA256 `e46fda043c0ce83582bc8db4b3ef85f77f4beb7333344c2f4193c17e1167a095`；CI Linux 包 SHA256 `95c011cee430e64dd6087c75c800f04b9c49832cc1000127a92a97f9c8d83af4`。
- xPack OpenOCD 0.12.0-7：Windows 包 SHA256 `6bfd3c97135aafef8affc9af1acf34fd0e2b9ca26044506f6abd7f95b7630052`。
