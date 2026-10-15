# STM32F405 平台文件

从工作区根目录使用 CMake 预设构建；本目录的 `CMakeLists.txt` 由根工程加载。

编译、OpenOCD 烧录、GDB/VS Code 调试及分电板映射见 [工作区说明](../README.md)。
`RoboFishF405.ioc` 仅作 MCU/引脚/时钟参考，构建使用这里的 HAL 初始化和 FreeRTOS 配置。
