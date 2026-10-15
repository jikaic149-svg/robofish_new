# RoboFish STM32F405

在工作区根目录构建。项目内源码、工具链文件、链接脚本、构建输出和 VS Code 配置均相对工作区定位。业务代码已收齐到 `src/`，本机工具位置单独保存在 `CMakeUserPresets.json`；移动工程后无需修改源码路径。

采用 ARM GCC、CMake ≥ 3.24、Ninja、OpenOCD/ST-Link。无需 CubeMX 或 `STM32CUBE_F4_PATH`。

## 本机编译

本机预设使用用户指定的工具：`D:/CMAKE/bin`、`D:/Ninja`、`D:/GCC/ARMGCC/10 2021.10/bin`、`D:/OPENOCD/openocd-custom-20260922/bin`。CMake 已在 PATH 中。

```powershell
cmake --preset debug-local
cmake --build --preset debug-local
ctest --preset debug-local

cmake --preset release-local
cmake --build --preset release-local
ctest --preset release-local
```

产物分别在 `build/debug/`、`build/release/`：`RoboFishF405.elf`、`.bin`、`.hex`、`.map` 和 `compile_commands.json`。Debug 为 `-O0 -g`，Release 使用 `MinSizeRel/-Os`。

其他机器将工具加入 PATH 后使用 `debug` / `release` 预设。也可设置 `ARM_GCC_BIN`、`OPENOCD_BIN`，或在本机预设中指定 `CMAKE_MAKE_PROGRAM`、`OPENOCD_EXECUTABLE`。本机预设和生成的 `.vscode/launch.json` 已列入 `.gitignore`。

依赖通过 FetchContent 获取并校验 SHA-256，缓存位于 `build/deps/`，Debug/Release 共用。首次配置需要访问官方 GitHub；此次复用了本机已有的相同上游压缩包，后续配置可使用工作区缓存。

| 依赖 | 固定 commit |
|---|---|
| STM32F4 HAL | `b6f0ed3829f3829eb358a2e7417d80bba1a42db7` |
| CMSIS Device F4 | `0fa0e489e053fa1ca7790bb40b4d76458f64c55d` |
| ARM CMSIS | `2b7495b8535bdcb306dac29b9ded4cfb679d7e5c` |
| FreeRTOS Kernel | `0adc196d4bd52a2d91102b525b0aafc1e14a2386` |

## 分电板与舵机

根据 2026-10-13 EDA 实时原理图/PCB 回读，以及用户给出的 FRONT1/FRONT2 和尾舵位置定义。STM32 板 U13 与分电板 U7 使用 **8 芯同脚号直连线束**，1→1 至 8→8；9/10 是接地固定焊盘。两端间距均为 1.0 mm，采购线束时仍须核对两端壳体及针号。

| 逻辑位置 | 分电板信号脚 | U7 ↔ U13 脚号 | MCU / PWM |
|---|---|---|---|
| FRONT2 左胸鳍 | CN5.3 | 2 | PA8 / TIM1_CH1 |
| FRONT1 右胸鳍 | CN8.3 | 7 | PA0 / TIM5_CH1 |
| 后部上侧尾舵机 1 | CN7.3 | 6 | PA1 / TIM5_CH2 |
| 后部下侧尾舵机 2 | CN6.3 | 5 | PA15 / TIM2_CH1 |

原先将 PA15 改到 PB10 的方案已不适用。最终证据记录在 [hardware-map.json](stm32f405/hardware-map.json)，固件资源在 [BoardResources.h](stm32f405/bsp/BoardResources.h)。原理图、PCB 网络一致不代表实物导通、方向或波形已经测量。

两路 ESC 保留 PB0/TIM3_CH3、PB1/TIM3_CH4。LoRa 保留 PA9/PA10、USART1 115200-8N1；灯带保留 PB8、TIM4_CH3 + DMA1 Stream7。水泵占位驱动及状态已删除，泵命令 8/18/19 继续被拒绝。

8 MHz HSE，经 PLL 得到 168 MHz；APB1/APB2 为 42/84 MHz。舵机及 ESC 为 50 Hz；舵机脉宽 500–2500 μs，ESC 为 1000–2000 μs。Bootloader位于0x08000000（32KiB），应用A/B分别位于0x08010000/0x08080000，各限448KiB；普通SRAM末端64字节固定保留故障记录，CCM未使用。

当前源码已完整独立构建。当前架构与约定见 [PROJECT_DECISIONS.md](../docs/PROJECT_DECISIONS.md)，验证见根目录 [validation.json](../validation.json)。沿用该版的静态 FreeRTOS 分配、HAL/RTOS SysTick 和启动顺序；驱动在调度器启动后初始化，并保留 2000 ms ESC 解锁等待。控制/视觉任务周期 20 ms、优先级 4/3、栈 4096/2048 字节；RadioPoll 周期 10 ms、优先级 2、栈 2048 字节。

## OpenOCD 烧录和调试

给板子供电，连接 ST-Link 的 SWDIO/PA13、SWCLK/PA14、GND。STM32 板 CN1.1 为 NRST，CN1.2 为 GND，CN1.3/4 分别经 R14/R15 接 SWCLK/SWDIO；CN1 不给板子供电。

```powershell
cmake --build --preset debug-local --target flash
```

`flash`现在执行首次/恢复有线整包安装：RoboFishFactory.bin从0x08000000写入Bootloader、双记录和两份应用；会覆盖引导区，不修改option bytes。正常应用OTA不得调用此维护目标。须连接NRST，脚本验证调试冻结和软件看门狗选项。

命令行调试使用两个终端：

```powershell
# 终端 1：保持服务运行，默认 GDB 端口 3333
cmake --build --preset debug-local --target debug-server
```

```powershell
# 终端 2：GDB 连接，复位暂停并设置 main 断点
cmake --build --preset debug-local --target debug
```

首次及每次更换固件须先安装匹配工厂镜像，再用应用A的ELF作符号。不能只load应用而保留旧CRC启动记录。F5自动加载RoboFishFactory.elf并复位后停在应用main；命令行使用flash目标安装后再连接GDB。

VS Code 已安装 Cortex-Debug。CMake 配置后从相对路径模板生成本机 `launch.json`；按 **F5** 选择 `F405: OpenOCD flash and debug`，将构建 Debug、启动 OpenOCD、烧录并停在 `main`。**Ctrl+Shift+B** 构建 Debug；也可运行 `Flash` 任务。

编译和向量检查不替代实机验证。此次 OpenOCD 配置解析通过，但探针连接返回 `Error: open failed`，实际烧录、断点调试、舵机方向和波形尚未验证。

参考：[CMake 工具链](https://cmake.org/cmake/help/latest/manual/cmake-toolchains.7.html)、[OpenOCD 烧录](https://openocd.org/doc/html/Flash-Programming.html)、[OpenOCD 与 GDB](https://openocd.org/doc/html/GDB-and-OpenOCD.html)。

## 代码入口

按TI-Template职责分开：main只初始化BSP、创建RobotTasks并启动调度器；bsp/BoardInit.cpp维护时钟/GPIO/DMA/PWM/UART及MSP与硬件IRQ；src/task/RobotTasks.cpp负责RadioPoll与设备启动；MotionController.cpp负责控制与快照，MotionVision.cpp负责同一控制器的视觉任务。当前只保留四舵机/双电调硬件实现。调试步骤与断点见[DEBUGGING.md](../docs/DEBUGGING.md)。CubeMX生成的main.c与stm32f4xx_hal_msp.c均不加入CMake，避免MSP重复定义。

## 当前交付与联调

产物另含RoboFishBoot、RoboFishF405_B、RoboFishFactory.bin/.elf及stm32_ab_manifest.json。LoRa目标有效期默认3000ms，配置见src/config/ControlConfig.h。发送端须周期续期，旧序号不续期；模式切换类命令不能不断换序号重放。视觉/自动化需人工授权及当前controlGeneration，旧结果不能跨退出恢复。9组主机故障回归、Debug/Release各3项ELF检查通过。IWDG名义启动8.192s、运行512ms，真实LSI、PWM与断电行为仍待上板；不要在运行中直接连接并暂停调试器。
