# 调试入口

在 `robofish_new` 打开 VS Code，运行 `Configure STM32 Debug`，然后按 F5。只打开 `stm32f405/` 时运行 `Configure Debug` 后按 F5。两种目录共用 `stm32f405/.vscode/launch.json.in`，CMake 根据本机工具配置生成 launch.json。

F5构建并加载匹配Debug工厂镜像（含Boot与CRC记录），使用APP A符号停在main，同时设置 Error_Handler 和栈溢出断点。命令行仍可用 `debug-server` 与 `debug` 目标；Debug 保持 `-O0 -g`，Release 保持 `-Os`。未连接实物时只验证配置和符号，不执行烧录。

| 关注问题 | 代码与断点 | 检查内容 |
|---|---|---|
| 上电/时钟/引脚 | `bsp/BoardInit.cpp` 的 `initializeBoard`、`SystemClock_Config` | HSE/PLL、TIM/UART/DMA 句柄 |
| 驱动启动 | `task/RobotTasks.cpp` 的 `radioTask` | 调度器已启动、2000ms电调解锁、驱动返回值 |
| LoRa旧包/拒绝 | `LoRaService::handle` | command.source/sequence/code，status_，sources_ |
| 串口错误恢复 | `LoRaUart::onUartError` | huart1.RxState，接收中断、错误标志 |
| 电压停机/恢复 | `MotionController::controlTick` | state_.diagnostics.battery.rawMillivolts、safety.mode、generation、fullStopEpoch_ |
| 输出提交失败 | `MotionController::recordFrameAttempt` | diagnostics.lastAttemptFrame、lastWrittenFrame、writtenMask、frameFailures |
| 视觉/PID | `task/MotionVision.cpp` 的 `visionTick` | state_.vision 与PID状态 |
| SPI错帧 | `bsp/SpiLink.cpp` 的 `progressTransfer` | phase、NSS、receiveDma/transmitDma.State/NDTR、transferErrors |

`task/RobotTasks.cpp` 持有 gDriver、gMotion、gLoRa。原流程不变：BSP初始化 → 创建RadioPoll → 调度器启动 → 设备初始化/解锁 → MotionControl与VisionControl启动。RadioPoll 10ms，运动/视觉20ms；DMA/串口IRQ优先级5。

调试器暂停会改变实际PWM/通信时序。时序裕量、采样误差及任务栈水位需要上板测量；主机故障回归入口为 `python tests/run_host_tests.py --compiler C:/msys64/ucrt64/bin/g++.exe`。

当前A/B布局：首次/恢复安装用RoboFishFactory.bin @ 0x08000000，应用A/B分别位于0x08010000/0x08080000。不要单独load应用却保留旧CRC启动记录；先安装匹配工厂镜像，再连接GDB。NRST须接线，OpenOCD检查停核与IWDG冻结；普通复位不清DBGMCU冻结位。断点暂停不能保证电调停止，先执行全停。

新增关注点：MotionController::serviceHealth 的任务最后完成时间，state_.safety.mode 与generation、outputRecovery_及consecutiveWriteFailures_；bsp/RuntimeSafety.cpp 的故障阶段和固定0x2001FFC0记录；boot/BootStore.c 的最新日志记录、candidate/confirmed/state及原生CRC。视觉/自动化异步消息须携带授权时的controlGeneration，不能取到旧结果后用新代次重新包装。
