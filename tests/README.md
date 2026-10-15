# 主机故障回归

在工程根目录执行（需要主机 GCC/G++ 和 Node，不使用 ARM 交叉编译器）：

```powershell
python tests/run_host_tests.py --compiler C:/msys64/ucrt64/bin/g++.exe
```

运行 ADC 换算、动态前馈、41 项 BSP 检查、LoRa 来源/序号与 ACK 去重、实际 MotionController 停机/恢复流程、实际 SPI BSP DMA/片选时序，以及 C 帧协议和网页显示检查。产物在系统临时目录，测试结束自动清理。

共 15 组原生回归，其中新控制组覆盖 L0 意图与统一配方、L1 四轴与采样合法性、L2 三子树约束、真实 L3 PWM／安装极性、Task 传感器接线及完整控制管线。`control_architecture_test.py` 同时检查层依赖、叶节点不互调、外部不能绕过 L2 根访问叶节点。测试不会把待标定 roll 增益或未测水动力写成能力通过。

测试替换 HAL/FreeRTOS 和执行器写入边界，不启动真实任务或连接设备。SPI 测试直接包含生产 SpiLink.cpp，交换 RX/TX DMA IRQ 与 NSS 释放顺序，注入短帧、超长帧/OVR、部分启动失败、DMA EN 延迟关闭及队列溢出。电压停机测试使用真实 MotionController 和运动计算，模拟最终驱动提交失败；PID、自动化、LoRa、手动入口均检查电压门控。

这些检查验证状态和数据处理，不代表已测得任务 WCET/栈水位、ADC 精度、真实 DMA IRQ 延迟、SPI 最小片选间隔或电调波形。上板后仍需万用表与逻辑分析仪核对。

新增安全回归直接编译真实MotionController：检查3秒失效、SPI启动限制与运行豁免、固定舵机零推力上浮、一般告警/中立恢复/40ms受控退出、任务期限与回绕、旧视觉/自动化代次拒绝。BootStore回归直接编译生产记录存储与选择代码，逐个中断内容/CRC/提交字写入，破坏旧记录页回收，检查镜像尾损坏和试运行回滚。ARM CTest同时检查A/B/Boot向量、容量、固定.noinit及Reset_Handler首个看门狗调用。
