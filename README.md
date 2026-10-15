# RoboFish 双 MCU 工程

当前源码在本目录：`esp32s3/` 为 S3 相机、Wi-Fi管理API和 CMSIS-DAP 烧录工程；`stm32f405/` 为运动控制工程。两端通过 `common/spi_link_protocol.h` 定义的 SPI 协议通信。

## 代码结构

main只初始化并启动调度器；BSP管理硬件与IRQ；Task管理调度、控制权和唯一输出提交；`control/l0`适配来源，`control/l1`定义并校验四轴指令和观测，`control/l2`按尾部／双桨／胸鳍树分配；Device作为L3提交输出；channel与service保存消息类型和协议。公共纯算法保存在algorithm，不再保留平行的mode执行链。

入口：`Core/Src/main.cpp` → `bsp/BoardInit.cpp` → `src/task/RobotTasks.cpp`。运动输出在`MotionController.cpp`，视觉/PID在`MotionVision.cpp`。详见[工程约定](docs/PROJECT_DECISIONS.md)、[调试入口](docs/DEBUGGING.md)；此前交付的[数据流图](docs/dataflow-3.1.2.html)为3.1.2快照。

本轮控制代码与论文核查见[运动控制验收](.agent_reports/motion_control_acceptance_2026-10-15.md)。姿态、深度、速度经`MotionFeedback.cpp`接入；后续算法经`submitBodyCommand()`提交四轴需求，沿用现有授权代次。软件验收不等同于实机标定。

构建和日志位于build；firmware保留当前交付。唯一版本声明为S3 CMake PROJECT_VER，签名与打包自动读取构建版本。

## 接线

| 信号 | ESP32-S3 | STM32F405 | 接口 |
|---|---|---|---|
| SPI CS | GPIO1 | PB12 / SPI2_NSS | S3 U5.5 ↔ STM32 U3.5 |
| SPI SCK | GPIO2 | PB13 / SPI2_SCK | U5.4 ↔ U3.4 |
| SPI MISO | GPIO44 | PB14 / SPI2_MISO | U5.3 ↔ U3.3 |
| SPI MOSI | GPIO43 | PB15 / SPI2_MOSI | U5.2 ↔ U3.2 |
| SPI GND | GND | GND | U5.1 ↔ U3.1 |
| SPI 预留 | 空置 | 空置 | U5.6 ↔ U3.6 |
| SWCLK | GPIO4 | PA14 | 两板 CN1.3 |
| SWDIO | GPIO5 | PA13 | 两板 CN1.4 |
| NRST | **GPIO6 / D5** | NRST | 两板 CN1.1 |
| SWD GND | GND | GND | 两板 CN1.2 |

SPI 为 6 芯同脚号直连；SWD 为 4 芯同脚号直连。两块板另接各自的 XT30 供电线。

S3 板载摄像头引脚沿用指定参考工程：XCLK 10、SDA 40、SCL 39；D7..D0 为 48、11、12、14、16、18、17、15；VSYNC 38、HREF 47、PCLK 13。JPEG、12 MHz XCLK，传感器由 esp32-camera 驱动识别，保留参考工程的帧大小/画质配置和拍照、参数控制、MJPEG 服务。S3 WS2812 控制模块已移除，GPIO48 由摄像头 D7 独占；默认 UART 日志和 USB Serial/JTAG 副控制台关闭，GPIO43/44 用于 SPI，USB PHY 用于既有 TinyUSB DAP。

STM32 四路舵机保持左胸鳍 PA8、右胸鳍 PA0、上侧尾舵1 PA1、下侧尾舵2 PA15；ESC PB0/PB1、LoRa PA9/PA10、STM32 灯带 PB8。S3 的 WS2812 移除不改变 STM32 灯带。

## SPI 行为

S3 主机、STM32 SPI2 从机，Mode0、8MHz、128字节，目标1ms周期，单次CS仅128字节，CS释放后至少100μs再提交下一帧；运动控制仍为20ms。帧包含会话、序号、确认号、时间戳及CRC32，逐字段小端编码。心跳100ms超时只上报离线，保留LoRa，不自动停运动。

**LoRa 是运动主控。2.4G 只作为 LoRa 丢失后的备用入口。** 用户确认的接管规则是：检测到LoRa失联后，只接受新到的2.4G指令；LoRa恢复后优先接回。本轮LoRa目标有效期按用户确认设为3000ms联调初值；2.4G会话和仲裁仍属二期，**本版本继续关闭SPI运动入口和网页运动按钮**。SPI继续同步链路、执行器状态及电池电压。备用仲裁尚未启用。

演示动作已从STM32、SPI和网页API移除；原命令类型4被拒绝。其它运动消息格式保留供确认备用链路后使用。SPI不会将网页运动命令转发到LoRa模块；它是S3与STM32之间的连接。

STM32 SPI DMA1 Stream3 RX、Stream4 TX、Channel0，IRQ优先级5；任务优先级3、2048字节栈。S3 SPI任务及DMA ISR在Core1，优先级5、4096字节栈。时间戳用于往返及偏移估算，STM32为毫秒Tick；8MHz/1ms尚未接线实测。

## 电池电压

分电板VBAT经R3=10kΩ、R2=2.2kΩ分压，ESC_V_SENSE由U7.8→U13.8接到PC2/ADC1_IN12。ADC1扫描PC2和内部VREFINT，12-bit，ADC时钟21MHz、每通道480周期采样；DMA2 Stream0/Channel0，DMA/ADC IRQ优先级5。

RadioPoll保持原10ms周期，每100ms启动一次非阻塞DMA采集。ISR发布单槽原始样本，RadioPoll用出厂VREFINT标定值换算VDDA与电池电压，并进行1/4平滑滤波；MotionControl通过单槽Latest Value接收样本。采样超过500ms未更新或数据无效时，停止运动，网页显示采样异常。保护先检查未滤波电压，再做显示滤波，异常样本不进入滤波状态。

电池电压经SPI传到S3，GET /api/v1/robot返回battery_voltage（V）、battery_valid、battery_age_ms、battery_state；网页显示两位小数。普通7.4V标称2S电池的本项目运行范围为6.6～8.4V（包含边界）；低于6.6V显示“请充电，已停机”，高于8.4V显示“电压异常，已停机”，采样无效/过期显示“电压采样异常，已停机”。四舵机置中、两路电调中立，清除旧运动目标；恢复正常电压后仍保持停机，收到新有效控制指令才解除。通信、相机和采样继续。仅测总压，不能识别单节失衡；电池具体型号与ADC精度仍待核对。6.6V截止参考[Hobbywing](https://support.hobbywingdirect.com/hc/en-us/articles/23823729425043-Low-Voltage-Cutoff-LVC-tuning)的每节3.3V设置，上限参考[GensTattu](https://www.genstattu.com/blog/how-to-protect-your-batteries-as-long-as-possible-when-you-use-it)的普通锂电每节4.2V。

**螺旋桨电压前馈已按用户确认接入：2S电池，7.4V基准。** 每20ms使用当前运动输入经档位、差速和过渡计算后的两路有符号油门，分别乘以7400/实测电池mV，四舍五入到整数百分比并限制在[-100,100]。之后沿用原电调脉宽映射；零油门保持中位，正反向保持符号。电压来自100ms ADC采样及原1/4滤波，电压不在运行范围内、采样无效、为0或超过500ms时电调中立并保持全部运动停止，不继续用原油门。

补偿位于Device电气输出边界。每周期只补偿一次，运动/过渡状态保留未补偿目标值，不将上次补偿结果累乘。网页两路ESC百分比返回驱动API接受写入的补偿后控制值，不代表实测转速或推力。此比例模型仍需上板检查，尚未做螺旋桨推力标定。

## 使用PC机器鱼工作站

双击桌面“机器鱼工作站”或根目录“启动上位机.cmd”，自动后台启动并打开http://127.0.0.1:8765/。上位机位于esp32s3/AmphiLink_UI/，融合控制、原始/识别视觉、数据记录与绘图、日志、连接配网、调试和烧录；详见[工作站交付](docs/HOST_WORKSTATION.md)。无实物时一键进入测试模式，不需要额外运行仿真脚本。

S3 3.1.5不再托管网页，保留2.4GHz AP/STA、登录/配网/升级API、USB/Wi-Fi CMSIS-DAP及相机:81/stream、/capture等接口。PC台架模式通过/api/v1/robot读取SPI遥测；任务控制与状态应走LoRa，PC发送端/上行尚未实现，不能用管理链替代。实物识别接口、2.4G接管和受限OTA仍待接入。OpenOCD使用用户指定D:/OPENOCD/openocd-custom-20260922。

## LoRa 接收与顺序

LoRa 保留原帧格式，按来源号分别比较16位递增序号，拒绝旧命令，近期重复仅返回原ACK（若已启用）且不重复执行。串口错误会清错并重启接收；环形缓冲溢出/硬件错误后丢弃残行。默认ACK发射仍关闭，发送端重发暂缓。完整契约及遥控器重启限制见 [LoRa 协议](common/LORA_PROTOCOL.md)。

SPI 下一帧只在NSS高电平、两路DMA收尾完成后装载；短帧/OVR/部分启动失败停止DMA请求，等待EN清除后复位本外设并重新装载。ISR不忙等关闭DMA，不在RX完成回调内重启。无READY额外接线，8MHz/1ms/100μs间隔的实际裕量待逻辑分析仪确认。

主机故障回归：`python tests/run_host_tests.py --compiler C:/msys64/ucrt64/bin/g++.exe`，详见 [测试说明](tests/README.md)。

## 编译

在工程根目录：
```powershell
python esp32s3/AmphiLink_flash/build-v2.py
cd stm32f405
cmake --preset debug-local
cmake --build --preset debug-local
ctest --preset debug-local
cmake --preset release-local
cmake --build --preset release-local
ctest --preset release-local
```

S3 产物在 `build/esp32s3/`；STM32 产物在 `stm32f405/build/debug/` 和 `release/`。工具使用本机 ESP-IDF 5.3.2、ESP GCC、ARM GCC、CMake/Ninja。S3 构建直接使用当前工程源码，不再复制到旧 TEMP 工程；应用代码路径相对工程定位。

协议检查：
```powershell
gcc -std=c11 -O2 -Wall -Wextra -Werror -I common tests/spi_protocol_test.c -o build/tests/spi-protocol-test.exe
build/tests/spi-protocol-test.exe
```

## 固件交付与验证

打包产物和逐文件 SHA-256 在 `firmware/`。STM32首次安装使用 **RoboFishFactory.bin @ 0x08000000**；APP A为RoboFishF405.bin @ 0x08010000，APP B为RoboFishF405_B.bin @ 0x08080000，不能沿用旧应用烧录地址。S3 首次 USB 安装应使用随附 `flasher_args.json` 中的各镜像地址；本布局应用地址为 **0x20000**，不是通用的 0x10000。

新 S3 固件版本 `3.1.5-robofish`、硬件配置 `xiaos3-8mb-nrst-d5`。新 D5 签名 OTA 包供已安装该硬件配置的固件升级；原 D2 固件的硬件校验会拒绝 D5 包，首次更换配置请使用 USB 安装。保留既有签名密钥，未生成或发布新密钥。

已验证：ADC电压换算、网页电压/备用按钮检查、Demo指令拒绝、两端编译、STM32 Debug/Release IRQ向量与静态分配、41项BSP回归及电压停机/恢复、LoRa旧包/来源隔离、SPI事件顺序故障回归、协议CRC已知向量和128字节逐位破坏测试、序号回绕/旧响应检查、网页语法。实物尚未接线：未执行烧录、图像/网络运行、SPI波形/吞吐/时钟偏移、任务栈水位或运动验证。

参考来源：用户指定的 `codeFish(1)/codeFish/embedding` 相机实现和本地 esp32-camera/esp_jpeg 组件；官方 [ESP-IDF SPI](https://docs.espressif.com/projects/esp-idf/en/v5.3.2/esp32s3/api-reference/peripherals/spi_master.html) 与 [esp32-camera](https://github.com/espressif/esp32-camera)。

## 一期安全与A/B

见[权威设计及实现状态](docs/SAFETY_AND_UPDATE_DESIGN.md)。运行中LoRa目标3秒不续期则停双桨、四舵机保持最后成功写入角度；STOP17/电池停机清目标，恢复须新运动指令。普通告警不直接停止；正常上浮、等待和电池停机均喂狗，按任务完成期限监测卡死。首次有线安装与F5调试使用包含Bootloader、双记录和两份应用的工厂镜像；此维护流程会写Bootloader。S3受限无线升级事务、2.4G接管和网页原因字段未实现，通用CMSIS-DAP不等同受限OTA。
