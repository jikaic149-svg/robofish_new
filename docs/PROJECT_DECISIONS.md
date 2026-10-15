# 当前工程约定

更新：2026-10-15，S3固件3.1.5，STM32运动控制树已实施。保留STM32 HAL/GCC与ESP-IDF环境。源码独立构建，不同步外部快照。

| 职责 | 当前目录/入口 |
|---|---|
| 初始化与调度器启动 | stm32f405/stm32f405/Core/Src/main.cpp |
| MCU外设、引脚与IRQ | stm32f405/stm32f405/bsp/，总入口BoardInit.cpp |
| 产品任务与编排 | stm32f405/src/task/RobotTasks.cpp；MotionController.cpp；MotionVision.cpp |
| 来源与策略适配（L0） | stm32f405/src/control/l0/ |
| 四轴与观测校验（L1） | stm32f405/src/control/l1/ |
| 目标分配（L2） | stm32f405/src/control/l2/Allocator → tail／propulsion／pectoral |
| 器件输出与电气标定 | stm32f405/src/device/motion/ |
| APP纯算法 | stm32f405/src/algorithm/ |
| 类型化消息与快照 | stm32f405/src/channel/ |
| 协议与接收服务 | stm32f405/src/service/lora/；common/ |
| S3应用与板级实现 | esp32s3/AmphiLink_flash/firmware-v2/components/APP、Module、BSP |

- main只初始化BSP、创建RobotTasks并启动调度器。RadioPoll仍10ms/优先级2/2048字节栈，运动/视觉仍20ms/优先级4、3/4096、2048字节栈；HAL设备在调度器启动后初始化，电调解锁等待2000ms。只使用静态FreeRTOS分配。
- F405明确使用当前四舵机/双电调硬件，已删除另一代硬件的条件输出代码、未调用包装、旧Wi-Fi/HTTP OTA配置和只写不读状态。用户明确保留视觉/PID/自动化、手动failsafe与电池保护。STM32水泵占位、AP/STA监控、OTA租约和ESP兼容层已删除；刷写由外部SWD完成。任务周期、栈、优先级及协议编号保持。
- HSE为8MHz；RF_HAS_NETWORK_LINK=0保持。MSP初始化唯一归BoardInit.cpp，CMake不编译CubeMX生成的main.c或stm32f4xx_hal_msp.c；.ioc仅作资源参考。
- 电池按普通7.4V标称锂电2S：6.6～8.4V含边界，先检查未滤波电压。异常/无效/超过500ms时四舵机置中、两ESC中立，恢复后等待新有效指令。前馈基准7.4V，每次补偿当前实际输入，不回灌补偿结果。
- LoRa为主控，按来源号＋16位递增序号拒绝旧包与重复执行；UART继续字节IT＋软件环形缓冲，错误恢复与丢残行保留。发送端重发、重启序号契约与ACK空口时序暂缓。
- SPI保持S3主机、STM32从机，8MHz/128字节/目标1ms，CS高至少100μs；NSS释放且双DMA收尾后装载下一帧。100ms心跳离线只诊断，保留LoRa。2.4G备用控制等待明确LoRa失联规则；保留协议编号，当前拒绝备用运动请求。摄像头鉴权按用户要求暂缓。
- 版本只在S3 CMake PROJECT_VER声明；发布/打包从构建元数据或镜像描述符读取。构建中间文件与日志集中build，firmware仅保留当前交付。签名沿用既有密钥；密钥不进入交付包。
- 验证：两端构建、STM32两种配置CTest、7组主机回归和网页检查通过。GDB断点符号与两种工作目录配置已检查。未烧录或上板，ADC校准、PWM/SPI波形、调度裕量和栈水位仍未测。当前证据见根validation.json。

历史过程日志、旧迁移/审计快照和重复旧包已由本轮清理替代。docs/dataflow-3.1.2.html仅保留此前交付的数据流图快照；当前电压保护与实现路径以上述说明为准。

2026-10-14 STM32支持裁剪验证：使用原生FreeRTOS静态任务及BSP毫秒时钟；删除不实际输出日志的ESP宏。HAL移除Flash/FlashEx、RCCEx、PWREx、TIMEx编译源，保留ADCEx供ADC中断回调引用。Debug/Release、启动/IRQ检查及7组主机回归通过，BSP检查41项。ESP32源码、配置、镜像、签名包未修改或重建；当前S3版本保持3.1.4。实物仍未烧录。

2026-10-14 安全重设计仅完成框架讨论，见[安全框架草案](SAFETY_FRAMEWORK.md)及[双区升级框架](FIRMWARE_UPDATE_FRAMEWORK.md)。最新确认：LoRa优先、2.4G备用，运行中全断则双桨中立、舵机保持最后成功写入角度并依靠浮力；S3烧录/SPI断链/双控全断仅禁止从停机状态启动，不中断健康LoRa；主动全停/退出或看门狗复位后等待新指令；电池异常仍停机，一般告警只显示；网页需显示禁止启动和异常原因。两端执行升级均须先全停，运行中可上传缓存，升级锁期间拒绝运动启动。用户要求固定向量/烧录区间、双区OTA及S3持久保存固件；建议沿用S3现有8MiB双OTA，STM32增加固定Bootloader、本地A/B和双启动记录。用户本轮已将S3烧录/OTA及原因展示纳入设计范围，旧的ESP32冻结约束不阻止这部分框架讨论。当前源码、协议和固件均未实施这些新规则。验证只含源码核对、分区容量/扇区边界核算；未构建或做断电测试。链路续期/超时、新会话规则、IWDG预算和首次部署方案仍待确定。

2026-10-14 一期实现完成：按用户授权实现3000ms LoRa目标有效期、固定舵机零推力上浮、全停/电池/异步代次退出屏障及任务期限喂狗。输出失败5次转中立恢复、15次转40ms受控中立退出；锁暂时失败后的恢复也清目标，不复活旧油门。IWDG从应用Reset_Handler覆盖启动，再切运行预算。Bootloader整镜像原生CRC、双记录最后提交字、一次试运行与回滚、A/B链接及固定64字节.noinit已接入。Debug/Release各3项CTest、9组主机回归和实际工厂镜像解码通过，未上板。S3/RFS1未变，2.4G和受限无线升级/网页原因仍二期；跨复位LoRa新会话待定。权威说明见SAFETY_AND_UPDATE_DESIGN.md。

2026-10-14（UTC）PC工作站与S3网页职责调整。原因：用户要求从Link烧录页扩展为统一机器鱼工作站，并全部在电脑运行。决策：运行/实验与设备/开发两层，9页共用本机服务；两种视觉为原始画面＋识别跟踪；使用指定D:/OPENOCD/openocd-custom-20260922。任务控制/上行保留LoRa入口，HTTP/SPI管理运动入口拒绝；仿真一键开启且阻止实物维护。后台1Hz记录30,000样本、同单位绘图与CSV/JSON/PNG，来源隔离，服务退出前导出。密钥可选DPAPI保存，不预填弱口令、不自动轮换设备密钥。涉及AmphiLink_UI、S3 provisioning_server及组件CMake、工具打包与工作站文档。S3 3.1.5去除网页注册/嵌入，API和相机、分区/引脚/任务保持。验证：61项PC回归、Edge完整操作/主题/窄屏、OpenOCD语法/target解析、S3构建及既有密钥签名验证通过。未上板；实物LoRa/识别入口、真机安全原因、2.4G接管与受限OTA仍未完成。详细证据见validation.json及HOST_WORKSTATION.md。

2026-10-15（UTC）运动文档审查修订。原因：协同章节存在推力投影、强制动作顺序和跨原型外推错误，用户审查后授权“修改”。决定：撤回以轴向推力计算横滚向心力、先姿态后推力的固定串行、J 越大效率越高、时间不对称必降 St 及固定切换阈值；保留论文限定工况和本机待验证方案。按两篇用户 PDF 区分三关节尾部 CFD 与背鳍／臀鳍＋单桨实验，以及动能指标与实验 CoT。涉及 MOTION_LAYERING_DESIGN.md、根目录 pectoral_fin_depth_control_report.md。验证：PDF 构型与关键数据、公式算例、链接、表格及 48 条文献编号检查通过，证据见 build/motion-review-20261015/corrections-validation.json。边界：仅文档修订，固件未改，未操作实物；本机推力、来流、姿态权限和能耗分配仍待实测。

2026-10-15（UTC）运动控制实施全局计划。用户要求按层逐步落实，先出计划，并明确重点是 L2 具体执行。MOTION_CONTROL_IMPLEMENTATION_PLAN.md 给出基线／接口契约、L3、L2 单机构、L2 混合、L1、L0、实机标定及后续闭环的推进顺序。详细内容集中在 L2：输入输出、逐周期计算、配方、双桨共同／差动、尾部相位与幅值、胸鳍成对约束、限速、部分提交后的状态接续及数值算例。尾摆取反修复、饱和优先级和档位位置等行为变化分别验收；旧视觉的独立尾／桨请求保留迁移依据。验证仅为源码核对、公式算例、文档格式与链接检查，L2 扩写之外的计划章节保持原内容；证据见 build/motion-control-plan/detail-validation.json。未开始阶段 0，未改变控制代码或运行设备；真实参数与运动能力仍待台架／水池证据。

2026-10-15（UTC）用户随后授权完成所有层代码，并将 tidy 范围扩大至整个 robofish_new，要求最终 Git 交付。现已接通 L0 来源适配、L1 四轴与采样校验、L2 根→尾部／双桨／胸鳍三子树、现有 L3 Device／BSP；Task 保留唯一相位／成功名义历史和输出提交权。统一已有 90°／25°／1.5Hz 硬件参考配方，删除旧 mode 五对文件与虚拟油门驱动状态，不把参数统一写成实物标定完成。尾部取反倒游被移除，纯尾反向清目标退出，Hybrid 反向由桨执行、尾归中；安装电气极性与电压补偿仅在 L3 应用。反馈接口支持姿态／深度／速度的本地时间、序号、有效期与显式重启；算法经 submitBodyCommand 沿用本地授权代次，无新 PID 或伪造观测。

验收：15组原生回归、15个控制文件依赖检查、网页检查通过；Debug／Release 的 A／B／Boot／Factory 构建及各3项 CTest 通过；PC 新 DLL 隔离回归84项通过。PC 目标预览复用同一分配根，已加载旧 DLL 的运行服务保持数据，下次进程首次使用按源码时间重建。全仓清理56个已确认派生文件（6,941,870B），旧源码恢复副本和运行遥测保留在 build；正式文档与工具状态未清删。论文复核分类及 L2 实际树见[验收文档](../.agent_reports/motion_control_acceptance_2026-10-15.md)，当前机器证据更新于 validation.json。未烧录或操作实物；roll 增益为0，真实方向／推力／胸鳍来流和混合收益仍须台架／水池验证。
