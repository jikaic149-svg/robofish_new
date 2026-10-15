# RoboFish 3.1.5 / phase1-safety-ab 固件

本轮交付PC机器鱼工作站与S3 3.1.5 API-only固件，STM32保持已验证的一期安全/A/B镜像。2.4G接管、实物安全原因字段及S3受限升级事务尚未实现。详细状态见SAFETY_AND_UPDATE_DESIGN.md，逐文件摘要见manifest.json。

## STM32文件和地址

| 文件 | 地址/用途 |
|---|---|
| RoboFishFactory.bin | **0x08000000**；首次/恢复有线安装，含Bootloader、启动记录和两份应用 |
| RoboFishFactory.elf | 同一工厂镜像的GDB加载容器；调试符号仍使用应用ELF |
| RoboFishBoot.bin | 0x08000000；32KiB引导区，仅有线维护 |
| RoboFishF405.bin/.elf/.hex | APP A，**0x08010000**，最大448KiB |
| RoboFishF405_B.bin/.elf/.hex | APP B，**0x08080000**，最大448KiB |
| stm32_ab_manifest.json | 布局、长度、SHA-256和F405原生CRC参数 |
| debug/ | 配套Debug产物，不能与Release的启动记录/CRC混用 |

**应用BIN已移址，不能再把RoboFishF405.bin烧在0x08000000。** 单独更新应用会使旧启动记录的CRC失配；本轮首次部署/调试应使用匹配工厂整包。工厂安装会写Bootloader，不是日常应用OTA。

先全停、供电，连接SWDIO/PA13、SWCLK/PA14、GND和**NRST**。使用ST-Link时从包根目录执行：

```powershell
openocd -f stm32f405/openocd.cfg -c "program stm32f405/RoboFishFactory.bin 0x08000000 verify reset exit"
```

脚本要求复位捕获/停核、IWDG冻结读回及软件看门狗选项；不修改option bytes。不能确认该状态时拒绝擦写。S3的通用USB/Wi-Fi CMSIS-DAP仍有维护调试权限，它不等同带白名单、升级锁和回滚事务的受限OTA入口。

## 运行与看门狗

LoRa目标有效期为**3000ms联调初值**。发送端须周期续期；重复旧序号不续期，辅助更新不能恢复旧目标。仅按键发送的旧遥控器会在持续动作3秒后降级，详见LORA_PROTOCOL.md。

运行中失联：双桨中立，舵机保持最后成功写入角度。全停/退出后失联保持停机；电池异常仍全部中立。恢复条件满足后须新的有效运动指令。等待状态需要SPI新鲜，运行中的健康LoRa不被SPI断链打断。

IWDG在应用Reset_Handler首个调用启用，早于SystemInit及C++构造。名义32kHz LSI下启动约8.192s，运行约512ms；关键任务按各自100ms期限检查。正常上浮、等待和电池停机照常喂狗；一般输出告警也不取消喂狗。连续写入失败5次进入中立恢复，15次进入受控退出，保留40ms中立输出窗口后复位。致命异常锁住普通输出，专用无锁中立及记录出口仍可执行，然后停止喂狗/复位。

新应用先试运行；连续3轮关键任务和中立写入检查通过并持久确认后才允许启动。未确认复位回旧槽。每次Boot核对整镜像CRC及向量，启动记录用独立双扇区日志和最后提交字。

## 调试

源码工程的F5配置会构建并加载匹配Debug工厂镜像，再用APP A符号停在main。命令行调试先安装debug/RoboFishFactory.bin，启动OpenOCD，再打开debug/RoboFishF405.elf；连接后执行monitor reset init、break main、continue。不能仅load应用并沿用旧CRC记录。运行中暂停调试器可能留下旧PWM，先执行全停。

## ESP32-S3

3.1.5-robofish / xiaos3-8mb-nrst-d5配置，NRST为GPIO6。首次USB安装在esp32s3目录执行：

```powershell
python -m esptool --chip esp32s3 --port COMx --baud 460800 --before default_reset --after hard_reset write_flash @flash_args
```

Bootloader 0x0、分区表0x8000、otadata 0x1d000、应用0x20000、只读网页0x607000；DIO/80MHz/8MB。新3.1.5.amphipkg使用既有密钥签名。S3不再提供网页；旧只读分区保留以兼容存储初始化和回滚布局。S3自身OTA的延后健康确认及STM32受限无线升级仍需二期实现，不把现有通用DAP烧录当作已验证双区OTA。

## 验证边界

9组主机故障回归和网页检查、Debug/Release各3项ELF检查通过。覆盖启动门控、失联上浮、停止/电池屏障、旧视觉/自动结果拒绝、看门狗状态独立、输出失败阶梯，以及启动记录逐步写入/回收中断和坏镜像回滚。

尚未接线或烧录。IWDG实际LSI预算、PWM波形、电调丢PWM、SWD捕获/冻结以及真实断电恢复必须上板验证；软件模拟通过不等于实际电调已经停转。

PC工作站另见HOST_WORKSTATION.md和独立robofish-workstation-3.1.5-robofish.zip；主机界面不属于S3 OTA包。
