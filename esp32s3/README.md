> 当前机器鱼集成版本为 **3.1.2-robofish**，NRST GPIO6/D5；增加板载相机、SPI心跳和网页控制，移除S3 WS2812。当前接线、编译和固件见 [工程说明](../README.md) 与 [交付目录](../firmware/README.md)。下面保留原3.0.0开发包的说明，首次安装新D5配置按当前工程说明执行。
# rlyoulink 私人开发包

包含 Windows 上位机、ESP32-S3 固件源码、定制 OpenOCD、签名发布工具和当前 3.0.0 升级包。内部保留 `AmphiLink_UI`、`AmphiLink_flash` 名称，供现有脚本相对定位。

## 启动和连接

1. 双击根目录 `启动上位机.cmd`，浏览器访问 http://127.0.0.1:8765/ 。不要直接打开 index.html。
2. 有线：用数据线连接 XIAO ESP32-S3，选择 USB 连接。无线：电脑先连接设备的 `rlyoulink-` 热点，通过设备管理页扫描并配置 2.4 GHz 路由器网络，然后将电脑连接同一局域网，在上位机填写设备实际 IP。
3. 设备热点、管理和升级的默认密码均为 `12345678`。已保存的路由器密码不属于此默认密码。
4. 先确认 PC ↔ ESP32 状态，再点击连接目标。上位机自动读取 CPUID、DBGMCU_IDCODE 和容量，加载相应 STM32 配置，不需要手选系列。烧录 STM32 时选择正确目标固件；暂停、继续、寄存器、断点操作共用该会话。

本机如果已有旧上位机占用 8765，启动器会打开已有服务。需要使用新目录代码时，应先确认并关闭旧上位机服务，再启动；不要结束不明 Python/OpenOCD 进程。

## 接线和适配范围

探针为 XIAO ESP32-S3（8 MiB Flash）；STM32 系列的 SWD 接法相同，目标板引脚位置以其原理图为准：

| XIAO | STM32 |
|---|---|
| D3 / GPIO4 | SWCLK |
| D4 / GPIO5 | SWDIO |
| D2 / GPIO3 | NRST（开漏，平时释放） |
| GND | GND |

目标应正确供电，信号采用 3.3 V 电平。GPIO6 / D5 已停用，不接 NRST。上位机当前 `reset_config none`，复位运行通过 SWD 发出软件 SYSRESETREQ，不代表拉动 NRST。

USB 探针序列号固定 `DCB4D93B2B00`，升级脚本检查 MAC `dc:b4:d9:3b:2b:00`。换 ESP32 探针仍需要修改并验证身份适配；换下表中的 STM32 目标不需要选择系列。

### STM32 自动识别范围

身份读取会短暂停核，随后恢复原先运行状态；容量通过对齐 32 位读取提取半字字段。驱动注册的 OTP Bank 不计入应用 Flash 容量，也不进入普通烧录范围。界面不再逐条刷出全部扇区记录，后端仍保留完整扇区表用于检查。

| 系列 | 已加入的芯片组 | 调试内核 |
|---|---|---|
| F1 | 小、中、大、XL 容量；F100、F105/F107 | Cortex-M3 |
| F4 | F401、F405/407/415/417、F410/411/412/413/423、F42x/43x、F446、F469/479 | Cortex-M4 |
| G4 | G43x/G44x、G47x/G48x、G49x/G4Ax | Cortex-M4 |
| H7 | H72x/H73x、H74x/H75x、H7Ax/H7Bx | Cortex-M7 |

IDCODE 通常只能确定芯片组，不保证推断到封装和完整订货料号。容量和 UID 实际读取；UID 读不到会显示限制，不生成替代数据。未知 ID、内核不匹配或容量异常时停止连接，不尝试其他系列烧录驱动。C5、H7R/S、外部 Flash、H7 双核同时调试本轮未加入；双核 H7 当前仅连接 M7，软件系统复位可能影响整片芯片。

首次连接先建立只用于身份读取的 Cortex-M 会话，再切换到对应 OpenOCD 配置，随后短暂停核检查 Flash 并恢复原先运行状态。此切换只发生在建立连接阶段，后续调试动作不重新启动 OpenOCD。

Flash 地址范围取自驱动读出的实际扇区布局，并与芯片容量核对，包含 H7/G4 非连续 Bank 地址。BIN 可写到有效第二 Bank，但不能跨越地址空洞；HEX/ELF 使用镜像自身地址，由所选驱动执行写入。受读保护、低功耗或固件关闭调试影响的目标可能仍需硬件恢复，自动识别不会绕过保护。

本次通过离线回归检查；修复 OTP 和容量读取后，已实板验证 G474 无线自动识别（ID 0x20036469、512 KiB、UID 有效、恢复运行）。F1/F4/H7 仍需对应实板验证。离线复测命令：

```powershell
& 'D:/python/3.13/python.exe' -B -X utf8 .\AmphiLink_UI\tests\test_targets.py
```

## ESP32 升级

STM32 页面另有“仅整片擦除（无需固件）”按钮：连接目标后可直接执行，清除主 Flash 后保持内核暂停，不写入程序。下面的“擦除范围”下拉框仍用于选择烧录新固件前的擦除方式。

连接性能：已去掉 `flash info` 前的重复 `flash probe`，并等待 OpenOCD 报告 Tcl 端口就绪后再连接，减少无效 socket 等待。本轮同一 G474 无线连接单次样本为修改前约 4.7 秒、修改后约 3.5 秒；仍有两次初始化，不能保证所有网络环境相同耗时。

STM32 烧录支持“仅覆盖区域”和“整片擦除”。整片模式先解析镜像并核对全部主 Flash Bank，再逐 Bank 擦除所有扇区，然后写入、按所选设置校验和运行；会清除主 Flash 中原程序及保存的数据。OTP、选项字节、外部 Flash 不在此范围，也不会自动解除写保护。此选项属于烧录流程，仍需先选择固件；本轮仅离线验证，没有对实板执行整片擦除。

如果页面显示“未选择文件”，升级按钮禁用是正常的：先选择 `.amphipkg` 升级包，签名校验通过后才能开始。项目 ZIP 不能用于固件升级；校验失败会在页面显示原因。

两类烧录均有进度条，每 0.5 秒从独立接口更新，不受烧录操作锁阻塞：

- ESP32 USB：实时读取 esptool 写入百分比；切换启动、等待新版本确认时显示进行中的阶段。
- ESP32 Wi-Fi：显示实际发送字节比例；上传 100% 不等于升级成功，随后等待设备完成写入、校验及启动确认。
- STM32：显示准备、擦写、校验和运行阶段。当前 OpenOCD 驱动没有可靠的字节百分比，耗时阶段使用动态进度条；不模拟百分比。

百分比标注“本阶段”，只有整个操作确认完成才标注“完成”。失败显示“未完成”和原因。本次进度功能经过离线回归验证，没有为测试进度条擦写实板。更新电脑端代码后需重启本地服务；仅刷新网页不会替换正在运行的 Python 后端。

先结束 STM32 调试会话，在上位机 ESP32 升级区域选择：

`AmphiLink_flash/rlyoulink-release/rlyoulink-xiaos3-d2-3.0.0.amphipkg`

校验包后选择 USB 或 Wi-Fi 安装；Wi-Fi 使用设备实际 IP 和管理密码。升级期间保持供电，等待重启及新应用确认。该包更新 ESP32 和它的设备管理网页，不烧录 STM32，也不更新电脑端上位机文件。

USB/Wi-Fi 日常升级都写非运行中的 OTA 槽，再切换启动；保留 NVS，没有第三个 USB 分区。设备网页嵌入应用，随应用一起升级和回滚。`bootstrap-rlyou.py` 是特定旧版本的一次性迁移工具，不是日常升级入口，也不是任意空白芯片的全量烧录脚本。旧 `upgrade_usb.py --flash` 在本包禁用；保留查询和 `--enter`，供新升级工具调用。

当前签名包已完成电脑端验证；此前迁移写入日志显示镜像校验通过。此整理操作没有执行实板烧录，也不代表新版本启动确认、USB/Wi-Fi 全流程升级和故障回滚已全部通过测试。

## 本机依赖

本包面向当前电脑，不包含整套编译器或 Python 安装：

- `D:/python/3.13/python.exe` 和 pythonw.exe：上位机、pyusb、pyserial。
- `D:/ESP32/ESP-IDF-v5.3.2`。
- `D:/ESP32/.espressif/python_env/idf5.3_py3.13_env/Scripts/python.exe`：cryptography、esptool。
- `D:/ESP32/.espressif/tools/xtensa-esp-elf/esp-13.2.0_20240530/xtensa-esp-elf/bin`。
- `D:/CMAKE/bin`、`D:/Ninja`、`D:/Git/cmd`。

OpenOCD 及 libusb 已随包提供。换电脑时先检查脚本中的这些依赖路径和 USB 驱动。启动失败查看运行时生成的 `AmphiLink_UI/server.log`。

## 编译和发布

在根目录用 PowerShell 执行：

```powershell
& 'D:/python/3.13/python.exe' .\AmphiLink_flash\build-v2.py
& 'D:/ESP32/.espressif/python_env/idf5.3_py3.13_env/Scripts/python.exe' .\AmphiLink_flash\prepare-rlyou.py
```

编译采用 CMake + Ninja + GCC；产物和日志在双MCU工程根目录 `build/esp32s3/`。版本只修改固件 CMake 的 PROJECT_VER；prepare与打包自动读取构建元数据，签名工具默认读取镜像版本。日常构建不复制源码到TEMP。

## 签名密钥

本包包含当前私钥 `AmphiLink_flash/.release-keys/rlyoulink-private.pem`，用于后续发布；公钥已在固件源码中。私钥无文件加密密码，需要单独备份。不要将整个开发包直接公开分享；给普通使用者只发送签名升级包和所需上位机文件。

打包使用现有私钥，不要每个版本重新生成。私钥缺失时工具会生成新私钥，但如果已有公钥不匹配会停止；新密钥不能签发旧设备认可的升级包。

## 使用技能

项目技能位于 `.agents/skills/rlyoulink/SKILL.md`，仅随本项目保存，不安装到全局技能目录。在 Codex 中打开本项目后使用；也可以明确要求读取该文件。

历史过程记录、日志、缓存、截图、Flash 备份和旧试验固件未收入本包。原工作目录保留。依赖源代码、许可证及 OpenOCD 源码压缩包属于维护资料，予以保留。
