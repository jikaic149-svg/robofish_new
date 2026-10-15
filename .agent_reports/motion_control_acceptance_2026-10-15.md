# 运动控制分层实现与验收

这是用户要求的本轮唯一验收报告。范围为 STM32 运动控制 L0～L3、后续传感器与算法入口、控制依赖检查、论文复核及全仓文件清理。

**当前结论：统一控制链和 L2 三个机构节点已经实现，主机回归、Debug／Release 构建及隔离 PC 回归通过。** 已证明的是命令生成、范围约束、输出接续和接口行为；本机推力、鳍面水动力、运动精度与能耗仍需台架和水池测量。未执行烧录、实机运动或实际传感器采集。

最终收尾状态见第 8 节；最终构建后的 CTest 复跑及 PC 下次启动重建状态按实际结果补入，未完成项目不记为通过。

## 1. 交付边界与职责

### 1.1 运行链

    来源输入、控制权与既有安全编排
        ↓
    L0：来源语义 → AdaptedIntent
        ↓
    L1：BodyCommand 四轴请求、输入有效性
        ↓
    MotionController 唯一周期 owner
        ↓
    L2：Allocator → tail / propulsion / pectoral
        ↓
    六路 candidateFrame
        ↓
    L3：RoboFishDriver → 安装映射、电压补偿、设备写入
        ↓
    writtenMask → 每通道成功的名义历史 → 下一控制周期

层次是职责划分，未为每层建立新任务。既有运动输出周期仍为 20 ms。MotionController 管理来源选择、控制代次和状态；输出任务推进唯一公共相位、调用分配器并提交六路输出。

| 归属 | 源码位置 | 拥有的职责 |
|---|---|---|
| L0 来源适配 | [control/l0/IntentAdapter.cpp](../stm32f405/src/control/l0/IntentAdapter.cpp) | LoRa、人工及既有视觉独立尾／桨请求转换；来源反向偏好与档位解释 |
| L0 视觉策略 | [control/l0/VisionMotionPolicy.cpp](../stm32f405/src/control/l0/VisionMotionPolicy.cpp) | 保留既有视觉推进、转向与制动策略，输出动作需求 |
| L1 命令契约 | [control/l1/Command.h](../stm32f405/src/control/l1/Command.h) | 四轴百分比的单位、正方向、有限值与范围检查 |
| L1 观测适配 | [control/l1/Feedback.h](../stm32f405/src/control/l1/Feedback.h) | 样本次序、时间、有限值、参考系转换与新鲜度快照 |
| L2 总分配器 | [control/l2/Allocator.cpp](../stm32f405/src/control/l2/Allocator.cpp) | 模式能力、造流许可、共享相位、三个子节点调用与帧合成 |
| L2 尾部 | [control/l2/tail/Tail.cpp](../stm32f405/src/control/l2/tail/Tail.cpp) | 两关节波形、偏置、幅值余量与尾部轨迹限制 |
| L2 双桨 | [control/l2/propulsion/Propulsion.cpp](../stm32f405/src/control/l2/propulsion/Propulsion.cpp) | 共同／差动命令、饱和策略与两路命令过渡 |
| L2 胸鳍 | [control/l2/pectoral/Pectoral.cpp](../stm32f405/src/control/l2/pectoral/Pectoral.cpp) | 俯仰基础目标、横滚及可选修正、成对行程限制 |
| 配方与几何 | [config/MotionAllocationConfig.h](../stm32f405/src/config/MotionAllocationConfig.h) | 所有来源共用的机构配方、几何与模式参数 |
| 产品参数 | [config/ControlConfig.h](../stm32f405/src/config/ControlConfig.h) | 周期、限速、权重和既有保护参数；未标定修正默认零 |
| 控制编排 | [task/MotionController.cpp](../stm32f405/src/task/MotionController.cpp) | 权限、安全退出、唯一周期调用与逐通道提交历史 |
| 算法输入 | [task/MotionCommandInput.cpp](../stm32f405/src/task/MotionCommandInput.cpp) | 授权算法请求接入，与人工、视觉、LoRa 共用输出链 |
| 传感器输入 | [task/MotionFeedback.cpp](../stm32f405/src/task/MotionFeedback.cpp) | 接收与保存最新样本，提供一致观测快照和显式复位 |
| L3 执行边界 | [device/motion/RoboFishDriver.cpp](../stm32f405/src/device/motion/RoboFishDriver.cpp) | 角度边界、电气方向、单次补偿、六路写入与结果 |
| L3 器件 | device/motion/ 下 TailServos、PectoralFins、Propulsion | 器件级命令与有限时间设备访问 |
| BSP | stm32f405/stm32f405/bsp/ | 定时器、引脚、PWM 和板级硬件 |

L3 放在现有 Device 目录中，直接承担执行职责；没有另建仅转发一次调用的空壳层。

### 1.2 四轴与六路的含义

L1 BodyCommand 的 surge、yaw、pitch、roll 均为 −100～100 的归一化请求，另有 enabled。正向分别表示前进、右转、抬头、右侧下滚。它们尚不代表 m/s、deg/s、目标姿态角或牛顿／牛米。

六路输出为尾前关节、尾后关节、左胸鳍、右胸鳍的名义角度，以及左右桨各自的名义百分比。两桨在 L2 中统一采用“正值向前”；右桨的安装电气反号只在 L3 执行边界处理。

当前鳍角沿用已配置的舵机命令角坐标。90°中立与各侧端点是现有配置基线，不是本轮新测得的机构零位；关节角、有效迎角和鱼体姿态角仍需分别标定。

## 2. L2 的树与调用规则

### 2.1 实际源码树

    control/l2/
    ├── Types.h                       请求、几何、配方、状态、结果
    ├── Allocator.h
    ├── Allocator.cpp                  唯一总入口 allocate()
    ├── tail/
    │   ├── Tail.h
    │   └── Tail.cpp                   tail::allocate()
    ├── propulsion/
    │   ├── Propulsion.h
    │   └── Propulsion.cpp             propulsion::allocate()
    └── pectoral/
        ├── Pectoral.h
        └── Pectoral.cpp               pectoral::allocate()

Hybrid 直接从这三个子节点取得结果，纯桨和纯尾也经过同一棵树。模式决定哪些输入生效，不复制另一套机构算法。

父节点先取得尾部结果和双桨结果，再把需要的尾部负载比例及共同推进比例作为普通数值传给胸鳍节点。胸鳍不查询尾部或推进器状态，也不调用它们。

    Allocator
    ├── 取 tail 的两关节目标、候选值、load、限制位
    ├── 取 propulsion 的左右目标、候选值、共同／差动、限制位
    └── 传入上述必要数值，取 pectoral 的两鳍目标、候选值、scale、限制位
        ↓
    父节点合成 targetFrame 与 candidateFrame

### 2.2 依赖检查

[control_architecture_test.py](../tests/control_architecture_test.py) 检查以下实际边界：

- L0、L1、L2 不直接依赖 Task、Device、BSP、网络、服务或 FreeRTOS。
- L0 通过 L1 表达请求，不包含 L2 或 L3 的入口。
- L1 不依赖 L0、L2、L3。
- L2 不向上依赖策略，不向下依赖执行设备。
- tail、propulsion、pectoral 三个叶节点不包含或调用同级子树。
- 只有 Allocator.cpp 可以包含三个叶节点；Task、Device 等外部目录不得直接访问叶节点。

纯数学工具 GaitPhase、PairedLimit 以及共享数据类型可以复用。Task 作为 owner 串起控制链，不构成叶节点跨区访问。

## 3. L2 每周期具体执行

### 3.1 输入、状态与输出

| 项目 | 内容 |
|---|---|
| Request | 四轴请求、模式、使能、输出比例、造流许可、立即输出标记 |
| 独立驱动选项 | 可保留视觉等来源明确给出的 tailDrivePercent、propulsionDrivePercent；不会被压成同一个推进值 |
| 独立尾偏航 | tailYawPercent 只影响尾部偏置，保留 tailReversed 的原有操控语义 |
| Recipe | 三机构增益、幅值、频率、相位差、限制与可选修正 |
| Geometry | 四关节各自的中心、下限、上限 |
| lastCommitted | 各通道上次成功提交的名义请求，由输出 owner 提供 |
| State | 一个公共 GaitPhase；不保存设备状态或来源状态 |
| 时间 | 当前本地毫秒时间和本次实际间隔 |
| Result | 限速前 targetFrame、可提交 candidateFrame、相位、共同／差动、尾部负载比例、胸鳍修正比例及限制位 |

几何、请求、配方和成功历史先经过有限值及范围检查。计算结束再检查六路目标，避免有限系数相加溢出后的非有限值流向设备。

### 3.2 能力与方向处理

| 条件 | 现用行为 |
|---|---|
| Bionic 正推进 | 尾部周期摆动；双桨目标为零 |
| Bionic 零 surge、有 yaw，且没有独立尾驱动 | 可以由偏航强度驱动尾摆 |
| Prop | 双桨推进／差动；尾部使用静态舵偏置 |
| Hybrid 正推进 | 尾摆与双桨同时参与，胸鳍按姿态请求参与 |
| Bionic 负推进 | 返回 ReverseUnsupported；输出任务清空旧目标并推进控制代次，进入已有停止流程 |
| Prop／Hybrid 反推 | 双桨承担反推；尾部停止周期／静态转向作用，按正常限制回配置中立；造流辅助关闭 |
| 停止／禁用 | 不允许旧尾／桨请求、尾部偏航或造流辅助留下活动目标 |

判定方向只使用当前模式实际会执行且已使能的驱动字段。Prop 忽略的负尾驱动，或者 Bionic 不使用的负桨字段，不会错误改变当前执行方向。

完整安全停止仍由 Task 选择。Surface 继续使用既有的停桨、保留进入时鳍姿态路径；全停／电池退出等不改成普通周期摆动的一种模式。

### 3.3 造流辅助

只有 Request.allowFlowAssist 明确允许，且请求不是反向／禁用时，才可根据 pitch 提高有效推进。当前最小值沿用现有配置：

    minimumDrive = abs(pitchPercent) × flowMinimumPercent / 100

这属于开环动作配方，不能证明胸鳍已经获得足够来流。普通算法提交纯 pitch 请求不会自动获得造流许可。额外阻力补偿增益当前为零；未实现自动识别流场或根据虚构水速补偿。

### 3.4 公共相位

父节点每周期只调用一次已有 GaitPhase：

    φ_next = remainder(φ_previous + 2π × f × 实际经过秒数, 2π)

前后关节共用 φ，不分别读取系统时间建立振荡器。首次相位沿用现有公共时间原点初始化，并未规定启动必从零相位开始。停机时由输出 owner 按既有规则复位分配状态。

相位使用实际时间间隔；限速时间取实际间隔与 maximumSlewElapsedMs 的较小值，当前上限 20 ms。因此丢掉一拍时，相位会追随真实时间，执行器仍受本拍允许变化量限制。

这保证相位推进规则明确，不保证任何频率、幅值、偏置切换都物理平滑；输出仍须经过轨迹限制。

### 3.5 尾部子节点

设各关节原中立为 n，偏置后中心为 b：

    bias = tailYawPercent × 对应偏置增益 / 100
    b_front = clamp(n_front + bias, front_min, front_max)
    b_rear  = clamp(n_rear  + bias, rear_min,  rear_max)

周期幅值先满足两个关节的共同可用行程：

    h_front = min(b_front − front_min, front_max − b_front)
    h_rear  = min(b_rear − rear_min, rear_max − b_rear)
    A_max   = min(h_front, h_rear / abs(rearAmplitudeRatio))
    A       = min(请求半幅, max(0, A_max))

后关节比例为零时不使用第二项。生成目标：

    front_target = b_front + A × sinφ
    rear_target  = b_rear  + A × rearAmplitudeRatio × sin(φ − phaseLag)

随后分别从成功名义历史限速。没有通过同时给两项正弦乘 −1 实现“尾鳍后退”的分支。

当前统一配方的标称前关节半幅为 25°、后／前幅值比 0.9、频率 1.5 Hz；Bionic／Hybrid 的相位差分别为 60°／72°。尾部偏航几何增益沿用当前配置方向，周期偏置满量程为 −18°、Prop 静态偏置为 −40°；这些响应方向与有效角带仍须实测。

相位差描述关节之间的关系。快划慢回、间歇占空比、动态最优相位等尚未加入当前执行器波形，不能把它们写为本轮已实现能力。

### 3.6 双桨子节点

当前明确保留“共同推进优先”：

    c = clamp(有效推进请求 × commandScale, −100, 100)
    margin = 100 − abs(c)
    d = clamp(yawPercent × yawGain, −margin, margin)
    left_target  = int((c + d) × outputScale)
    right_target = int((c − d) × outputScale)

yawGain 为 +1。按照前／右／下的右手坐标，左桨位于负 y，左名义推力大于右名义推力对应正偏航力矩。电机电气反号不再混入这一几何关系。

输出比例仍在静态分配后作用，保持既有档位位置。因此共同请求已经占满时，随后整体降低档位不会自动恢复差动余量。

| c、请求 d；比例 1 | 左／右目标 | 含义 |
|---|---:|---|
| 60、20 | 80／40 | 范围内分配 |
| 80、40 | 100／60 | d 限为 20，并报告 PropulsionLimited |
| 100、40 | 100／100 | 当前策略全部让出差动，不宣称已满足转向 |
| 0、40 | 40／−40 | 命令对称；真实净推力是否为零须测量 |

之后两桨各自限速。当前每秒允许变化 500 个百分点，20 ms 可变化 10 个百分点。命令分辨率为整数 1%；允许变化量向下取整，避免趋零截断时超过限速。小于 1% 的本拍预算会保持原值，不积累小数信用；20 ms 下低于 50 个百分点／秒的配方不能形成连续小步进。

逐桨独立限速不保证起步时共同／差动比例不变。例如目标 80／40，从 0／0 启动的第一拍是 10／10。该行为有测试和诊断，不宣称联合过渡最优。

### 3.7 胸鳍子节点

先用 pitch 生成各侧基础目标，再叠加 roll 与明确配置的前馈：

    base_L = center_L + pitch映射_L
    base_R = center_R + pitch映射_R
    delta_L = roll映射_L + yaw修正_L + 共同推进修正_L + 尾部负载修正_L
    delta_R = roll映射_R + yaw修正_R + 共同推进修正_R + 尾部负载修正_R

基础目标先限制到各侧行程内，再求最大的公共比例 λ，使两侧同时合法：

    0 ≤ λ ≤ 1
    min_L ≤ base_L + λ × delta_L ≤ max_L
    min_R ≤ base_R + λ × delta_R ≤ max_R

这明确采用“俯仰基础目标优先，修正共用剩余行程”。例如基础均为 +15°、修正为左 +10°／右 −10°、两侧限位 ±20°，λ=0.5，结果为 +20°／+10°。

实际默认 pitch 端点仍按现有取整得到左 132°／56°、右 48°／124°。roll、yaw 耦合、推进配平及尾部配平修正默认均为零；没有以猜测的安装符号开放实机 roll。

父节点传给胸鳍的共同推进比例来自限速前双桨目标，tailLoad 来自受行程约束后的尾部幅值比例。这些是命令配方量，不是测得推力或局部水流。

### 3.8 六路实例：真实默认配方

设 Hybrid，surge=60、yaw=20、pitch=20、roll=0，比例为 1，造流辅助关闭；本拍公共相位为 90°，四鳍成功历史为 90°、两桨为 0。

尾部请求半幅 15°，尾偏置 −3.6°，后关节相位为 18°。目标与候选如下：

| 通道 | 限速前目标 | 本拍候选 |
|---|---:|---:|
| 尾前 | 90 − 3.6 + 15 = 101.4° | 95° |
| 尾后 | 90 − 3.6 + 13.5 sin18° ≈ 90.5717° | 90.5717° |
| 左胸鳍 | 98.4° | 95° |
| 右胸鳍 | 81.6° | 85° |
| 左名义桨 | 80% | 10% |
| 右名义桨 | 40% | 10% |

四鳍限速为 250°/s，即本拍最多 5°；桨最多 10 个百分点。L3 随后执行舵机整数角量化、安装方向和电压补偿。表中的角度是名义请求，不是位置传感器读数。

### 3.9 提交接续与诊断

L2 不修改成功历史。L3 返回 writtenMask 后，由唯一输出 owner 使用 mergeWrittenFrame 更新成功通道。

例如第一拍候选双桨为 10／10，只有左桨成功：下一拍左桨从 10 开始，右桨从之前的 0 开始；其他关节按各自成功结果接续。公共相位按时间前进，不回滚到失败前的相位。

FrameApplyResult 中的桨百分比是安装／电压补偿后的电气命令，不能回灌为下一周期的名义目标。诊断保留尝试帧、成功帧、逐通道掩码、相位、共同／差动和以下限制原因：

| 限制位 | 数值 | 含义 |
|---|---:|---|
| PropulsionLimited | 1 | 双桨命令范围／差动余量限制 |
| TailLimited | 2 | 尾偏置或周期幅值受几何限制 |
| PectoralLimited | 4 | 胸鳍基础目标／成对修正受行程限制 |
| ReverseUnsupported | 8 | 不支持的纯尾反向请求 |
| InvalidInput | 16 | 请求、几何、参数、历史或计算结果不合法 |
| TailRateLimited | 32 | 尾部本周期未到目标 |
| PropulsionRateLimited | 64 | 双桨本周期未到目标 |
| PectoralRateLimited | 128 | 胸鳍本周期未到目标 |

## 4. L3 与来源统一中的行为变化

L3 通过同一六路帧写入 TailServos、PectoralFins、Propulsion。运行期访问使用既有有限时间／非阻塞限制；六路写入不是原子操作。

安装方向和参考 7.4 V 的前馈补偿在电气执行边界处理一次。每拍由当前名义请求重新计算，不对上次电气结果反复补偿。边界前先夹紧桨请求，避免异常整数在电气反号时溢出。

写入成功表示驱动接受请求，不证明鳍已经到位，也不证明桨已达到目标转速／推力。

| 有意变化 | 影响与验收重点 |
|---|---|
| 所有来源统一硬件配方 | 人工、LoRa、视觉使用同一几何、1.5 Hz 尾摆、相位与限速；保留来源明确给出的独立尾／桨强度 |
| 旧 SDK 86°尾中位及虚拟油门配方退出执行链 | 不再由来源选择另一套物理步态；兼容通信中的 pitch／显示字段须与执行器配方区分 |
| 双桨 yawGain=+1 | 在统一 FRD 名义坐标修正共同／差动方向；实机电气极性与运动响应仍须核对 |
| 删除“共同正弦取反即倒游” | 纯尾负请求被消费、撤销旧目标，切换模式不会使旧前进复活；混合后退由桨反推承担 |
| tailReversed 仅保留尾偏航偏好 | 不反相周期推进波形，不翻转所有机构 |
| roll 默认零增益 | 四轴接口与共享行程算法具备；尚未标定为实机横滚能力 |
| 桨整数限速向下取整 | 避免靠向零截断导致超出本拍预算，明确 1% 分辨率下限 |

旧 mode 的 ActuatorMapping、ActuatorTransition、FusedMotion、MotionMode、SdkMotionAdapter 五对源／头文件已备份到 build/control-implementation/retired-mode 后移除。RuntimeConfig 中无人消费的旧 86°尾中位、1.7／2 Hz 配方、虚拟油门／舵角和死区宏也已删除；86°仅作为旧 pitch 输入的兼容表示保留。旧执行链不再与 control 树并存。

## 5. 传感器与后续算法接入

### 5.1 实际公共入口

[MotionController.h](../stm32f405/src/task/MotionController.h) 已提供：

    submitAttitude(const AttitudeSample &sample)
    clearAttitude()
    submitDepth(const DepthSample &sample)
    submitVelocity(const VelocitySample &sample)
    getFeedback(ControlFeedback &snapshot, uint32_t maximumAgeMs)
    clearFeedback()
    submitBodyCommand(DriveMode mode, const BodyCommand &command, uint32_t generation)

这些入口已接到实际 owner 保存状态及控制输出，不是只有声明的预留接口。接口会因未启动、共享锁忙或输入不合法返回 false，生产者不能把拒绝当接收成功。

### 5.2 样本数据契约

| 样本 | 数据与约定 |
|---|---|
| AttitudeSample | 三个 ZYX 欧拉角 rad、三轴机体系角速度 rad/s、sequence、sampledAtMs |
| DepthSample | depthMeters，下为正；深度零基准由生产者声明 |
| VelocitySample | surgeMetersPerSecond，机体前向为正；生产者与消费者必须约定相对水速或对地速度 |

采样时间须换算成 F405 本地单调毫秒时钟，不能直接把远端开机时间当同一时钟。序号属于各自样本流，允许首样本为零及 uint32 回绕，拒绝重复／倒退序号、倒退时间、未来时间与非有限值。比较使用半范围次序；有效间隔及新鲜度预算应小于 2³¹ ms。

生产者重启不得靠复用旧序号自动绕过检查。clearAttitude 清除姿态历史；clearFeedback 显式清除三个观测历史，均不授权运动。

### 5.3 参考系与有效性

已有姿态输入保持“参考系与机体系配对的 FLU”契约。L1 快照显式转成配对 FRD：roll 保持，pitch／yaw 和机体系 q／r 反号。任意 ENU 世界参考的 IMU 姿态必须先做完整参考变换，不能只因为机体采用 FLU 就直接套反号。机体系角速度也不等于欧拉角导数。

getFeedback 成功仅说明取到了快照；使用者仍必须分别检查 attitudeAvailable、depthAvailable、velocityAvailable 及年龄。过期值可供诊断，但不伪装成有效观测；未接入传感器时，相应标志为 false。

getDiagnostics 中已有 attitudeAvailable 表示历史存在，不是闭环新鲜度判据。算法应使用 getFeedback 的有效性结果。

### 5.4 后续算法怎样接

    传感器生产者 → 样本入口 → 一致观测快照
                                  ↓
                         算法计算 BodyCommand
                                  ↓
                   submitBodyCommand(mode, command, generation)
                                  ↓
                          同一 L2 与 L3

算法请求要求当前有效的本地授权会话、正确 generation、Running 状态、正常电池条件，并且不能越过 LoRa 控制权或输出恢复限制。新的人工命令、停止及安全退出会使旧异步结果失效。enabled=false 也须通过该入口的权限检查。

命令中的非有限值或越界请求被拒绝，不替换此前已接受目标。纯尾反向在执行链被明确识别并清空旧目标，是另一项有意的行为修复。

本轮没有声称已接入 IMU、压力、流速传感器生产者，没有新增姿态／深度／航速闭环。已有视觉策略和 PID 消费者保留。之后新增算法可消费上述观测并输出四轴请求，不需要访问叶节点或 PWM。

## 6. 文献复核：可行、待验证、错误

主机验收之后再次在线核对原始研究、作者资料与大学讲义，并将源码实现和物理能力分别判断。论文没有直接验证本机“双串联尾关节＋侧置双桨＋双胸鳍”的全部参数。

### 6.1 数学与软件可验收的部分

| 内容 | 结论与证据边界 |
|---|---|
| 请求、分配、执行分工 | 结构合理；[Johansen 与 Fossen，§1.1～1.2、式 3～8，PDF 第 2～3 页](https://www.fossen.biz/publications/2013%20Johansen%20and%20Fossen%20Automatica.pdf)讨论类似分层及范围、速率约束 |
| 共同／差动与成对行程 | 当前公式可逐周期代数检查；实现百分比命令分配，不声称已识别物理控制效能矩阵 |
| 相位差双关节周期目标 | [Liang 等，§3.2.1、式 3～4，PDF 第 4 页](https://softrobotics.buaa.edu.cn/2011-Liangjianhong-JFR.pdf)采用相位相关正弦；其并联四杆与本机串联舵机不同，参数不直接搬用 |
| 公共相位、限幅、逐通道接续 | 由实际回归证明软件规则；不需要把软件通过包装成水池推进效率证明 |
| 双桨转向正方向 | [Fossen 的力／力矩与坐标定义](https://www.fossen.biz/html/marineCraftModel.html)支持按安装位置叉乘核对；具体推力曲线仍需测量 |

上述分配综述 §2.2 也说明，简单逐通道饱和不保证最优分配。本轮共同优先、独立限速是明确可回放的策略，不使用“最优混合”名称。

### 6.2 有研究依据、仍需本机标定

| 项目 | 当前处理与需要的数据 |
|---|---|
| 尾幅值、频率与相位差产生多少推力 | 保留可配置周期轨迹；需要真实关节轨迹、带载能力、速度和功耗 |
| 尾偏置转向 | 保留现有增益；核对偏置符号、前后关节协同、转向率和速度损失 |
| 胸鳍俯仰／横滚 | [Snapp §III-A 式 9～10](https://arxiv.org/html/2308.13163v1#S3.SS1)支持来流下的两鳍控制及速度平方依赖；不证明本机零速权限，roll 默认关闭 |
| 左右／正反桨推力 | 需要本机各桨、电压及来流下曲线；[厂家 T200 数据](https://bluerobotics.com/store/thrusters/t100-t200-thrusters/t200-thruster-r2-rp/)只能说明其他型号存在正反差异 |
| 下潜与定深 | 需要净浮力、力臂、姿态／深度观测和持续竖向作用；本轮未增加定深闭环 |
| 全程 Hybrid 的收益 | 保留尾／桨持续共同参与的基线；没有固定“低速尾、高速桨”切换表，需要同速度／同任务比较 |
| 造流辅助和配平 | 旧辅助需要试验确认来流覆盖；未标定额外修正保持零 |

### 6.3 用户两篇 PDF 的准确适用范围

**Bai 等，Numerical investigation and swimming performance evaluation on self-propelled hydrodynamics of hybrid-driven biomimetic robotic fish。** [发行方／DOI](https://doi.org/10.1016/j.joes.2025.10.002)。已按用户提供全文核对：

- §2.1～2.2、PDF 第 2～3 页：原型长 1.493 m、重 48.567 kg、三关节尾部、两个水平桨，另有竖向桨；本文 CFD 主要研究简化平面直游。
- §3.2：其 energy utilization factor 使用整机动能、多项动能与流场量构造，不能当作电池输入效率或与实验 CoT 互换。
- 原型验证受舵机能力限制，尾频最高 1.5 Hz；较高频数值结果不能视为本机舵机已可跟随。
- §5 中部分混合工况比同转速纯桨更慢，说明机构间作用需要实测；不支持“全部混合输入一定叠加获益”。

**Liangyu Li 等，Enhanced Speed and Maneuverability of a Robotic Fish via Fin-Propeller Hybrid Propulsion。** [IEEE／DOI](https://doi.org/10.1109/ACIRS66343.2025.11360966)。网络全文访问有限，构型及数据以用户提供全文核对：

- §II、PDF 第 2～3 页：背鳍、臀鳍及尾部单桨的 MPF 构型，质量 3.35 kg，并非本机两关节摆尾。
- §III 的 CoT 为 P/(mgv)。纯鳍最大速度 0.373 m/s 对应 5 Hz、±40°；最低 CoT 0.968 对应 3 Hz、±20°，最快与最低成本工况不同。
- 混合最大速度 0.434 m/s 对应 5 Hz、±40°、80% 桨占空比；功率／CoT 图使用 ±30°，不能把这些图拼成完全相同工况。
- §IV 的鳍偏置与转向近似分工仅支持其原型和试验范围，不直接决定本机尾偏置、差动权重或横滚控制。

两篇研究都支持研究混合推进，没有给出本机的最优权重或统一切换速度。

### 6.4 不成立或证据不足的逻辑如何处理

| 判断 | 本轮处理 |
|---|---|
| 两个关节正弦同时乘 −1 就能后退 | 现用实现删除；只是公共相位平移 π，相对相位不变 |
| 反向关节相位差必然产生倒游 | 未实现为后退能力，保留为需要独立实验的研究问题 |
| 横滚产生轴向推力的 T sinφ 侧向分量 | 未进入控制算法；[Fossen 旋转矩阵](https://www.fossen.biz/html/marineCraftModel.html)第一列与滚角无关，不能据此承诺滚转转弯 |
| 通用 St 窗口决定本机尾部启停 | 未加入；[Eloy 的模型](https://arxiv.org/abs/1102.0223)讨论尺度相关最优值，[Brouzet §2.2](https://arxiv.org/html/2501.09671v1#S2.SS2)的 0.48 为特定整机推阻平衡 |
| J 越大效率越高 | 未加入；[MIT §12.2.1，PDF 第 58～59 页](https://ocw.mit.edu/courses/2-154-maneuvering-and-control-of-surface-and-underwater-vehicles-13-49-fall-2004/2609f8b1bdebe4566891080afe007f51_1349_notes.pdf)的 K_T、K_Q 随 J 变化 |
| 固定反推比例保证原地转 | 未加入；保持真实推力未标定状态 |
| 固定“先姿态后推进”动作顺序 | 未强制加入；来流和姿态可能需要共同建立 |
| 快划慢回必降 St | 未加入；f、尾尖峰峰位移和 U 不变时 St 不变 |
| 电流／RPM 等于相对水速，PWM 提交等于机构到位 | 接口和诊断明确禁止这种等同，不以假观测闭环 |

只有已经证明错误的映射被删除；带有合理物理依据但缺少本机数据的配方保持可调或默认关闭，没有因为“尚未测量”就伪称不可行。

## 7. 全仓 tidy 与文件治理

已使用 tidy-skill 审计整个 robofish_new，再对确认来源的派生物执行清理。每个删除目标均先解析绝对路径、确认位于工作区，并拒绝重解析点；使用 PowerShell LiteralPath 完成删除。

**派生物清理 29 个目标、56 个文件，合计 6,941,870 bytes。** 此数量不包含第 4 节另行备份后删除的 10 个旧控制源／头文件。

| 类别 | 实际清理 |
|---|---|
| 语言缓存 | tools/emberprobe、AmphiLink_UI、其 tests、AmphiLink_flash 下四处 __pycache__ |
| 旧测试产物 | build/_hostverify 的 7 个旧主机可执行文件 |
| 一次性改写／截图脚本 | build 根下七个旧脚本；workstation 的 refine_feedback_ui、refine_vision_views、vision-feedback |
| 重复测试副本 | workstation/engine-check；其恢复源与正式恢复副本哈希相同 |
| 重复图片与展开 HTML | 两张与 docs 完全一致的图片、native-sidebar.html |
| 过期插件包 | bridge 0.1.0、0.2.0；保留当前 0.2.1 |
| 论文阅读派生物 | 四份文本、两张 PDF 页面截图；用户原 PDF 不动 |
| 空占位文件 | 根 app.js、robot.js，实际网页使用 AmphiLink_UI 内文件 |

保留正式文档、用户胸鳍报告、根 validation.json、模型、标定、发行包及当前构建。保留用户遥测、正在运行的工作站启动器、工具状态、私人日志和密钥；未读取其内容。原稿及代码修改备份保留；未知 ACL 测试临时目录未强行清理。

根 .gitignore 已补充代理／编辑器状态、秘密与 provisioning 输入、嵌套构建、Python 缓存、本地环境及日志规则。原文件备份在 build/control-implementation/gitignore.before-tidy。

16 组真实 git check-ignore --no-index 检查通过：新控制源码、正式 docs、根胸鳍报告、validation.json、本验收报告、公开验签 PEM、STM32 launch.json.in 都不会被忽略。

本报告放在用户明确要求的 .agent_reports/；未新增通用 plan、summary、todo 或第二份验收报告。

## 8. 验收结果与复现

### 8.1 当前已取得的结果

| 检查 | 状态 | 实际覆盖／限制 |
|---|---|---|
| 主机 C/C++ 回归 | 通过：15 组 | 最终源码使用严格告警编译；实际控制计算和控制器，RTOS／硬件边界使用主机替身 |
| 依赖边界检查 | 通过：15 个控制文件 | L0／L1／L2 方向、L2 父取三个子节点、外部不得直达叶节点 |
| 网页语法与回归 | 通过 | 既有 robot.js 语法及网页行为检查 |
| Debug 构建 | 通过：应用 A／B、Boot、Factory | 编译／链接／生成，不代表烧录或实物运行 |
| Debug 应用资源 | Flash 96,412 B；RAM 20,456 B；Retain 64 B | 最终静态链接统计；Factory 文件 620,700 B；不代表栈水位或实际调度测量 |
| Debug CTest | 最终构建后 3／3 通过 | A 镜像、B 镜像、Boot 固件静态校验；`build/control-implementation/debug-ctest.log` |
| Release 构建 | 通过：应用 A／B、Boot、Factory | Flash 56,204 B；RAM 20,456 B；Retain 64 B；Factory 文件 580,492 B |
| Release CTest | 最终构建后 3／3 通过 | `build/control-implementation/release-ctest.log` |
| PC 模型 DLL／仿真回归 | 隔离运行通过：84 项 | 含源码过期重建检查；新 DLL 由现用 L0／L2 源码构建；当前 8765 服务加载的旧 DLL 保持原状，用户运行数据未被重启清空 |
| 旧 Mode／无用配方清理 | 通过 | 五对旧源／头备份后删除，旧尾中位及虚拟驱动配方宏清除 |
| 全仓 tidy | 通过 | 数量和保留边界见第 7 节 |
| Git 交付 | 全工程本地快照；GitHub 推送待仓库地址 | 根仓库保存普通文件，不将 esp32s3 当作 gitlink；旧嵌套仓库保留，密钥／工具状态／缓存不纳入提交 |
| 实机／水池验收 | 未执行 | 无烧录、无真实运动、无传感器采集与推力测量 |

主机回归包含 motion-allocation、motion-command、motion-intent、motion-feedback、motion-pipeline、motion-output、boot-store、adc、feedforward、bsp、lora、battery-stop、safety-control、spi-state、spi-protocol。本机记录见 build/control-implementation/host-tests.log；日志和二进制均按本地派生物处理。

关键断言覆盖：

- 三机构六路合成、相位和时间回绕、尾幅／偏置范围、胸鳍共同修正比例。
- 双桨共同优先、独立驱动、整数限速、小步预算与正负切换。
- 非法值／几何／结果检查；部分通道失败后从成功名义历史接续。
- 纯尾反向撤除与模式切换不复活旧目标；停止后造流不复活。
- 来源统一、视觉独立尾／桨需求、正确电气反号与补偿不累计。
- 观测样本排序／年龄／坐标转换／复位、算法代次和来源权限。
- 电池、保护、SPI、启动与既有停止回归。

### 8.2 复现入口

在工程根目录执行主机检查：

    python -B -X utf8 tests/run_host_tests.py --compiler C:\msys64\ucrt64\bin\g++.exe
    python -B -X utf8 tests/control_architecture_test.py

STM32 构建及测试在 stm32f405/ 目录使用现有 debug／release presets；工具链需已在 PATH 或本地配置中可用：

    cmake --preset debug
    cmake --build --preset debug
    ctest --test-dir build/debug --output-on-failure
    cmake --preset release
    cmake --build --preset release
    ctest --test-dir build/release --output-on-failure

PC 模型源码通过 tools/build_pc_motion_model.py 重建。现有六输出 ABI 是无会话状态的目标预览，不能覆盖真实任务的限速、租约、失败接续或实测电池；这些由固件流水线测试负责。

当前服务占用旧 DLL，不能把隔离验证等同于该进程已经热替换。`motion_model.py` 已实现下个进程首次使用时检查源码修改时间，DLL 缺失或过期则重建再加载；加载后的进程不热替换。缺失／更新／无需重建／已加载保持的四项检查已通过。本轮没有为替换预览 DLL 强制终止用户运行服务；隔离新 DLL 保存在 `build/control-implementation/motion_model.dll`。

### 8.3 剩余物理验收

软件回归通过后，可按机构逐项测量：四鳍真实中位／方向／限位，尾部带载轨迹，左右桨正反推力及电压响应，胸鳍来流与姿态作用，最后比较全程混合时的同速功耗、转向权限和升沉表现。

当前可以接入传感器生产者和后续算法，但能否形成稳定闭环、达到某个深度或速度，必须以观测质量和上述本机数据继续验收。
