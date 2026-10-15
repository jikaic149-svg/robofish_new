#pragma once
#include <stdint.h>

// sampledAtMs是采样时刻，接收边界须换算为F405本地单调毫秒时钟。
// 生产者保持参考系与机体系配对的FLU（前/左/上）右手坐标、ZYX欧拉角契约；
// 任意ENU参考系姿态须由生产者先转换，不能只凭“机体FLU”直接接入。角度rad，
// 角速度rad/s是FLU机体系分量。正yaw向左；正pitch低头。
// 给算法的FRD快照在control/l1/Feedback.h显式转换：roll不变，pitch/yaw取负，
// 对应机体系q/r角速度取负。这里不选择传感器、引脚或实现闭环反馈。
struct AttitudeSample {
  uint32_t sequence = 0;
  uint32_t sampledAtMs = 0;
  float rollRad = 0;
  float pitchRad = 0;
  float yawRad = 0;
  float rollRateRadPerSec = 0;
  float pitchRateRadPerSec = 0;
  float yawRateRadPerSec = 0;
};
