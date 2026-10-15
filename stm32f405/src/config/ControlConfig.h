#pragma once

#include <stdint.h>
#include "config/RuntimeConfig.h"

// 产品策略与通信整定值集中于此；机械标定继续以 RuntimeConfig 为唯一来源。
namespace robofish::config::control {
inline constexpr uint32_t kPeriodMs = 20;
inline constexpr unsigned kControlTaskStackBytes = 4096;
inline constexpr unsigned kVisionTaskStackBytes = 2048;
inline constexpr float kVisionMinimumDtSeconds = 0.02f;
inline constexpr float kVisionMaximumDtSeconds = 0.20f;
inline constexpr float kVisionMaximumErrorMeters = 5.0f;
inline constexpr float kVisionMaximumSpeedMps = 5.0f;
inline constexpr int kVisionBrakeNeutralPercent = 1; // Both nominal props, percent.
inline constexpr uint32_t kMaximumSlewElapsedMs = kPeriodMs;
// Nominal independent-prop command slew, percentage points/s. At 20 ms this
// permits 10 points per tick; integer outputs need at least 1 point to progress.
inline constexpr float kPropulsionRatePercentPerSecond = 500.0f;
inline constexpr float kFlowMinimumPercent = 100.0f; // 满潜浮沿用原档位造流。
inline constexpr float kHybridTailCommandScale = 1.0f; // 命令权重，不表示推力份额。
inline constexpr float kHybridPropulsionCommandScale = 1.0f;
// 单位 °，分别为满幅请求时的舵机偏移；未标定项零，禁止推测安装正负号。
inline constexpr float kPectoralLeftYawDegrees = 0.0f;
inline constexpr float kPectoralRightYawDegrees = 0.0f;
inline constexpr float kPectoralLeftSurgeTrimDegrees = 0.0f;
inline constexpr float kPectoralRightSurgeTrimDegrees = 0.0f;
inline constexpr float kPectoralLeftTailTrimDegrees = 0.0f;
inline constexpr float kPectoralRightTailTrimDegrees = 0.0f;
inline constexpr float kDiveDragCompensationPercent = 0.0f;
inline constexpr uint32_t kCommandLockTimeoutMs = 0;
inline constexpr uint32_t kTickLockTimeoutMs = 0;
inline constexpr uint32_t kDeviceRuntimeLockTimeoutMs = 0;
inline constexpr uint32_t kDeviceMaintenanceTimeoutMs = 100;
}

namespace robofish::config::radio {
inline constexpr uint32_t kPollPeriodMs = RF_MAIN_LOOP_DELAY_MS;
inline constexpr unsigned kReadBudgetBytes = 128;
inline constexpr unsigned kRecentTransactionCount = 4;
// Fixed source capacity; never evict sequence history to admit an unknown source.
inline constexpr unsigned kMaximumSources = 4;
// 实际空速、0x0E/0x0F、隐式扫描/重发/唤醒和对端静默窗未核实。
// 默认只接收；不能用厂商默认值或 AUX 高电平伪造空口时长已验证。
struct Timing {
  bool ackTxEnabled = false;
  bool peerWindowsVerified = false;
  bool moduleConfigurationVerified = false;
  unsigned packetBytes = 0;
  uint32_t packetIdleUs = 0;
  uint32_t replyDelayMs = lora::kAckDelayMs;
  uint32_t replyWindowMs = 0;
  uint32_t uartDrainTimeoutMs = 0;
  uint32_t rfReleaseBudgetMs = 0; // 包括组包等待、空口尾部和保护余量。
};
inline constexpr Timing kTiming{};
}

// ADC acquisition and motion interlock for a standard 7.4 V nominal 2S pack.
namespace robofish::config::battery {
inline constexpr uint32_t kSamplePeriodMs = 100;
inline constexpr uint32_t kConversionTimeoutMs = 10;
inline constexpr uint32_t kMaximumSampleAgeMs = 500;
inline constexpr unsigned kFilterDivisor = 4;
// Inclusive operating limits, mV: 3.3 V/cell cutoff, 4.2 V/cell full charge.
// Outside these limits or with invalid/stale samples, stop all motion.
inline constexpr uint16_t kMinimumOperatingMv = 6600;
inline constexpr uint16_t kMaximumOperatingMv = 8400;
}

namespace robofish::config::propulsion {
// User-confirmed 2S nominal reference, mV; each current signed motor command
// is multiplied by reference/measured voltage and limited to [-100, 100].
inline constexpr uint16_t kReferenceVoltageMv = 7400;
}

// First-phase safety parameters. Units ms unless stated; LoRa must renew goals.
namespace robofish::config::safety {
inline constexpr uint32_t kLoRaTargetLeaseMs = 3000; // User-approved integration initial value.
inline constexpr uint32_t kRadioDeadlineMs = 100;
inline constexpr uint32_t kControlDeadlineMs = 100;
inline constexpr uint32_t kVisionDeadlineMs = 100;
inline constexpr uint32_t kSpiFreshnessMs = 100; // Must match RFS1 OFFLINE_MS.
inline constexpr uint32_t kControlledNeutralMs = 40; // Two 20ms PWM periods, not position feedback.
inline constexpr uint32_t kOutputNeutralFailures = 5;
inline constexpr uint32_t kOutputResetFailures = 15;
static_assert(kOutputResetFailures > kOutputNeutralFailures);
}
