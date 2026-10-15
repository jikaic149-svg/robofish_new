#pragma once

// ============================================================================
// RoboFish runtime configuration
// 集中保存产品行为、控制周期和服务参数，不包含板级引脚映射。
// ============================================================================

// RadioPoll cadence, milliseconds.
#define RF_MAIN_LOOP_DELAY_MS 10

// Geometric tail limits. The shared physical centers and gait are below.
#define RF_TAIL_MIN_ANGLE       20
#define RF_TAIL_MAX_ANGLE      152

// Legacy input/display units, never a second physical actuator profile.
// Both 86 (old API) and 90 (UI) mean neutral at the L0 pitch boundary.
#define RF_PITCH_CENTER         86
#define RF_PITCH_UI_NEUTRAL     90
#define RF_PITCH_MIN_ANGLE      45
#define RF_PITCH_MAX_ANGLE     135
#define RF_ESC_THROTTLE_NEUTRAL 90
#define RF_ESC_STEERING_CENTER  90

// Safety timeouts. The partner may study and adjust these values, but disabling
// them changes the propulsion timeout behavior.
#define RF_PROPULSION_TIMEOUT_MS 2000

// Computer-vision control lease. The PC must refresh /vision_control within
// this many milliseconds; otherwise propulsion stops and steering centers.
#define RF_VISION_COMMAND_TIMEOUT_MS 500
// Fixed-period PC tracking datagrams. UDP is only a data plane; HTTP/manual
// commands hold a short priority lease so a late datagram cannot re-arm motion.
#define RF_VISION_HTTP_PRIORITY_HOLD_MS 400

// Application task priorities on Core 1. Actuator output must preempt command
// ingestion; the vision lease watchdog must preempt the UDP receive loop.
#define RF_MOTION_CONTROL_TASK_PRIORITY 4
#define RF_VISION_LEASE_TASK_PRIORITY 3

// MCU-side visual control tuning. Inputs are metres.  The LOS heading error
// is already filtered and latency-compensated on the PC, so X uses P only;
// adding I/D here amplifies fish-tail marker swing and delayed samples.
// 300 %/m maps the visual +/-0.333 m limit to +/-100% steering.
#define RF_VISION_X_KP 300.0f
#define RF_VISION_X_KI   0.0f
#define RF_VISION_X_KD   0.0f
// 800 %/m reaches full straight power at about 0.125 m. This keeps normal
// tracking firm while retaining a short proportional slowdown near the goal.
#define RF_VISION_Y_KP 800.0f
#define RF_VISION_Y_KI   0.0f
#define RF_VISION_Y_KD   0.0f
#define RF_VISION_PID_INTEGRAL_LIMIT 0.50f
#define RF_VISION_PID_DEADBAND_M     0.006f

// Vision HYBRID open-loop turn mixing.  The PC sends total turn demand after
// combining curvature feedforward, heading error and cross-track error.  At
// 100% demand the power endpoints intentionally match the web HYBRID full-turn
// profile (tail 75%, propeller 75%), whose observed circle diameter is roughly
// 0.40 m.  That observation is an initial estimate, not a precision calibration.
// Measured camera speed is deliberately not closed around throttle because its
// latency would create another oscillating loop.  Terminal braking keeps the
// existing neutral/dwell/current-limited reverse sequence.
#define RF_VISION_TAIL_STRAIGHT_POWER_PERCENT       74
#define RF_VISION_TAIL_TURN_POWER_PERCENT           75
#define RF_VISION_PROP_CURVE_MIN_POWER_PERCENT      75
#define RF_VISION_BRAKE_REVERSE_POWER_PERCENT       26
#define RF_VISION_BRAKE_REAPPROACH_POWER_PERCENT    26
#define RF_VISION_BRAKE_NEUTRAL_DWELL_MS           160
#define RF_VISION_BRAKE_REVERSE_MIN_MS              120
#define RF_VISION_BRAKE_REVERSE_MAX_MS              480
#define RF_VISION_BRAKE_SETTLED_SPEED_MPS          0.03f

// 用户提供的 E32 四鳍/双独立桨标定。旧 RF_* SDK 数值保持不变；
// 本组只用于四鳍硬件及 LoRa 原始动作，避免把两套机械标定混为一套。
namespace robofish::config::lora {
inline constexpr int kBaud = 115200;
inline constexpr int kFishNode = 1;
inline constexpr unsigned kMaxLineLength = 32;
inline constexpr unsigned kAckDelayMs = 260;
inline constexpr unsigned kEscArmMs = 2000;
inline constexpr int kPectoralLeftCenter = 90;
inline constexpr int kPectoralRightCenter = 90;
inline constexpr int kTailFrontCenter = 90;
inline constexpr int kTailRearCenter = 90;
inline constexpr int kPectoralLeftMinimum = 45;
inline constexpr int kPectoralLeftMaximum = 145;
inline constexpr int kPectoralRightMinimum = 35;
inline constexpr int kPectoralRightMaximum = 155;
inline constexpr int kPectoralLeftUp = 140;
inline constexpr int kPectoralRightUp = 40;
inline constexpr int kPectoralLeftDown = 50;
inline constexpr int kPectoralRightDown = 130;
inline constexpr float kPectoralScale = 0.85f;
inline constexpr float kServoStepPer10Ms = 2.5f;
inline constexpr float kTailStaticSteeringDegrees = 40.0f;
inline constexpr float kTailAmplitudeDegrees = 25.0f;
inline constexpr float kTailTurnBiasDegrees = 18.0f;
inline constexpr float kTailFrequencyHz = 1.5f;
inline constexpr float kRearTailAmplitudeRatio = 0.90f;
inline constexpr float kBionicPhaseLagDegrees = 60.0f;
inline constexpr float kHybridPhaseLagDegrees = 72.0f;
inline constexpr int kForwardDirection = 1;
inline constexpr int kTurnDirection = -1;
inline constexpr int kLeftEscDirection = 1;
inline constexpr int kRightEscDirection = -1;
inline constexpr int kTailSteeringDirection = 1;
inline constexpr int kTailFrontDirection = 1;
inline constexpr int kTailRearDirection = 1;
inline constexpr int kGearMinimumPercent = 50;
inline constexpr int kGearMaximumPercent = 100;
inline constexpr int kEscMinimumUs = 1000;
inline constexpr int kEscNeutralUs = 1500;
inline constexpr int kEscMaximumUs = 2000;
}  // namespace robofish::config::lora
