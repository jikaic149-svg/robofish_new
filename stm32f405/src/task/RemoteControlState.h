#pragma once

#include "control/l0/IntentAdapter.h"
#include "channel/LoRaCommand.h"

// 仅由 MotionController 在状态锁内持有和更新，不作为共享全局状态。
struct RemoteControlState {
  robofish::control::l0::BodyMotionTarget target = [] {
    robofish::control::l0::BodyMotionTarget value;
    value.mode = DriveMode::Prop;  // 旧程序启动为纯桨模式。
    return value;
  }();
  bool active = false;
  bool immediateNeutral = false;
};

// 协议动作保持语义；只改目标，不写舵机、桨或泵。可在主机直接验证。
inline bool applyRemoteCommand(RemoteControlState &state,
                               const robofish::lora::Command &command) {
  if (command.code < 1 || command.code > 19 ||
      command.code == 8 || command.code == 18 || command.code == 19) return false;
  auto &target = state.target;
  state.active = true;
  state.immediateNeutral = command.code == 17;
  target.enabled = true;
  if (command.hasKnob) target.knob = command.knob;
  switch (command.code) {
    case 1:
      target.surgePercent = target.yawPercent = 0;
      target.allowFlowAssist = false;  // STOP 保持胸鳍，但不能因造流策略重新开桨。
      break;
    case 2: target.surgePercent = 100; target.yawPercent = 0; break;
    case 3: target.surgePercent = 0; target.yawPercent = -100; break;
    case 4: target.surgePercent = 0; target.yawPercent = 100; break;
    case 5:
      target.mode = target.mode == DriveMode::Prop ? DriveMode::Bionic
                    : target.mode == DriveMode::Bionic ? DriveMode::Hybrid : DriveMode::Prop;
      break;
    case 6:
      target.heavePercent = 100;
      target.allowFlowAssist = true;
      break;
    case 7:
      target.heavePercent = target.heavePercent > 0 ? 0 : -100;
      target.allowFlowAssist = target.heavePercent != 0;
      break;
    case 9: target.surgePercent = -100; target.yawPercent = 0; break;
    case 10: target.surgePercent = 100; target.yawPercent = -100; break;
    case 11: target.surgePercent = 100; target.yawPercent = 100; break;
    case 12: target.surgePercent = -100; target.yawPercent = -100; break;
    case 13: target.surgePercent = -100; target.yawPercent = 100; break;
    case 14: target.mode = DriveMode::Prop; break;
    case 15: target.mode = DriveMode::Bionic; break;
    case 16: target.mode = DriveMode::Hybrid; break;
    case 17:
      target.enabled = false;
      target.surgePercent = target.yawPercent = target.heavePercent = 0;
      target.knob = 0;
      target.allowFlowAssist = false;
      state.immediateNeutral = true;
      break;
  }
  return true;
}
