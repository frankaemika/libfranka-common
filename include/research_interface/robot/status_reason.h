// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

namespace research_interface {
namespace robot {

enum class StatusReason : uint8_t {
  kOther,
  kStopCommandPreempted,
  kGuidingPreempted,
  kEmergencyAborted,
  kReflexAborted,
  kOutOfRangeRejected,
  kNotValidFrameRejected,
  kNotValidElementRejected,
  kElbowSignInconsistentRejected,
  kCommandNotPossibleRejected,
  kNoop,
  kStopMovePreempted,
  kRcuInputErrorAborted,
  kNotValidElbowRejected,
  kStartAtSingularPoseRejected
};

}  // namespace robot
}  // namespace research_interface
