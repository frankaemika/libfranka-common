// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

namespace research_interface {
namespace robot {

enum class StatusReason : size_t {
  kNoReason,
  kMoveFinished,
  kCommandApplied,
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
  kStopFinished,
  kStopMovePreempted,
  kRcuInputErrorAborted,
  kNotValidElbowRejected,
  kGuidingSlowDownFinished,
  kStartAtSingularPoseRejected
};

}  // namespace robot
}  // namespace research_interface
