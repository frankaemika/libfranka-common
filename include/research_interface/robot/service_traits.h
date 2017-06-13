#pragma once

#include <research_interface/robot/service_types.h>

namespace research_interface {
namespace robot {

template <typename T>
struct CommandTraits {};

template <>
struct CommandTraits<Move> {
  static constexpr const char* kName = "Move";
};

template <>
struct CommandTraits<StopMove> {
  static constexpr const char* kName = "Stop Move";
};

template <>
struct CommandTraits<GetCartesianLimit> {
  static constexpr const char* kName = "Get Cartesian Limit";
};

template <>
struct CommandTraits<SetControllerMode> {
  static constexpr const char* kName = "Set Controller Mode";
};

template <>
struct CommandTraits<SetCollisionBehavior> {
  static constexpr const char* kName = "Set Collision Behavior";
};

template <>
struct CommandTraits<SetJointImpedance> {
  static constexpr const char* kName = "Set Joint Impedance";
};

template <>
struct CommandTraits<SetCartesianImpedance> {
  static constexpr const char* kName = "Set Cartesian Impedance";
};

template <>
struct CommandTraits<SetGuidingMode> {
  static constexpr const char* kName = "Set Guiding Mode";
};

template <>
struct CommandTraits<SetEEToK> {
  static constexpr const char* kName = "Set EE to K";
};

template <>
struct CommandTraits<SetFToEE> {
  static constexpr const char* kName = "Set F to EE";
};

template <>
struct CommandTraits<SetLoad> {
  static constexpr const char* kName = "Set Load";
};

template <>
struct CommandTraits<SetTimeScalingFactor> {
  static constexpr const char* kName = "Set Time Scaling Factor";
};

template <>
struct CommandTraits<AutomaticErrorRecovery> {
  static constexpr const char* kName = "Automatic Error Recovery";
};

}  // namespace robot
}  // namespace research_interface