#pragma once

#include <cstdint>

namespace research_interface {

using Version = uint16_t;

constexpr Version kVersion = 1;
constexpr uint16_t kCommandPort = 1337;

enum class Function : uint32_t {
  kConnect = 0,
  kStartMotionGenerator = 1,
  kStopMotionGenerator = 2,
  kStartController = 3,
  kStopController = 4
};

struct ConnectRequest {
  ConnectRequest(uint16_t udp_port)
      : function(Function::kConnect), version(kVersion), udp_port(udp_port) {}

  const Function function;
  const Version version;
  const uint16_t udp_port;
};

struct ConnectReply {
  enum class Status : uint32_t {
    kSuccess = 0,
    kIncompatibleLibraryVersion = 1
  };

  ConnectReply(Status status) : status(status), version(kVersion) {}

  const Status status;
  const Version version;
};

struct StartMotionGeneratorRequest {
  enum class Type : uint32_t {
    kJointPosition = 0,
    kJointVelocity = 1,
    kCartesianPosition = 2,
    kCartesianVelocity = 3
  };

  StartMotionGeneratorRequest(Type type)
      : function(Function::kStartMotionGenerator), type(type) {}

  const Function function;
  const Type type;
};

struct StartMotionGeneratorReply {
  enum class Status : uint32_t {
    kSuccess = 0,
    kNotConnected = 1,
    kInvalidType = 2
  };

  StartMotionGeneratorReply(Status status) : status(status) {}
  const Status status;
};

struct BaseModeChangeReply {
  enum class Status : uint32_t {
    kSuccess = 0,
    kNotConnected = 1
  };

  BaseModeChangeReply(Status status) : status(status) {}
  const Status status;
};

struct StopMotionGeneratorRequest {
  StopMotionGeneratorRequest()
      : function(Function::kStopMotionGenerator) {}

  const Function function;
};

struct StopMotionGeneratorReply : public BaseModeChangeReply {
  using BaseModeChangeReply::BaseModeChangeReply;
};

struct StartControllerRequest {
  StartControllerRequest()
      : function(Function::kStartController) {}

  const Function function;
};

struct StartControllerReply : public BaseModeChangeReply {
  using BaseModeChangeReply::BaseModeChangeReply;
};

struct StopControllerRequest {
  StopControllerRequest()
      : function(Function::kStopController) {}

  const Function function;
};

struct StopControllerReply : public BaseModeChangeReply {
  using BaseModeChangeReply::BaseModeChangeReply;
};

}  // namespace research_interface
