#pragma once

#include <cinttypes>
#include <type_traits>

namespace research_interface {

using Version = uint16_t;

constexpr Version kVersion = 1;
constexpr uint16_t kCommandPort = 1337;

enum class Function : uint32_t {
  kConnect,
  kStartMotionGenerator,
  kStopMotionGenerator,
  kStartController,
  kStopController
};

template <typename T>
struct RequestBase {
  RequestBase() : function(T::kFunction) {}
  const Function function;
};

template <typename T>
struct ResponseBase {
  ResponseBase(typename T::Status status)
      : function(T::kFunction), status(status) {}

  const Function function;
  const typename T::Status status;

  static_assert(std::is_enum<decltype(status)>::value,
                "Status must be an enum.");
  static_assert(
      std::is_same<typename std::underlying_type<decltype(status)>::type,
                   uint32_t>::value,
      "Status must be of type uint32_t.");
  static_assert(static_cast<uint32_t>(decltype(status)::kSuccess) == 0,
                "Status must define kSuccess with value of 0.");
};

template <typename T, Function F>
struct CommandBase {
  static constexpr Function kFunction = F;

  enum class Status : uint32_t { kSuccess };

  using Request = RequestBase<T>;
  using Response = ResponseBase<T>;
};

struct Connect : CommandBase<Connect, Function::kConnect> {
  enum class Status : uint32_t { kSuccess, kIncompatibleLibraryVersion };

  struct Request : public RequestBase<Connect> {
    Request(uint16_t udp_port) : version(kVersion), udp_port(udp_port) {}

    const Version version;
    const uint16_t udp_port;
  };

  struct Response : public ResponseBase<Connect> {
    Response(Status status) : ResponseBase(status), version(kVersion) {}

    const Version version;
  };
};

struct StartMotionGenerator
    : public CommandBase<StartMotionGenerator,
                         Function::kStartMotionGenerator> {
  enum class MotionGeneratorMode : uint32_t {
    kJointPosition,
    kJointVelocity,
    kCartesianPosition,
    kCartesianVelocity
  };

  enum class Status : uint32_t {
    kSuccess,
    kInvalidType,
    kFinished,
    kAborted,
    kRejected
  };

  struct Request : public RequestBase<StartMotionGenerator> {
    Request(MotionGeneratorMode mode) : mode(mode) {}

    const MotionGeneratorMode mode;
  };
};

struct StopMotionGenerator
    : public CommandBase<StopMotionGenerator, Function::kStopMotionGenerator> {
};

struct StartController
    : public CommandBase<StartController, Function::kStartController> {};

struct StopController
    : public CommandBase<StopController, Function::kStopController> {};

}  // namespace research_interface
