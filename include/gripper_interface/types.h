#pragma once

#include <cinttypes>
#include <type_traits>

namespace gripper_interface {

#pragma pack(push, 1)

using Version = uint16_t;

constexpr Version kVersion = 1;
constexpr uint16_t kCommandPort = 1338;

enum class Function : uint16_t { kConnect, kHoming, kGrasp, kMove, kStop, kRelease };

template <typename T>
struct RequestBase {
  RequestBase() : function(T::kFunction) {}
  const Function function;
};

template <typename T>
struct ResponseBase {
  ResponseBase(typename T::Status status) : function(T::kFunction), status(status) {}

  const Function function;
  const typename T::Status status;

  static_assert(std::is_enum<decltype(status)>::value, "Status must be an enum.");
  static_assert(
      std::is_same<typename std::underlying_type<decltype(status)>::type, uint16_t>::value,
      "Status must be of type uint16_t.");
  static_assert(static_cast<uint16_t>(decltype(status)::kSuccess) == 0,
                "Status must define kSuccess with value of 0.");
};

template <typename T, Function F>
struct CommandBase {
  static constexpr Function kFunction = F;

  enum class Status : uint16_t { kSuccess, kFail, kUnsuccessful };

  using Request = RequestBase<T>;
  using Response = ResponseBase<T>;
};

struct Connect : CommandBase<Connect, Function::kConnect> {
  enum class Status : uint16_t { kSuccess, kIncompatibleLibraryVersion };

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

struct Homing : public CommandBase<Homing, Function::kHoming> {};

struct Grasp : public CommandBase<Grasp, Function::kGrasp> {
  struct Request : public RequestBase<Grasp> {
    Request(double width, double speed, double force) : width(width), speed(speed), force(force) {}

    const double width;
    const double speed;
    const double force;
  };
};

struct Move : public CommandBase<Move, Function::kMove> {
  struct Request : public RequestBase<Move> {
    Request(double width, double speed) : width(width), speed(speed) {}

    const double width;
    const double speed;
  };
};

struct Stop : public CommandBase<Stop, Function::kStop> {};

struct Release : public CommandBase<Release, Function::kRelease> {
  struct Request : public RequestBase<Release> {
    Request(double width, double speed) : width(width), speed(speed) {}

    const double width;
    const double speed;
  };
};

enum class Command : uint16_t { kNone, kHoming, kGrasp, kMove, kStop, kRelease };

struct GripperState {
  Command command;
  double width;
  double max_width;
  bool is_grasped;
  uint16_t temperature;
};

#pragma pack(pop)

}  // namespace gripper_interface
