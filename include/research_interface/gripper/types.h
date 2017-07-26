#pragma once

#include <cstdint>
#include <type_traits>

namespace research_interface {
namespace gripper {

#pragma pack(push, 1)

using Version = uint16_t;

constexpr Version kVersion = 1;
constexpr uint16_t kCommandPort = 1338;

enum class Command : uint16_t { kConnect, kHoming, kGrasp, kMove, kStop };

struct CommandHeader {
  CommandHeader() = default;
  CommandHeader(Command command, uint32_t command_id) : command(command), command_id(command_id) {}

  Command command;
  uint32_t command_id;
};

template <typename T>
struct RequestBase {
  RequestBase(uint32_t command_id) : header(T::kCommand, command_id) {}
  const union { const CommandHeader header; };
};

template <typename T>
struct ResponseBase {
  ResponseBase(uint32_t command_id, typename T::Status status)
      : header(T::kCommand, command_id), status(status) {}

  const union { const CommandHeader header; };
  const typename T::Status status;

  static_assert(std::is_enum<decltype(status)>::value, "Status must be an enum.");
  static_assert(
      std::is_same<typename std::underlying_type<decltype(status)>::type, uint16_t>::value,
      "Status must be of type uint16_t.");
  static_assert(static_cast<uint16_t>(decltype(status)::kSuccess) == 0,
                "Status must define kSuccess with value of 0.");
};

template <typename T, Command C>
struct CommandBase {
  static constexpr Command kCommand = C;

  enum class Status : uint16_t { kSuccess, kFail, kUnsuccessful };

  using Header = CommandHeader;
  using Request = RequestBase<T>;
  using Response = ResponseBase<T>;
};

struct Connect : CommandBase<Connect, Command::kConnect> {
  enum class Status : uint16_t { kSuccess, kIncompatibleLibraryVersion };

  struct Request : public RequestBase<Connect> {
    Request(uint32_t command_id, uint16_t udp_port)
        : RequestBase(command_id), version(kVersion), udp_port(udp_port) {}

    const Version version;
    const uint16_t udp_port;
  };

  struct Response : public ResponseBase<Connect> {
    Response(uint32_t command_id, Status status)
        : ResponseBase(command_id, status), version(kVersion) {}

    const Version version;
  };
};

struct Homing : public CommandBase<Homing, Command::kHoming> {};

struct Grasp : public CommandBase<Grasp, Command::kGrasp> {
  struct Request : public RequestBase<Grasp> {
    Request(uint32_t command_id, double width, double speed, double force)
        : RequestBase(command_id), width(width), speed(speed), force(force) {}

    const double width;
    const double speed;
    const double force;
  };
};

struct Move : public CommandBase<Move, Command::kMove> {
  struct Request : public RequestBase<Move> {
    Request(uint32_t command_id, double width, double speed)
        : RequestBase(command_id), width(width), speed(speed) {}

    const double width;
    const double speed;
  };
};

struct Stop : public CommandBase<Stop, Command::kStop> {};

struct GripperState {
  uint32_t message_id;
  double width;
  double max_width;
  bool is_grasped;
  uint16_t temperature;
};

#pragma pack(pop)

}  // namespace gripper
}  // namespace research_interface
