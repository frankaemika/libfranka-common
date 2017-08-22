#pragma once

#include <array>
#include <cstdint>
#include <type_traits>

namespace research_interface {
namespace robot {

#pragma pack(push, 1)

using Version = uint16_t;

constexpr Version kVersion = 1;
constexpr uint16_t kCommandPort = 1337;

enum class Command : uint32_t {
  kConnect,
  kMove,
  kStopMove,
  kGetCartesianLimit,
  kSetControllerMode,
  kSetCollisionBehavior,
  kSetJointImpedance,
  kSetCartesianImpedance,
  kSetGuidingMode,
  kSetEEToK,
  kSetFToEE,
  kSetLoad,
  kSetTimeScalingFactor,
  kAutomaticErrorRecovery,
  kLoadModelLibrary
};

struct CommandHeader {
  CommandHeader() = default;
  CommandHeader(Command command, uint32_t command_id) : command(command), command_id(command_id) {}

  Command command;
  uint32_t command_id;
};

template <typename T>
struct RequestBase {
  RequestBase(uint32_t command_id) : header(T::kCommand, command_id) {}

  const CommandHeader header;
};

template <typename T>
struct ResponseBase {
  ResponseBase(uint32_t command_id, typename T::Status status)
      : header(T::kCommand, command_id), status(status) {}

  const CommandHeader header;
  const typename T::Status status;

  static_assert(std::is_enum<decltype(status)>::value, "Status must be an enum.");
  static_assert(
      std::is_same<typename std::underlying_type<decltype(status)>::type, uint32_t>::value,
      "Status must be of type uint32_t.");
  static_assert(static_cast<uint32_t>(decltype(status)::kSuccess) == 0,
                "Status must define kSuccess with value of 0.");
};

template <typename T, Command C>
struct CommandBase {
  static constexpr Command kCommand = C;

  enum class Status : uint32_t { kSuccess, kAborted, kRejected, kPreempted };

  using Header = CommandHeader;
  using Request = RequestBase<T>;
  using Response = ResponseBase<T>;
};

struct Connect : CommandBase<Connect, Command::kConnect> {
  enum class Status : uint32_t { kSuccess, kIncompatibleLibraryVersion };

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

struct Move : public CommandBase<Move, Command::kMove> {
  enum class ControllerMode : uint32_t {
    kMotorPD,
    kJointPosition,
    kJointImpedance,
    kCartesianImpedance,
    kExternalController
  };

  enum class MotionGeneratorMode : uint32_t {
    kJointPosition,
    kJointVelocity,
    kCartesianPosition,
    kCartesianVelocity
  };

  enum class Status : uint32_t { kSuccess, kAborted, kRejected, kPreempted, kMotionStarted };

  struct Deviation {
    constexpr Deviation(double translation, double rotation, double elbow)
        : translation(translation), rotation(rotation), elbow(elbow) {}
    const double translation;
    const double rotation;
    const double elbow;
  };

  struct Request : public RequestBase<Move> {
    Request(uint32_t command_id,
            ControllerMode controller_mode,
            MotionGeneratorMode motion_generator_mode,
            const Deviation& maximum_path_deviation,
            const Deviation& maximum_goal_pose_deviation)
        : RequestBase(command_id),
          controller_mode(controller_mode),
          motion_generator_mode(motion_generator_mode),
          maximum_path_deviation(maximum_path_deviation),
          maximum_goal_pose_deviation(maximum_goal_pose_deviation) {}

    const ControllerMode controller_mode;
    const MotionGeneratorMode motion_generator_mode;
    const Deviation maximum_path_deviation;
    const Deviation maximum_goal_pose_deviation;
  };
};

struct StopMove : public CommandBase<StopMove, Command::kStopMove> {};

struct GetCartesianLimit : public CommandBase<GetCartesianLimit, Command::kGetCartesianLimit> {
  struct Request : public RequestBase<GetCartesianLimit> {
    Request(uint32_t command_id, int32_t id) : RequestBase(command_id), id(id) {}

    const int32_t id;
  };

  struct Response : public ResponseBase<GetCartesianLimit> {
    Response(uint32_t command_id,
             Status status,
             const std::array<double, 3>& object_p_min,
             const std::array<double, 3>& object_p_max,
             const std::array<double, 16>& object_frame,
             bool object_activation)
        : ResponseBase(command_id, status),
          object_p_min(object_p_min),
          object_p_max(object_p_max),
          object_frame(object_frame),
          object_activation(object_activation) {}
    Response(uint32_t command_id, Status status)
        : Response(command_id, status, {}, {}, {}, false) {}

    const std::array<double, 3> object_p_min;
    const std::array<double, 3> object_p_max;
    const std::array<double, 16> object_frame;
    const bool object_activation;
  };
};

struct SetControllerMode : public CommandBase<SetControllerMode, Command::kSetControllerMode> {
  enum class ControllerMode : uint32_t {
    kMotorPD,
    kJointPosition,
    kJointImpedance,
    kCartesianImpedance,
    kExternalController
  };

  struct Request : public RequestBase<SetControllerMode> {
    Request(uint32_t command_id, ControllerMode mode) : RequestBase(command_id), mode(mode) {}

    const ControllerMode mode;
  };
};

struct SetCollisionBehavior
    : public CommandBase<SetCollisionBehavior, Command::kSetCollisionBehavior> {
  struct Request : public RequestBase<SetCollisionBehavior> {
    Request(uint32_t command_id,
            const std::array<double, 7> lower_torque_thresholds_acceleration,
            const std::array<double, 7> upper_torque_thresholds_acceleration,
            const std::array<double, 7> lower_torque_thresholds_nominal,
            const std::array<double, 7> upper_torque_thresholds_nominal,
            const std::array<double, 6> lower_force_thresholds_acceleration,
            const std::array<double, 6> upper_force_thresholds_acceleration,
            const std::array<double, 6> lower_force_thresholds_nominal,
            const std::array<double, 6> upper_force_thresholds_nominal)
        : RequestBase(command_id),
          lower_torque_thresholds_acceleration(lower_torque_thresholds_acceleration),
          upper_torque_thresholds_acceleration(upper_torque_thresholds_acceleration),
          lower_torque_thresholds_nominal(lower_torque_thresholds_nominal),
          upper_torque_thresholds_nominal(upper_torque_thresholds_nominal),
          lower_force_thresholds_acceleration(lower_force_thresholds_acceleration),
          upper_force_thresholds_acceleration(upper_force_thresholds_acceleration),
          lower_force_thresholds_nominal(lower_force_thresholds_nominal),
          upper_force_thresholds_nominal(upper_force_thresholds_nominal) {}

    const std::array<double, 7> lower_torque_thresholds_acceleration;
    const std::array<double, 7> upper_torque_thresholds_acceleration;

    const std::array<double, 7> lower_torque_thresholds_nominal;
    const std::array<double, 7> upper_torque_thresholds_nominal;

    const std::array<double, 6> lower_force_thresholds_acceleration;
    const std::array<double, 6> upper_force_thresholds_acceleration;

    const std::array<double, 6> lower_force_thresholds_nominal;
    const std::array<double, 6> upper_force_thresholds_nominal;
  };
};

struct SetJointImpedance : public CommandBase<SetJointImpedance, Command::kSetJointImpedance> {
  struct Request : public RequestBase<SetJointImpedance> {
    Request(uint32_t command_id, const std::array<double, 7> K_theta)
        : RequestBase(command_id), K_theta(K_theta) {}

    const std::array<double, 7> K_theta;
  };
};

struct SetCartesianImpedance
    : public CommandBase<SetCartesianImpedance, Command::kSetCartesianImpedance> {
  struct Request : public RequestBase<SetCartesianImpedance> {
    Request(uint32_t command_id, const std::array<double, 6> K_x)
        : RequestBase(command_id), K_x(K_x) {}

    const std::array<double, 6> K_x;
  };
};

struct SetGuidingMode : public CommandBase<SetGuidingMode, Command::kSetGuidingMode> {
  struct Request : public RequestBase<SetGuidingMode> {
    Request(uint32_t command_id, const std::array<bool, 6>& guiding_mode, bool nullspace)
        : RequestBase(command_id), guiding_mode(guiding_mode), nullspace(nullspace) {}

    const std::array<bool, 6> guiding_mode;
    const bool nullspace;
  };
};

struct SetEEToK : public CommandBase<SetEEToK, Command::kSetEEToK> {
  struct Request : public RequestBase<SetEEToK> {
    Request(uint32_t command_id, const std::array<double, 16>& EE_T_K)
        : RequestBase(command_id), EE_T_K(EE_T_K) {}

    const std::array<double, 16> EE_T_K;
  };
};

struct SetFToEE : public CommandBase<SetFToEE, Command::kSetFToEE> {
  struct Request : public RequestBase<SetFToEE> {
    Request(uint32_t command_id, const std::array<double, 16>& F_T_EE)
        : RequestBase(command_id), F_T_EE(F_T_EE) {}

    const std::array<double, 16> F_T_EE;
  };
};

struct SetLoad : public CommandBase<SetLoad, Command::kSetLoad> {
  struct Request : public RequestBase<SetLoad> {
    Request(uint32_t command_id,
            double m_load,
            const std::array<double, 3>& F_x_Cload,
            const std::array<double, 9>& I_load)
        : RequestBase(command_id), m_load(m_load), F_x_Cload(F_x_Cload), I_load(I_load) {}

    const double m_load;
    const std::array<double, 3> F_x_Cload;
    const std::array<double, 9> I_load;
  };
};

struct SetTimeScalingFactor
    : public CommandBase<SetTimeScalingFactor, Command::kSetTimeScalingFactor> {
  struct Request : public RequestBase<SetTimeScalingFactor> {
    Request(uint32_t command_id, double time_scaling_factor)
        : RequestBase(command_id), time_scaling_factor(time_scaling_factor) {}

    const double time_scaling_factor;
  };
};

struct AutomaticErrorRecovery
    : public CommandBase<AutomaticErrorRecovery, Command::kAutomaticErrorRecovery> {};

struct LoadModelLibrary : public CommandBase<LoadModelLibrary, Command::kLoadModelLibrary> {
  enum class Status : uint32_t { kSuccess, kError };

  enum class Architecture : uint8_t { kX64 };

  enum class System : uint8_t { kLinux };

  struct Request : public RequestBase<LoadModelLibrary> {
    Request(uint32_t command_id, Architecture architecture, System system)
        : RequestBase(command_id), architecture(architecture), system(system) {}

    const Architecture architecture;
    const System system;
  };

  struct Response : public ResponseBase<LoadModelLibrary> {
    Response(uint32_t command_id, Status status, uint32_t size)
        : ResponseBase(command_id, status), size(size) {}
    Response(uint32_t command_id, Status status) : Response(command_id, status, 0u) {}

    const uint32_t size;
  };
};

#pragma pack(pop)

}  // namespace robot
}  // namespace research_interface
