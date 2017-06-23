#pragma once

#include <array>
#include <cinttypes>
#include <type_traits>

namespace research_interface {
namespace robot {

#pragma pack(push, 1)

using Version = uint16_t;

constexpr Version kVersion = 1;
constexpr uint16_t kCommandPort = 1337;

enum class Function : uint32_t {
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
      std::is_same<typename std::underlying_type<decltype(status)>::type, uint32_t>::value,
      "Status must be of type uint32_t.");
  static_assert(static_cast<uint32_t>(decltype(status)::kSuccess) == 0,
                "Status must define kSuccess with value of 0.");
};

template <typename T, Function F>
struct CommandBase {
  static constexpr Function kFunction = F;

  enum class Status : uint32_t { kSuccess, kAborted, kRejected, kPreempted };

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

struct Move : public CommandBase<Move, Function::kMove> {
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
    Request(ControllerMode controller_mode,
            MotionGeneratorMode motion_generator_mode,
            const Deviation& maximum_path_deviation,
            const Deviation& maximum_goal_pose_deviation)
        : controller_mode(controller_mode),
          motion_generator_mode(motion_generator_mode),
          maximum_path_deviation(maximum_path_deviation),
          maximum_goal_pose_deviation(maximum_goal_pose_deviation) {}

    const ControllerMode controller_mode;
    const MotionGeneratorMode motion_generator_mode;
    const Deviation maximum_path_deviation;
    const Deviation maximum_goal_pose_deviation;
  };
};

struct StopMove : public CommandBase<StopMove, Function::kStopMove> {};

struct GetCartesianLimit : public CommandBase<GetCartesianLimit, Function::kGetCartesianLimit> {
  struct Request : public RequestBase<GetCartesianLimit> {
    Request(int32_t id) : id(id) {}

    const int32_t id;
  };

  struct Response : public ResponseBase<GetCartesianLimit> {
    Response(Status status) : Response(status, {}, {}, {}, {}) {}

    Response(Status status,
             const std::array<double, 3>& object_p_min,
             const std::array<double, 3>& object_p_max,
             const std::array<double, 16>& object_frame,
             bool object_activation)
        : ResponseBase<GetCartesianLimit>(status),
          object_p_min(object_p_min),
          object_p_max(object_p_max),
          object_frame(object_frame),
          object_activation(object_activation) {}

    const std::array<double, 3> object_p_min;
    const std::array<double, 3> object_p_max;
    const std::array<double, 16> object_frame;
    const bool object_activation;
  };
};

struct SetControllerMode : public CommandBase<SetControllerMode, Function::kSetControllerMode> {
  enum class ControllerMode : uint32_t {
    kMotorPD,
    kJointPosition,
    kJointImpedance,
    kCartesianImpedance,
    kExternalController
  };

  struct Request : public RequestBase<SetControllerMode> {
    Request(ControllerMode mode) : mode(mode) {}

    const ControllerMode mode;
  };
};

struct SetCollisionBehavior
    : public CommandBase<SetCollisionBehavior, Function::kSetCollisionBehavior> {
  struct Request : public RequestBase<SetCollisionBehavior> {
    Request(const std::array<double, 7> lower_torque_thresholds_acceleration,
            const std::array<double, 7> upper_torque_thresholds_acceleration,
            const std::array<double, 7> lower_torque_thresholds_nominal,
            const std::array<double, 7> upper_torque_thresholds_nominal,
            const std::array<double, 6> lower_force_thresholds_acceleration,
            const std::array<double, 6> upper_force_thresholds_acceleration,
            const std::array<double, 6> lower_force_thresholds_nominal,
            const std::array<double, 6> upper_force_thresholds_nominal)
        : lower_torque_thresholds_acceleration(lower_torque_thresholds_acceleration),
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

struct SetJointImpedance : public CommandBase<SetJointImpedance, Function::kSetJointImpedance> {
  struct Request : public RequestBase<SetJointImpedance> {
    Request(const std::array<double, 7> K_theta) : K_theta(K_theta) {}

    const std::array<double, 7> K_theta;
  };
};

struct SetCartesianImpedance
    : public CommandBase<SetCartesianImpedance, Function::kSetCartesianImpedance> {
  struct Request : public RequestBase<SetCartesianImpedance> {
    Request(const std::array<double, 6> K_x) : K_x(K_x) {}

    const std::array<double, 6> K_x;
  };
};

struct SetGuidingMode : public CommandBase<SetGuidingMode, Function::kSetGuidingMode> {
  struct Request : public RequestBase<SetGuidingMode> {
    Request(const std::array<bool, 6>& guiding_mode, bool nullspace)
        : guiding_mode(guiding_mode), nullspace(nullspace) {}

    const std::array<bool, 6> guiding_mode;
    const bool nullspace;
  };
};

struct SetEEToK : public CommandBase<SetEEToK, Function::kSetEEToK> {
  struct Request : public RequestBase<SetEEToK> {
    Request(const std::array<double, 16>& EE_T_K) : EE_T_K(EE_T_K) {}

    const std::array<double, 16> EE_T_K;
  };
};

struct SetFToEE : public CommandBase<SetFToEE, Function::kSetFToEE> {
  struct Request : public RequestBase<SetFToEE> {
    Request(const std::array<double, 16>& F_T_EE) : F_T_EE(F_T_EE) {}

    const std::array<double, 16> F_T_EE;
  };
};

struct SetLoad : public CommandBase<SetLoad, Function::kSetLoad> {
  struct Request : public RequestBase<SetLoad> {
    Request(double m_load,
            const std::array<double, 3>& F_x_Cload,
            const std::array<double, 9>& I_load)
        : m_load(m_load), F_x_Cload(F_x_Cload), I_load(I_load) {}

    const double m_load;
    const std::array<double, 3> F_x_Cload;
    const std::array<double, 9> I_load;
  };
};

struct SetTimeScalingFactor
    : public CommandBase<SetTimeScalingFactor, Function::kSetTimeScalingFactor> {
  struct Request : public RequestBase<SetTimeScalingFactor> {
    Request(double time_scaling_factor) : time_scaling_factor(time_scaling_factor) {}

    const double time_scaling_factor;
  };
};

struct AutomaticErrorRecovery
    : public CommandBase<AutomaticErrorRecovery, Function::kAutomaticErrorRecovery> {};

struct LoadModelLibrary
    : public CommandBase<LoadModelLibrary, Function::kLoadModelLibrary> {
  enum class Status : uint32_t { kSuccess, kRejected };

  enum class Architecture : uint8_t {
    kX64
  };

  enum class Platform : uint8_t {
    kLinux
  };

  struct Request : public RequestBase<LoadModelLibrary> {
    Request(Architecture architecture, Platform platform) : architecture(architecture), platform(platform) {}

    const Architecture architecture;
    const Platform platform;
  };

  struct Response : public ResponseBase<LoadModelLibrary> {
    Response() : ResponseBase(Status::kRejected), size(0) {}
    Response(uint32_t size) : ResponseBase(Status::kSuccess), size(size) {}

    const uint32_t size;
  };
};

#pragma pack(pop)

}  // namespace robot
}  // namespace research_interface
