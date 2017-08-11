#pragma once

#include <array>
#include <cstdint>

namespace research_interface {
namespace robot {

#pragma pack(push, 1)

enum class MotionGeneratorMode : uint8_t {
  kIdle,
  kJointPosition,
  kJointVelocity,
  kCartesianPosition,
  kCartesianVelocity
};

enum class ControllerMode : uint8_t {
  kMotorPD,
  kJointPosition,
  kJointImpedance,
  kCartesianImpedance,
  kExternalController
};

enum class RobotMode : uint8_t {
  kOther,
  kIdle,
  kMove,
  kMoveWithController,
  kGuiding,
  kReflex,
  kUserStopped,
  kAutomaticErrorRecovery
};

struct RobotState {
  uint64_t message_id;
  std::array<double, 16> O_T_EE;
  std::array<double, 16> O_T_EE_d;
  std::array<double, 16> F_T_EE;
  std::array<double, 16> EE_T_K;
  double m_load;
  std::array<double, 9> I_load;
  std::array<double, 3> F_x_Cload;
  std::array<double, 2> elbow;
  std::array<double, 2> elbow_d;
  std::array<double, 7> tau_J;
  std::array<double, 7> dtau_J;
  std::array<double, 7> q;
  std::array<double, 7> q_d;
  std::array<double, 7> dq;
  std::array<double, 7> dq_d;
  std::array<double, 7> joint_contact;
  std::array<double, 6> cartesian_contact;
  std::array<double, 7> joint_collision;
  std::array<double, 6> cartesian_collision;
  std::array<double, 7> tau_ext_hat_filtered;
  std::array<double, 6> O_F_ext_hat_K;
  std::array<double, 6> K_F_ext_hat_K;
  MotionGeneratorMode motion_generator_mode;
  ControllerMode controller_mode;
  std::array<bool, 27> errors;
  std::array<bool, 27> reflex_reason;
  RobotMode robot_mode;
};

struct MotionGeneratorCommand {
  std::array<double, 7> q_d;
  std::array<double, 7> dq_d;
  std::array<double, 16> O_T_EE_d;
  std::array<double, 6> O_dP_EE_d;
  std::array<double, 2> elbow_d;
  bool valid_elbow;
  bool motion_generation_finished;
};

struct ControllerCommand {
  std::array<double, 7> tau_J_d;
};

struct RobotCommand {
  uint64_t message_id;
  MotionGeneratorCommand motion;
  ControllerCommand control;
};

#pragma pack(pop)

}  // namespace robot
}  // namespace research_interface
