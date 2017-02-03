#pragma once

#include <array>

namespace research_interface {

struct RobotState {
  std::array<double, 7> q_start;
  std::array<double, 16> O_T_EE_start;
  std::array<double, 2> elbow_start;
  std::array<double, 7> tau_J;
  std::array<double, 7> dtau_J;
  std::array<double, 7> q;
  std::array<double, 7> dq;
  std::array<double, 7> q_d;
  std::array<double, 7> joint_contact;
  std::array<double, 6> cartesian_contact;
  std::array<double, 7> joint_collision;
  std::array<double, 6> cartesian_collision;
  std::array<double, 7> tau_ext_hat_filtered;
  std::array<double, 6> O_F_ext_hat_EE;
  std::array<double, 6> EE_F_ext_hat_EE;
};

struct MotionGeneratorCommand {
  std::array<double, 7> q_d;
  std::array<double, 7> dq_d;
  std::array<double, 7> ddq_d;
  std::array<double, 16> O_T_EE_d;
  std::array<double, 6> O_dP_EE_d;
  std::array<double, 2> elbow_d;
  std::array<double, 2> delbow_d;
  bool valid_elbow;
  bool motion_generation_finished;
};

struct ControllerCommand {
  std::array<double, 7> tau_J_d;
};

struct RobotCommand {
  MotionGeneratorCommand motion;
  ControllerCommand control;
};

}  // namespace research_interface
