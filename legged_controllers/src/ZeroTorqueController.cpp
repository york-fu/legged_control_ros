
#include "legged_controllers/ZeroTorqueController.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <pluginlib/class_list_macros.hpp>

namespace legged {

void ZeroTorqueController::update(const ros::Time& time, const ros::Duration& period) {
  // Update state estimation
  updateStateEstimation(time, period);

  vector_t joint_pos = getJointPos();
  vector_t joint_vel = vector_t::Zero(joint_pos.size());
  vector_t joint_eff = vector_t::Zero(joint_pos.size());

  // Write zero torque commands (keep current position, zero velocity, zero torque)
  setJointCmd(joint_pos, joint_vel, joint_eff);
}

void ZeroTorqueController::starting(const ros::Time& time) {
  // Initialize state estimation
  current_observation_.state.setZero(legged_interface_->getCentroidalModelInfo().stateDim);
  updateStateEstimation(time, ros::Duration(0.002));
  ROS_INFO("[ZeroTorqueController] Started - applying zero torque to all joints");
}

}  // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::ZeroTorqueController, controller_interface::ControllerBase)
