//
// Created for damping controller.
//

#include "legged_controllers/DampingController.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <pluginlib/class_list_macros.hpp>

namespace legged {

bool DampingController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) {
  // Initialize base controller
  if (!BaseController::init(robot_hw, controller_nh)) {
    return false;
  }

  // Load damping_kd from parameter server (loaded by launch file from config/joint.yaml to root namespace)
  ros::NodeHandle nh_global;
  joint_kd_.resize(hybrid_joint_handles_.size());
  joint_kd_.setConstant(0.0);
  
  if (nh_global.hasParam("damping_kd")) {
    std::vector<double> kd;
    nh_global.getParam("damping_kd", kd);
    if (kd.size() == joint_kd_.size()) {
      for (size_t i = 0; i < kd.size(); ++i) {
        joint_kd_(i) = kd[i];
      }
      ROS_INFO("[DampingController] Loaded per-joint damping gain from config file");
    } else {
      ROS_WARN_STREAM("[DampingController] Damping gain size mismatch. Expected " 
                      << joint_kd_.size() << ", got " << kd.size() 
                      << ". Using default gain: 0.0");
    }
  } else {
    ROS_WARN("[DampingController] damping_kd not found in parameter server, using default: 0.0");
  }

  return true;
}

void DampingController::update(const ros::Time& time, const ros::Duration& period) {
  // Update state estimation
  updateStateEstimation(time, period);

  vector_t joint_pos = getJointPos();
  vector_t joint_vel = getJointVel();
  vector_t joint_eff = -joint_kd_.cwiseProduct(joint_vel);  // Damping: tau = -kd * vel
  vector_t joint_kp = vector_t::Zero(joint_pos.size());
  setJointCmd(joint_pos, joint_vel, joint_eff, joint_kp, joint_kd_);
}

void DampingController::starting(const ros::Time& time) {
  // Initialize state estimation
  current_observation_.state.setZero(legged_interface_->getCentroidalModelInfo().stateDim);
  updateStateEstimation(time, ros::Duration(0.002));

  ROS_INFO("[DampingController] Started - applying damping to joints");
}

}  // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::DampingController, controller_interface::ControllerBase)
