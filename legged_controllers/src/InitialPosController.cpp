#include "legged_controllers/InitialPosController.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <pluginlib/class_list_macros.hpp>

namespace legged {

bool InitialPosController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) {
  // Initialize base controller
  if (!BaseController::init(robot_hw, controller_nh)) {
    return false;
  }

  // Load parameters from config file (loaded by launch file to root namespace)
  ros::NodeHandle nh_global;
  
  // Load maximum joint velocity from config file
  nh_global.param("max_joint_vel", max_joint_vel_, 1.5);

  // Load default joint positions from config file
  def_joint_pos_.resize(hybrid_joint_handles_.size());
  if (nh_global.hasParam("def_joint_pos")) {
    std::vector<double> def_pos;
    nh_global.getParam("def_joint_pos", def_pos);
    if (def_pos.size() == def_joint_pos_.size()) {
      for (size_t i = 0; i < def_pos.size(); ++i) {
        def_joint_pos_(i) = def_pos[i];
      }
      ROS_INFO("[InitialPosController] Loaded default joint positions from config file");
    } else {
      ROS_WARN_STREAM("[InitialPosController] Default positions size mismatch. Expected " 
                      << def_joint_pos_.size() << ", got " << def_pos.size() 
                      << ". Will use current positions as default.");
    }
  } else {
    ROS_WARN("[InitialPosController] def_joint_pos not found in parameter server. Will use current positions as default.");
  }

  // Load joint gains (kp and kd) from config file
  joint_kp_.resize(hybrid_joint_handles_.size());
  joint_kd_.resize(hybrid_joint_handles_.size());
  joint_kp_.setConstant(0.0);
  joint_kd_.setConstant(0.0);
  
  if (nh_global.hasParam("joint_kp")) {
    std::vector<double> kp;
    nh_global.getParam("joint_kp", kp);
    if (kp.size() == joint_kp_.size()) {
      for (size_t i = 0; i < kp.size(); ++i) {
        joint_kp_(i) = kp[i];
      }
      ROS_INFO("[InitialPosController] Loaded joint_kp from config file");
    } else {
      ROS_WARN_STREAM("[InitialPosController] joint_kp size mismatch. Expected " 
                      << joint_kp_.size() << ", got " << kp.size() 
                      << ". Using default: 0.0");
    }
  } else {
    ROS_WARN("[InitialPosController] joint_kp not found in parameter server, using default: 0.0");
  }
  
  if (nh_global.hasParam("joint_kd")) {
    std::vector<double> kd;
    nh_global.getParam("joint_kd", kd);
    if (kd.size() == joint_kd_.size()) {
      for (size_t i = 0; i < kd.size(); ++i) {
        joint_kd_(i) = kd[i];
      }
      ROS_INFO("[InitialPosController] Loaded joint_kd from config file");
    } else {
      ROS_WARN_STREAM("[InitialPosController] joint_kd size mismatch. Expected " 
                      << joint_kd_.size() << ", got " << kd.size() 
                      << ". Using default: 0.0");
    }
  } else {
    ROS_WARN("[InitialPosController] joint_kd not found in parameter server, using default: 0.0");
  }

  init_joint_pos_.resize(hybrid_joint_handles_.size());
  target_reached_ = false;
  duration_ = 0.0;

  return true;
}

void InitialPosController::update(const ros::Time& time, const ros::Duration& period) {
  // Update state estimation
  updateStateEstimation(time, period);

  // Get current joint positions
  vector_t joint_pos = getJointPos();

  // Initialize default positions if not set (use current positions)
  if (def_joint_pos_.size() != joint_pos.size()) {
    def_joint_pos_ = joint_pos;
  }

  // Check if target is reached
  vector_t position_error = def_joint_pos_ - joint_pos;
  double max_error = position_error.cwiseAbs().maxCoeff();
  if (max_error < 0.01) {  // 0.01 rad threshold
    if (!target_reached_) {
      target_reached_ = true;
      ROS_INFO("[InitialPosController] Target position reached!");
    }
    // Maintain position with kp and kd gains
    vector_t joint_vel = vector_t::Zero(joint_pos.size());
    vector_t joint_eff = vector_t::Zero(joint_pos.size());
    setJointCmd(def_joint_pos_, joint_vel, joint_eff, joint_kp_, joint_kd_);
    return;
  }

  // Calculate elapsed time since start
  double elapsed_time = (time - start_time_).toSec();

  // Linear interpolation
  double t = (duration_ > 0.0) ? std::min(elapsed_time / duration_, 1.0) : 1.0;
  
  // Interpolate joint positions
  vector_t desired_joint_pos = init_joint_pos_ + t * (def_joint_pos_ - init_joint_pos_);
  
  // Calculate desired velocity for linear interpolation
  // Since duration is calculated based on max error / max_joint_vel_,
  // the joint with max error will move at max_joint_vel_, others will be slower
  vector_t desired_joint_vel = vector_t::Zero(joint_pos.size());
  if (duration_ > 0.0 && t < 1.0) {
    vector_t position_diff = def_joint_pos_ - init_joint_pos_;
    desired_joint_vel = position_diff / duration_;
    // Ensure no joint exceeds max_joint_vel_ (shouldn't happen if duration is correct, but check anyway)
    for (size_t i = 0; i < desired_joint_vel.size(); ++i) {
      if (std::abs(desired_joint_vel(i)) > max_joint_vel_) {
        desired_joint_vel(i) = (desired_joint_vel(i) > 0) ? max_joint_vel_ : -max_joint_vel_;
      }
    }
  }

  // Zero effort (position control with kp and kd gains)
  vector_t joint_eff = vector_t::Zero(joint_pos.size());

  // Write commands with kp and kd gains
  setJointCmd(desired_joint_pos, desired_joint_vel, joint_eff, joint_kp_, joint_kd_);
}

void InitialPosController::starting(const ros::Time& time) {
  // Initialize state estimation
  current_observation_.state.setZero(legged_interface_->getCentroidalModelInfo().stateDim);
  updateStateEstimation(time, ros::Duration(0.002));
  
  // Get current joint positions as initial positions
  init_joint_pos_ = getJointPos();

  // Initialize default positions if not set
  if (def_joint_pos_.size() != init_joint_pos_.size()) {
    def_joint_pos_ = init_joint_pos_;
  }

  // Calculate duration based on maximum position error and max_joint_vel_
  vector_t position_error = def_joint_pos_ - init_joint_pos_;
  double max_error = position_error.cwiseAbs().maxCoeff();
  duration_ = (max_joint_vel_ > 0.0) ? (max_error / max_joint_vel_) : 0.0;

  start_time_ = time;
  target_reached_ = false;

  ROS_INFO_STREAM("[InitialPosController] Started - moving to default position. Duration: " << duration_ << "s");
}

}  // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::InitialPosController, controller_interface::ControllerBase)
