
#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include "legged_controllers/BaseController.h"

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

#include <angles/angles.h>
#include <legged_estimation/LinearKalmanFilter.h>

namespace legged {

bool BaseController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) {
  // Initialize hardware interface
  if (!initHardwareInterface(robot_hw, controller_nh)) {
    ROS_ERROR("[BaseController] Failed to initialize hardware interface");
    return false;
  }

  // Initialize legged interface
  if (!initLeggedInterface(controller_nh)) {
    ROS_ERROR("[BaseController] Failed to initialize legged interface");
    return false;
  }

  return true;
}

bool BaseController::initHardwareInterface(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) {
  // Get joint names from parameter server or use defaults
  if (controller_nh.hasParam("joints")) {
    controller_nh.getParam("joints", joint_names_);
  }

  // Initialize joint handles
  auto* hybrid_joint_interface = robot_hw->get<HybridJointInterface>();
  hybrid_joint_handles_.clear();
  for (const auto& joint_name : joint_names_) {
    try {
      hybrid_joint_handles_.push_back(hybrid_joint_interface->getHandle(joint_name));
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM("[BaseController] Could not find joint '" << joint_name << "': " << e.what());
      return false;
    }
  }

  // Initialize contact sensor handles (for LeggedController use)
  if (legged_interface_) {
    auto* contact_interface = robot_hw->get<ContactSensorInterface>();
    contact_handles_.clear();
    for (const auto& name : legged_interface_->modelSettings().contactNames3DoF) {
      try {
        contact_handles_.push_back(contact_interface->getHandle(name));
      } catch (const hardware_interface::HardwareInterfaceException& e) {
        ROS_WARN_STREAM("[BaseController] Could not find contact sensor '" << name << "': " << e.what());
      }
    }
  }

  // Initialize IMU handle
  try {
    imu_sensor_handle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle("base_imu");
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM("[BaseController] Could not find IMU 'base_imu': " << e.what());
    return false;
  }

  return true;
}

bool BaseController::initLeggedInterface(ros::NodeHandle& controller_nh) {
  // Get configuration files
  std::string urdf_file;
  std::string task_file;
  std::string reference_file;
  if (!controller_nh.getParam("/urdfFile", urdf_file)) {
    ROS_ERROR("[BaseController] Failed to get /urdfFile parameter");
    return false;
  }
  if (!controller_nh.getParam("/taskFile", task_file)) {
    ROS_ERROR("[BaseController] Failed to get /taskFile parameter");
    return false;
  }
  if (!controller_nh.getParam("/referenceFile", reference_file)) {
    ROS_ERROR("[BaseController] Failed to get /referenceFile parameter");
    return false;
  }

  bool verbose = false;
  loadData::loadCppDataType(task_file, "legged_robot_interface.verbose", verbose);

  // Initialize legged interface
  legged_interface_ = std::make_shared<LeggedInterface>(task_file, urdf_file, reference_file);
  legged_interface_->setupOptimalControlProblem(task_file, urdf_file, reference_file, verbose);

  // Setup end effector kinematics
  CentroidalModelPinocchioMapping pinocchio_mapping(legged_interface_->getCentroidalModelInfo());
  ee_kinematics_ptr_ = std::make_shared<PinocchioEndEffectorKinematics>(
      legged_interface_->getPinocchioInterface(), pinocchio_mapping,
      legged_interface_->modelSettings().contactNames3DoF);

  // Setup state estimation
  setupStateEstimate(task_file, verbose);

  // Setup RBD conversions
  rbd_conversions_ = std::make_shared<CentroidalModelRbdConversions>(
      legged_interface_->getPinocchioInterface(),
      legged_interface_->getCentroidalModelInfo());

  // Initialize observation
  current_observation_.time = 0;
  current_observation_.state.setZero(legged_interface_->getCentroidalModelInfo().stateDim);
  current_observation_.input.setZero(legged_interface_->getCentroidalModelInfo().inputDim);
  current_observation_.mode = ModeNumber::STANCE;
  measured_rbd_state_.setZero(legged_interface_->getCentroidalModelInfo().generalizedCoordinatesNum);

  return true;
}

void BaseController::setupStateEstimate(const std::string& task_file, bool verbose) {
  state_estimate_ = std::make_shared<KalmanFilterEstimate>(
      legged_interface_->getPinocchioInterface(),
      legged_interface_->getCentroidalModelInfo(), *ee_kinematics_ptr_);
  dynamic_cast<KalmanFilterEstimate&>(*state_estimate_).loadSettings(task_file, verbose);
}

void BaseController::updateStateEstimation(const ros::Time& time, const ros::Duration& period) {
  vector_t joint_pos = getJointPos();
  vector_t joint_vel = getJointVel();
  contact_flag_t contact_flag = getContactState();
  Eigen::Quaternion<scalar_t> quat = getImuQuat();
  vector3_t angular_vel = getImuGyro();
  vector3_t linear_accel = getImuAccel();
  matrix3_t orientation_covariance, angular_vel_covariance, linear_accel_covariance;

  // Get IMU covariance
  for (size_t i = 0; i < 9; ++i) {
    orientation_covariance(i) = imu_sensor_handle_.getOrientationCovariance()[i];
    angular_vel_covariance(i) = imu_sensor_handle_.getAngularVelocityCovariance()[i];
    linear_accel_covariance(i) = imu_sensor_handle_.getLinearAccelerationCovariance()[i];
  }

  // Update state estimation
  state_estimate_->updateJointStates(joint_pos, joint_vel);
  state_estimate_->updateContact(contact_flag);
  state_estimate_->updateImu(quat, angular_vel, linear_accel, orientation_covariance, 
                           angular_vel_covariance, linear_accel_covariance);
  measured_rbd_state_ = state_estimate_->update(time, period);
  
  // Update observation
  current_observation_.time += period.toSec();
  scalar_t yaw_last = current_observation_.state(9);
  current_observation_.state = rbd_conversions_->computeCentroidalStateFromRbdModel(measured_rbd_state_);
  current_observation_.state(9) = yaw_last + angles::shortest_angular_distance(yaw_last, current_observation_.state(9));
  current_observation_.mode = state_estimate_->getMode();
}

vector_t BaseController::getJointPos() {
  vector_t joint_pos(hybrid_joint_handles_.size());
  for (size_t i = 0; i < hybrid_joint_handles_.size(); ++i) {
    joint_pos(i) = hybrid_joint_handles_[i].getPosition();
  }
  return joint_pos;
}

vector_t BaseController::getJointVel() {
  vector_t joint_vel(hybrid_joint_handles_.size());
  for (size_t i = 0; i < hybrid_joint_handles_.size(); ++i) {
    joint_vel(i) = hybrid_joint_handles_[i].getVelocity();
  }
  return joint_vel;
}

vector_t BaseController::getJointEff() {
  vector_t joint_eff(hybrid_joint_handles_.size());
  for (size_t i = 0; i < hybrid_joint_handles_.size(); ++i) {
    joint_eff(i) = hybrid_joint_handles_[i].getEffort();
  }
  return joint_eff;
}

Eigen::Quaternion<scalar_t> BaseController::getImuQuat() {
  Eigen::Quaternion<scalar_t> quat;
  for (size_t i = 0; i < 4; ++i) {
    quat.coeffs()(i) = imu_sensor_handle_.getOrientation()[i];
  }
  return quat;
}

vector3_t BaseController::getImuGyro() {
  vector3_t angular_vel;
  for (size_t i = 0; i < 3; ++i) {
    angular_vel(i) = imu_sensor_handle_.getAngularVelocity()[i];
  }
  return angular_vel;
}

vector3_t BaseController::getImuAccel() {
  vector3_t linear_accel;
  for (size_t i = 0; i < 3; ++i) {
    linear_accel(i) = imu_sensor_handle_.getLinearAcceleration()[i];
  }
  return linear_accel;
}

contact_flag_t BaseController::getContactState() {
  contact_flag_t contact_flag;
  for (size_t i = 0; i < contact_handles_.size(); ++i) {
    contact_flag[i] = contact_handles_[i].isContact();
  }
  return contact_flag;
}

void BaseController::setJointCmd(const vector_t& joint_pos, const vector_t& joint_vel,
                                  const vector_t& torque, const vector_t& kp, const vector_t& kd) {
  size_t num_joints = hybrid_joint_handles_.size();
  if (joint_pos.size() != num_joints || joint_vel.size() != num_joints || torque.size() != num_joints) {
    ROS_ERROR_STREAM("[BaseController] Command size mismatch. Expected " << num_joints 
                     << ", got pos:" << joint_pos.size() 
                     << ", vel:" << joint_vel.size() 
                     << ", torque:" << torque.size());
    return;
  }

  // Check kp and kd sizes
  bool kp_valid = (kp.size() == 0 || kp.size() == num_joints);
  bool kd_valid = (kd.size() == 0 || kd.size() == num_joints);
  
  if (!kp_valid || !kd_valid) {
    ROS_ERROR_STREAM("[BaseController] Gain size mismatch. Expected " << num_joints 
                     << " or 0, got kp:" << kp.size() 
                     << ", kd:" << kd.size());
    return;
  }

  // Use scalar values if vectors are empty, otherwise use per-joint values
  for (size_t j = 0; j < num_joints; ++j) {
    double kp_val = (kp.size() == 0) ? 0.0 : kp(j);
    double kd_val = (kd.size() == 0) ? 0.0 : kd(j);
    hybrid_joint_handles_[j].setCommand(joint_pos(j), joint_vel(j), kp_val, kd_val, torque(j));
  }
}

}  // namespace legged
