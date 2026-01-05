//
// Created by qiayuan on 2022/6/24.
//

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include "legged_controllers/LeggedController.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_legged_robot_ros/gait/GaitReceiver.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_sqp/SqpMpc.h>

#include <angles/angles.h>
#include <legged_estimation/FromTopiceEstimate.h>
#include <legged_estimation/LinearKalmanFilter.h>
#include <legged_wbc/HierarchicalWbc.h>
#include <legged_wbc/WeightedWbc.h>
#include <pluginlib/class_list_macros.hpp>

namespace legged {
bool LeggedController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) {
  // Initialize base controller (hardware interface and legged interface)
  if (!BaseController::init(robot_hw, controller_nh)) {
    return false;
  }

  // Check if contact sensors are available
  use_contact_sensor_ = (contact_handles_.size() == 4);  // 4 contact sensors for quadruped
  if (!use_contact_sensor_) {
    ROS_WARN("[LeggedController] Contact sensors not available, will use mode-based contact detection");
  }

  // Get task file for MPC setup
  std::string taskFile;
  controller_nh.getParam("/taskFile", taskFile);
  bool verbose = false;
  loadData::loadCppDataType(taskFile, "legged_robot_interface.verbose", verbose);

  // Setup MPC
  setupMpc();
  setupMrt();

  // Visualization
  ros::NodeHandle nh;
  CentroidalModelPinocchioMapping pinocchio_mapping(legged_interface_->getCentroidalModelInfo());
  robot_visualizer_ = std::make_shared<LeggedRobotVisualizer>(legged_interface_->getPinocchioInterface(),
                                                             legged_interface_->getCentroidalModelInfo(), *ee_kinematics_ptr_, nh);
  self_collision_visualization_.reset(new LeggedSelfCollisionVisualization(legged_interface_->getPinocchioInterface(),
                                                                         legged_interface_->getGeometryInterface(), pinocchio_mapping, nh));

  // Whole body control
  wbc_ = std::make_shared<WeightedWbc>(legged_interface_->getPinocchioInterface(), legged_interface_->getCentroidalModelInfo(),
                                       *ee_kinematics_ptr_);
  wbc_->loadTasksSetting(taskFile, verbose);

  // Safety Checker
  safety_checker_ = std::make_shared<SafetyChecker>(legged_interface_->getCentroidalModelInfo());

  return true;
}

void LeggedController::starting(const ros::Time& time) {
  // Initial state
  current_observation_.state.setZero(legged_interface_->getCentroidalModelInfo().stateDim);
  BaseController::updateStateEstimation(time, ros::Duration(0.002));
  current_observation_.input.setZero(legged_interface_->getCentroidalModelInfo().inputDim);
  current_observation_.mode = ModeNumber::STANCE;

  TargetTrajectories target_trajectories({current_observation_.time}, {current_observation_.state}, {current_observation_.input});

  // Set the first observation and command and wait for optimization to finish
  mpc_mrt_interface_->setCurrentObservation(current_observation_);
  mpc_mrt_interface_->getReferenceManager().setTargetTrajectories(target_trajectories);
  ROS_INFO_STREAM("Waiting for the initial policy ...");
  while (!mpc_mrt_interface_->initialPolicyReceived() && ros::ok()) {
    mpc_mrt_interface_->advanceMpc();
    ros::WallRate(legged_interface_->mpcSettings().mrtDesiredFrequency_).sleep();
  }
  ROS_INFO_STREAM("Initial policy has been received.");

  mpc_running_ = true;
}

void LeggedController::update(const ros::Time& time, const ros::Duration& period) {
  // State Estimate (from BaseController)
  BaseController::updateStateEstimation(time, period);

  // Update the current state of the system
  mpc_mrt_interface_->setCurrentObservation(current_observation_);

  // Load the latest MPC policy
  mpc_mrt_interface_->updatePolicy();

  // Evaluate the current policy
  vector_t optimized_state, optimized_input;
  size_t planned_mode = 0;  // The mode that is active at the time the policy is evaluated at.
  if (use_contact_sensor_)
  {
    mpc_mrt_interface_->evaluatePolicy(current_observation_.time, current_observation_.state, optimized_state, optimized_input, planned_mode);
  }
  else
  {
    if (mpc_mrt_interface_->isRolloutSet())
    {
      mpc_mrt_interface_->rolloutPolicy(current_observation_.time,
                                      current_observation_.state,
                                      period.toSec(),
                                      optimized_state,
                                      optimized_input,
                                      planned_mode);
    }
    else
    {
      mpc_mrt_interface_->evaluatePolicy(current_observation_.time,
                                       current_observation_.state,
                                       optimized_state,
                                       optimized_input,
                                       planned_mode);
    }
    current_observation_.mode = planned_mode;
  }

  // Whole body control
  current_observation_.input = optimized_input;

  wbc_timer_.startTimer();
  vector_t x = wbc_->update(optimized_state, optimized_input, measured_rbd_state_, planned_mode, period.toSec());
  wbc_timer_.endTimer();

  vector_t torque = x.tail(12);

  vector_t pos_des = centroidal_model::getJointAngles(optimized_state, legged_interface_->getCentroidalModelInfo());
  vector_t vel_des = centroidal_model::getJointVelocities(optimized_input, legged_interface_->getCentroidalModelInfo());

  // Safety check, if failed, stop the controller
  if (!safety_checker_->check(current_observation_, optimized_state, optimized_input)) {
    ROS_ERROR_STREAM("[Legged Controller] Safety check failed, stopping the controller.");
    stopRequest(time);
  }

  // Write commands using BaseController interface
  vector_t kp = vector_t::Zero(pos_des.size());
  vector_t kd = vector_t::Constant(pos_des.size(), 3.0);
  setJointCmd(pos_des, vel_des, torque, kp, kd);

  // Visualization
  robot_visualizer_->update(current_observation_, mpc_mrt_interface_->getPolicy(), mpc_mrt_interface_->getCommand());
  self_collision_visualization_->update(current_observation_);

  // Publish the observation. Only needed for the command interface
  observation_publisher_.publish(ros_msg_conversions::createObservationMsg(current_observation_));
}


LeggedController::~LeggedController() {
  controller_running_ = false;
  if (mpc_thread_.joinable()) {
    mpc_thread_.join();
  }
  std::cerr << "########################################################################";
  std::cerr << "\n### MPC Benchmarking";
  std::cerr << "\n###   Maximum : " << mpc_timer_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cerr << "\n###   Average : " << mpc_timer_.getAverageInMilliseconds() << "[ms]." << std::endl;
  std::cerr << "########################################################################";
  std::cerr << "\n### WBC Benchmarking";
  std::cerr << "\n###   Maximum : " << wbc_timer_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cerr << "\n###   Average : " << wbc_timer_.getAverageInMilliseconds() << "[ms].";
}


void LeggedController::setupMpc() {
  mpc_ = std::make_shared<SqpMpc>(legged_interface_->mpcSettings(), legged_interface_->sqpSettings(),
                                  legged_interface_->getOptimalControlProblem(), legged_interface_->getInitializer());
  rbd_conversions_ = std::make_shared<CentroidalModelRbdConversions>(legged_interface_->getPinocchioInterface(),
                                                                    legged_interface_->getCentroidalModelInfo());

  const std::string robot_name = "legged_robot";
  ros::NodeHandle nh;
  // Gait receiver
  auto gait_receiver_ptr =
      std::make_shared<GaitReceiver>(nh, legged_interface_->getSwitchedModelReferenceManagerPtr()->getGaitSchedule(), robot_name);
  // ROS ReferenceManager
  auto ros_reference_manager_ptr = std::make_shared<RosReferenceManager>(robot_name, legged_interface_->getReferenceManagerPtr());
  ros_reference_manager_ptr->subscribe(nh);
  mpc_->getSolverPtr()->addSynchronizedModule(gait_receiver_ptr);
  mpc_->getSolverPtr()->setReferenceManager(ros_reference_manager_ptr);
  observation_publisher_ = nh.advertise<ocs2_msgs::mpc_observation>(robot_name + "_mpc_observation", 1);
}

void LeggedController::setupMrt() {
  mpc_mrt_interface_ = std::make_shared<MPC_MRT_Interface>(*mpc_);
  mpc_mrt_interface_->initRollout(&legged_interface_->getRollout());
  mpc_timer_.reset();

  controller_running_ = true;
  mpc_thread_ = std::thread([&]() {
    while (controller_running_) {
      try {
        executeAndSleep(
            [&]() {
              if (mpc_running_) {
                mpc_timer_.startTimer();
                mpc_mrt_interface_->advanceMpc();
                mpc_timer_.endTimer();
              }
            },
            legged_interface_->mpcSettings().mrtDesiredFrequency_);
      } catch (const std::exception& e) {
        controller_running_ = false;
        ROS_ERROR_STREAM("[Ocs2 MPC thread] Error : " << e.what());
        stopRequest(ros::Time());
      }
    }
  });
  setThreadPriority(legged_interface_->sqpSettings().threadPriority, mpc_thread_);
}

contact_flag_t LeggedController::getContactState() {
  contact_flag_t contact_flag;
  if (use_contact_sensor_)
  {
    return BaseController::getContactState();
  }
  contact_flag = modeNumber2StanceLeg(current_observation_.mode);
  return contact_flag;
}


void LeggedCheaterController::setupStateEstimate(const std::string& /*task_file*/, bool /*verbose*/) {
  state_estimate_ = std::make_shared<FromTopicStateEstimate>(legged_interface_->getPinocchioInterface(),
                                                            legged_interface_->getCentroidalModelInfo(), *ee_kinematics_ptr_);
}

}  // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::LeggedController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(legged::LeggedCheaterController, controller_interface::ControllerBase)
