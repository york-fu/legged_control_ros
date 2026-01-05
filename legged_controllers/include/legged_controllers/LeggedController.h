//
// Created by qiayuan on 2022/6/24.
//

#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <legged_common/hardware_interface/ContactSensorInterface.h>

#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_legged_robot_ros/visualization/LeggedRobotVisualizer.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>

#include <legged_estimation/StateEstimateBase.h>
#include <legged_interface/LeggedInterface.h>
#include <legged_wbc/WbcBase.h>

#include "legged_controllers/BaseController.h"
#include "legged_controllers/SafetyChecker.h"
#include "legged_controllers/visualization/LeggedSelfCollisionVisualization.h"

namespace legged {
using namespace ocs2;
using namespace legged_robot;

class LeggedController : public BaseController {
 public:
  LeggedController() = default;
  ~LeggedController() override;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void starting(const ros::Time& time) override;
  void stopping(const ros::Time& /*time*/) override { mpc_running_ = false; }

 protected:
  virtual void setupMpc();
  virtual void setupMrt();
  
  // Override getContactState to use contact sensors when available
  contact_flag_t getContactState() override;

  // Whole Body Control
  std::shared_ptr<WbcBase> wbc_;
  std::shared_ptr<SafetyChecker> safety_checker_;

  // Nonlinear MPC
  std::shared_ptr<MPC_BASE> mpc_;
  std::shared_ptr<MPC_MRT_Interface> mpc_mrt_interface_;

  // Visualization
  std::shared_ptr<LeggedRobotVisualizer> robot_visualizer_;
  std::shared_ptr<LeggedSelfCollisionVisualization> self_collision_visualization_;
  ros::Publisher observation_publisher_;

 private:
  std::thread mpc_thread_;
  std::atomic_bool controller_running_{}, mpc_running_{};
  benchmark::RepeatedTimer mpc_timer_;
  benchmark::RepeatedTimer wbc_timer_;

  bool use_contact_sensor_{true};
};

class LeggedCheaterController : public LeggedController {
 protected:
  void setupStateEstimate(const std::string& task_file, bool verbose) override;
};

}  // namespace legged
