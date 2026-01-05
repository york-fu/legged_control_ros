#pragma once

#include "legged_controllers/BaseController.h"

namespace legged {
using namespace ocs2;
using namespace legged_robot;

/**
 * @brief Initial position controller - moves robot to default joint positions using linear interpolation
 */
class InitialPosController : public BaseController {
 public:
  InitialPosController() = default;
  ~InitialPosController() override = default;

  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void starting(const ros::Time& time) override;

 protected:
  // Default joint positions (target pose)
  vector_t def_joint_pos_;
  
  // Maximum joint velocity (rad/s)
  double max_joint_vel_{1.5};
  
  // Position and velocity gains
  vector_t joint_kp_;
  vector_t joint_kd_;
  
  // Initial joint positions when starting
  vector_t init_joint_pos_;
  
  // Start time for interpolation
  ros::Time start_time_;
  
  // Total duration for interpolation
  double duration_{0.0};
  
  // Flag to indicate if target is reached
  bool target_reached_{false};
};

}  // namespace legged
