
#pragma once

#include "legged_controllers/BaseController.h"

namespace legged {
using namespace ocs2;
using namespace legged_robot;

/**
 * @brief Damping controller
 */
class DampingController : public BaseController {
 public:
  DampingController() = default;
  ~DampingController() override = default;

  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void starting(const ros::Time& time) override;

 protected:
  // Damping gains for each joint
  vector_t joint_kd_;
};

}  // namespace legged
