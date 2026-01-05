
#pragma once

#include "legged_controllers/BaseController.h"

namespace legged {
using namespace ocs2;
using namespace legged_robot;

/**
 * @brief Zero torque controller - applies zero torque to all joints
 */
class ZeroTorqueController : public BaseController {
 public:
  ZeroTorqueController() = default;
  ~ZeroTorqueController() override = default;

  void update(const ros::Time& time, const ros::Duration& period) override;
  void starting(const ros::Time& time) override;
};

}  // namespace legged
