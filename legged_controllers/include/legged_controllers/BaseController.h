
#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <legged_common/hardware_interface/ContactSensorInterface.h>

#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_core/Types.h>
#include <ocs2_mpc/SystemObservation.h>

#include <legged_estimation/StateEstimateBase.h>
#include <legged_interface/LeggedInterface.h>

namespace legged {
using namespace ocs2;
using namespace legged_robot;

/**
 * @brief Base controller class that provides common functionality for all legged controllers.
 * 
 * This class implements:
 * 1. Reading robot state and writing robot commands interface for other controllers to use
 * 2. Updating model and state estimation for other controllers to access results
 */
class BaseController : public controller_interface::MultiInterfaceController<HybridJointInterface, 
                                                                              hardware_interface::ImuSensorInterface,
                                                                              ContactSensorInterface> {
 public:
  BaseController() = default;
  virtual ~BaseController() = default;

  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;

  // Virtual methods to be implemented by derived classes
  virtual void update(const ros::Time& time, const ros::Duration& period) = 0;
  virtual void starting(const ros::Time& time) {}
  virtual void stopping(const ros::Time& /*time*/) {}

 protected:
  /**
   * @brief Initialize hardware interfaces (joints, IMU, contacts)
   */
  virtual bool initHardwareInterface(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh);

  /**
   * @brief Initialize legged interface and state estimation
   */
  virtual bool initLeggedInterface(ros::NodeHandle& controller_nh);

  /**
   * @brief Setup state estimation (can be overridden by derived classes)
   */
  virtual void setupStateEstimate(const std::string& task_file, bool verbose);

  /**
   * @brief Update state estimation from hardware sensors
   */
  virtual void updateStateEstimation(const ros::Time& time, const ros::Duration& period);

  /**
   * @brief Get joint positions from hardware interfaces
   * @return Joint positions vector
   */
  vector_t getJointPos();

  /**
   * @brief Get joint velocities from hardware interfaces
   * @return Joint velocities vector
   */
  vector_t getJointVel();

  /**
   * @brief Get joint efforts (torques) from hardware interfaces
   * @return Joint efforts vector
   */
  vector_t getJointEff();

  /**
   * @brief Get IMU orientation quaternion
   * @return IMU quaternion
   */
  Eigen::Quaternion<scalar_t> getImuQuat();

  /**
   * @brief Get IMU angular velocity (gyroscope)
   * @return IMU angular velocity vector
   */
  vector3_t getImuGyro();

  /**
   * @brief Get IMU linear acceleration
   * @return IMU linear acceleration vector
   */
  vector3_t getImuAccel();

  /**
   * @brief Get contact state from sensors or mode
   * @return Contact flags
   */
  virtual contact_flag_t getContactState();

  /**
   * @brief Set joint commands to hardware interfaces
   * @param joint_pos Desired joint positions
   * @param joint_vel Desired joint velocities
   * @param torque Feedforward torques
   * @param kp Position gains vector (optional, defaults to zero vector)
   * @param kd Velocity gains vector (optional, defaults to zero vector)
   */
  void setJointCmd(const vector_t& joint_pos, const vector_t& joint_vel, 
                   const vector_t& torque, const vector_t& kp = vector_t(), const vector_t& kd = vector_t());

  // Hardware interfaces
  std::vector<HybridJointHandle> hybrid_joint_handles_;
  std::vector<ContactSensorHandle> contact_handles_;
  hardware_interface::ImuSensorHandle imu_sensor_handle_;

  // Legged interface and state estimation
  std::shared_ptr<LeggedInterface> legged_interface_;
  std::shared_ptr<PinocchioEndEffectorKinematics> ee_kinematics_ptr_;
  std::shared_ptr<StateEstimateBase> state_estimate_;
  std::shared_ptr<CentroidalModelRbdConversions> rbd_conversions_;

  // State estimation results
  SystemObservation current_observation_;
  vector_t measured_rbd_state_;

  // Joint names (default for quadruped robots)
  std::vector<std::string> joint_names_{"LF_HAA", "LF_HFE", "LF_KFE", 
                                       "LH_HAA", "LH_HFE", "LH_KFE",
                                       "RF_HAA", "RF_HFE", "RF_KFE", 
                                       "RH_HAA", "RH_HFE", "RH_KFE"};
};

}  // namespace legged
