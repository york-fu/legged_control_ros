
#pragma once

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64MultiArray.h>
#include <urdf/model.h>
#include <realtime_tools/realtime_buffer.h>
#include <XmlRpcValue.h>

#include <string>
#include <vector>
#include <unordered_map>

namespace robot
{
  struct HybridJointCommand
  {
    ros::Time stamp;
    double pos{}, vel{}, kp{}, kd{}, eff{};
  };

  struct JointData
  {
    gazebo::physics::JointPtr joint;
    double position{};
    double velocity{};
    double effort{};
    double pos_des{};
    double vel_des{};
    double tau_ff{};
    double kp{};
    double kd{};
  };

  struct ImuData
  {
    gazebo::physics::LinkPtr link_ptr;
    double ori[4];
    double ori_cov[9];
    double angular_vel[3];
    double angular_vel_cov[9];
    double linear_acc[3];
    double linear_acc_cov[9];
    std::string frame_id;
  };

  class RobotGazeboSim : public gazebo::ModelPlugin
  {
  public:
    RobotGazeboSim();
    ~RobotGazeboSim() override;

    void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf) override;

  private:
    void onUpdate();
    void parseImu(XmlRpc::XmlRpcValue &imu_datas);
    void jointCmdCallback(const sensor_msgs::JointStateConstPtr &msg);
    void jointParamsCallback(const std_msgs::Float64MultiArrayConstPtr &msg);
    void publishJointStates();
    void publishImuData();
    void publishContactStates();
    void updateJoints();
    void parseContacts(XmlRpc::XmlRpcValue &contact_names);
    void updateContacts();

    gazebo::physics::ModelPtr model_;
    gazebo::event::ConnectionPtr update_connection_;
    gazebo::common::Time last_update_time_;

    ros::NodeHandle nh_;
    ros::Publisher joint_state_pub_;
    ros::Publisher imu_pub_;
    ros::Publisher contact_state_pub_;
    ros::Subscriber joint_cmd_sub_;
    ros::Subscriber joint_params_sub_;

    std::string joint_state_topic_;
    std::string imu_topic_;
    std::string contact_state_topic_;
    std::string joint_cmd_topic_;
    std::string joint_params_topic_;
    std::string robot_namespace_;

    std::vector<JointData> joints_;
    std::vector<ImuData> imu_datas_;
    std::unordered_map<std::string, size_t> joint_name_to_index_;

    std::unordered_map<std::string, realtime_tools::RealtimeBuffer<HybridJointCommand>> joint_cmd_buffers_;
    std::unordered_map<std::string, realtime_tools::RealtimeBuffer<std::pair<double, double>>> joint_params_buffers_;

    gazebo::physics::ContactManager *contact_manager_{};
    std::unordered_map<std::string, bool> name2contact_;
    std::vector<std::string> contact_names_;

    double delay_{};
    double update_rate_{1000.0};
    gazebo::common::Time update_period_;
  };

} // namespace robot
