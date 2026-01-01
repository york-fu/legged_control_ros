
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <string>
#include <vector>
#include <cmath>
#include <map>
#include <set>
#include <chrono>
#include <thread>
#include <atomic>
#include <csignal>
#include <cstdlib>

class JointControl
{
public:
  JointControl() : nh_("~"), nh_global_(), initialized_(false), start_time_(0.0), running_(false)
  {
    nh_.param<std::string>("joint_state_topic", joint_state_topic_, "/joint/states");
    nh_.param<std::string>("joint_cmd_topic", joint_cmd_topic_, "/joint/commands");
    nh_.param<std::string>("joint_params_topic", joint_params_topic_, "/joint/params");
    nh_.param<double>("ctrl_freq", ctrl_freq_, 500.0);
    nh_.param<std::vector<int>>("joint_indices", joint_indices_, std::vector<int>());
    nh_.param<double>("traj_amp", traj_amp_, 0.5);
    nh_.param<double>("traj_freq", traj_freq_, 1.0);

    omega_ = 2.0 * M_PI * traj_freq_;

    // Load kp and kd from root namespace (loaded by rosparam in launch file)
    nh_global_.param<std::vector<double>>("joint_kp", joint_kp_, std::vector<double>());
    nh_global_.param<std::vector<double>>("joint_kd", joint_kd_, std::vector<double>());

    if (joint_kp_.empty() || joint_kd_.empty())
    {
      ROS_WARN("joint_kp or joint_kd not found, using default values 100.0 and 5.0");
    }

    joint_state_sub_ = nh_.subscribe(joint_state_topic_, 1, &JointControl::jointStateCallback, this);

    joint_cmd_pub_ = nh_.advertise<sensor_msgs::JointState>(joint_cmd_topic_, 1);
    joint_params_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(joint_params_topic_, 1);

    ROS_INFO("JointControl node initialized");
    ROS_INFO("  Joint state topic: %s", joint_state_topic_.c_str());
    ROS_INFO("  Joint cmd topic: %s", joint_cmd_topic_.c_str());
    ROS_INFO("  Joint params topic: %s", joint_params_topic_.c_str());
    ROS_INFO("  Control frequency: %.1f Hz", ctrl_freq_);
    ROS_INFO("  Joint indices: [");
    for (size_t i = 0; i < joint_indices_.size(); ++i)
    {
      ROS_INFO("    %d", joint_indices_[i]);
    }
    ROS_INFO("  ]");
    ROS_INFO("  Trajectory amplitude: %.3f", traj_amp_);
    ROS_INFO("  Trajectory frequency: %.2f Hz", traj_freq_);
  }

  ~JointControl()
  {
    shutdown();
  }

  void startControlLoop()
  {
    running_ = true;
    control_thread_ = std::thread(&JointControl::controlLoop, this);
  }

  void shutdown()
  {
    running_ = false;
    if (control_thread_.joinable())
    {
      control_thread_.join();
    }

    if (!initialized_ || joint_names_.empty())
    {
      ROS_WARN("Cannot reset commands: not initialized or no joints");
      return;
    }

    if (!ros::ok())
    {
      ROS_WARN("ROS is not running, cannot reset commands");
      return;
    }

    ROS_INFO("Sending reset commands and parameters");

    sensor_msgs::JointState cmd_msg;
    cmd_msg.header.stamp = ros::Time::now();
    cmd_msg.name = joint_names_;

    std_msgs::Float64MultiArray params_msg;
    params_msg.data.resize(joint_names_.size() * 2, 0.0);

    const size_t num_joints = joint_names_.size();
    for (size_t i = 0; i < num_joints; ++i)
    {
      cmd_msg.position.push_back(0.0);
      cmd_msg.velocity.push_back(0.0);
      cmd_msg.effort.push_back(0.0);

      params_msg.data[i * 2] = 0.0;
      params_msg.data[i * 2 + 1] = 0.0;
    }

    ros::Time start_time = ros::Time::now();
    ros::Duration duration(0.2);
    while ((ros::Time::now() - start_time) < duration && ros::ok())
    {
      joint_cmd_pub_.publish(cmd_msg);
      joint_params_pub_.publish(params_msg);
      ros::spinOnce();
      ros::Duration(0.0001).sleep();
    }
    ROS_INFO("Reset commands sent successfully");
  }

  void jointStateCallback(const sensor_msgs::JointStateConstPtr &msg)
  {
    if (!initialized_)
    {
      joint_names_ = msg->name;
      const size_t num_joints = joint_names_.size();

      initial_positions_.clear();
      initial_positions_.reserve(num_joints);

      for (size_t i = 0; i < num_joints; ++i)
      {
        double pos = 0.0;
        if (i < msg->position.size())
        {
          pos = msg->position[i];
        }
        initial_positions_.push_back(pos);
      }

      controlled_joints_.clear();
      controlled_joints_.resize(num_joints, false);
      for (size_t j = 0; j < joint_indices_.size(); ++j)
      {
        int idx = joint_indices_[j];
        if (idx >= 0 && idx < static_cast<int>(num_joints))
        {
          controlled_joints_[idx] = true;
        }
      }

      joint_cmd_msg_.name = joint_names_;
      joint_cmd_msg_.position.resize(num_joints, 0.0);
      joint_cmd_msg_.velocity.resize(num_joints, 0.0);
      joint_cmd_msg_.effort.resize(num_joints, 0.0);

      joint_params_msg_.data.resize(num_joints * 2, 0.0);

      initialized_ = true;
      start_time_ = ros::Time::now().toSec();

      ROS_INFO("Received initial joint states with %zu joints", num_joints);
      for (size_t i = 0; i < num_joints; ++i)
      {
        ROS_INFO("  Joint[%zu]: %s = %.4f", i, joint_names_[i].c_str(),
                 initial_positions_[i]);
      }
    }
  }

private:
  void controlLoop()
  {
    using Clock = std::chrono::steady_clock;

    const std::chrono::duration<double> desired_duration(1.0 / ctrl_freq_);
    const auto dt = std::chrono::duration_cast<Clock::duration>(desired_duration);

    auto next_cycle_time = Clock::now() + dt;

    ROS_INFO("Control loop thread started, running at %.1f Hz", ctrl_freq_);

    while (running_ && ros::ok())
    {
      update();

      std::this_thread::sleep_until(next_cycle_time);
      next_cycle_time += dt;
    }
  }

  void update()
  {
    if (!initialized_ || joint_names_.empty())
    {
      return;
    }

    const double current_time = ros::Time::now().toSec();
    const double elapsed_time = current_time - start_time_;

    joint_cmd_msg_.header.stamp = ros::Time::now();

    const double cos_val = -std::cos(omega_ * elapsed_time);
    const double sin_val = -std::sin(omega_ * elapsed_time);
    const double vel_scale = -traj_amp_ * omega_;

    const size_t num_joints = joint_names_.size();
    for (size_t i = 0; i < num_joints; ++i)
    {
      const double initial_pos = initial_positions_[i];

      if (controlled_joints_[i])
      {
        joint_cmd_msg_.position[i] = initial_pos + traj_amp_ * (cos_val + 1.0);
        joint_cmd_msg_.velocity[i] = vel_scale * sin_val;
        joint_cmd_msg_.effort[i] = 0.0;
      }
      else
      {
        joint_cmd_msg_.position[i] = initial_pos;
        joint_cmd_msg_.velocity[i] = 0.0;
        joint_cmd_msg_.effort[i] = 0.0;
      }

      const double kp = (i < joint_kp_.size()) ? joint_kp_[i] : 100.0;
      const double kd = (i < joint_kd_.size()) ? joint_kd_[i] : 2.0;

      joint_params_msg_.data[i * 2] = kp;
      joint_params_msg_.data[i * 2 + 1] = kd;
    }

    joint_cmd_pub_.publish(joint_cmd_msg_);
    joint_params_pub_.publish(joint_params_msg_);
  }
  ros::NodeHandle nh_;
  ros::NodeHandle nh_global_;
  ros::Subscriber joint_state_sub_;
  ros::Publisher joint_cmd_pub_;
  ros::Publisher joint_params_pub_;

  std::string joint_state_topic_;
  std::string joint_cmd_topic_;
  std::string joint_params_topic_;
  double ctrl_freq_;
  std::vector<int> joint_indices_;
  double traj_amp_;
  double traj_freq_;

  bool initialized_;
  double start_time_;
  std::vector<std::string> joint_names_;
  std::vector<double> initial_positions_;
  std::vector<bool> controlled_joints_;

  std::vector<double> joint_kp_;
  std::vector<double> joint_kd_;

  sensor_msgs::JointState joint_cmd_msg_;
  std_msgs::Float64MultiArray joint_params_msg_;

  double omega_;

  std::thread control_thread_;
  std::atomic<bool> running_;
};

static JointControl *g_joint_ctrl_ptr = nullptr;

void signalHandler(int sig)
{
  ROS_INFO("Received signal %d, exit", sig);
  if (g_joint_ctrl_ptr != nullptr)
  {
    g_joint_ctrl_ptr->shutdown();
  }
  ros::shutdown();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joint_control", ros::init_options::NoSigintHandler);
  ROS_INFO("JointControl node started");

  signal(SIGINT, signalHandler);
  signal(SIGTERM, signalHandler);

  JointControl joint_ctrl;
  g_joint_ctrl_ptr = &joint_ctrl;
  joint_ctrl.startControlLoop();

  ros::spin();

  g_joint_ctrl_ptr = nullptr;

  return 0;
}
