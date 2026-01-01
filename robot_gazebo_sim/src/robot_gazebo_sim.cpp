
#include "robot_gazebo_sim/robot_gazebo_sim.h"
#include <gazebo/common/Time.hh>
#include <gazebo/physics/physics.hh>
#include <angles/angles.h>
#include <urdf_parser/urdf_parser.h>
#include <urdf/model.h>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <XmlRpcValue.h>

namespace robot
{
  RobotGazeboSim::RobotGazeboSim() : model_(nullptr), update_connection_(nullptr)
  {
  }

  RobotGazeboSim::~RobotGazeboSim()
  {
    // Event connection will be automatically disconnected when plugin is unloaded
    update_connection_.reset();
  }

  void RobotGazeboSim::Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf)
  {
    model_ = parent;

    if (!ros::isInitialized())
    {
      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, "robot_gazebo_sim", ros::init_options::NoSigintHandler);
    }

    // Get robot namespace from SDF or use default
    if (sdf->HasElement("robotNamespace"))
    {
      robot_namespace_ = sdf->Get<std::string>("robotNamespace");
    }
    else
    {
      robot_namespace_ = model_->GetName();
    }

    nh_ = ros::NodeHandle(robot_namespace_);

    nh_.param<std::string>("joint_state_topic", joint_state_topic_, "/state/joint");
    nh_.param<std::string>("imu_topic", imu_topic_, "/state/imu");
    nh_.param<std::string>("contact_state_topic", contact_state_topic_, "/state/contact");
    nh_.param<std::string>("joint_cmd_topic", joint_cmd_topic_, "/desire/joint/commands");
    nh_.param<std::string>("joint_params_topic", joint_params_topic_, "/desire/joint/params");

    if (sdf->HasElement("jointStateTopic"))
    {
      joint_state_topic_ = sdf->Get<std::string>("jointStateTopic");
    }
    if (sdf->HasElement("imuTopic"))
    {
      imu_topic_ = sdf->Get<std::string>("imuTopic");
    }
    if (sdf->HasElement("jointCmdTopic"))
    {
      joint_cmd_topic_ = sdf->Get<std::string>("jointCmdTopic");
    }
    if (sdf->HasElement("jointParamsTopic"))
    {
      joint_params_topic_ = sdf->Get<std::string>("jointParamsTopic");
    }
    if (sdf->HasElement("contactStateTopic"))
    {
      contact_state_topic_ = sdf->Get<std::string>("contactStateTopic");
    }

    if (sdf->HasElement("updateRate"))
    {
      update_rate_ = sdf->Get<double>("updateRate");
    }
    else
    {
      nh_.param<double>("update_rate", update_rate_, 1000.0);
    }
    update_period_ = gazebo::common::Time(1.0 / update_rate_);

    if (sdf->HasElement("delay"))
    {
      delay_ = sdf->Get<double>("delay");
    }
    else
    {
      nh_.param<double>("gazebo/delay", delay_, 0.0);
    }

    urdf::Model urdf_model;
    std::string urdf_param_name = "legged_robot_description";
    if (sdf->HasElement("robotParam"))
    {
      urdf_param_name = sdf->Get<std::string>("robotParam");
    }

    std::string urdf_string;
    bool urdf_loaded = false;
    if (nh_.getParam(urdf_param_name, urdf_string))
    {
      if (urdf_model.initString(urdf_string))
      {
        urdf_loaded = true;
        ROS_INFO_STREAM("Loaded URDF model from parameter '" << urdf_param_name << "'");
      }
      else
      {
        ROS_WARN_STREAM("Failed to parse URDF string from parameter '" << urdf_param_name << "'");
      }
    }
    else
    {
      if (urdf_model.initParam(urdf_param_name))
      {
        urdf_loaded = true;
        ROS_INFO_STREAM("Loaded URDF model from parameter '" << urdf_param_name << "'");
      }
      else
      {
        ROS_WARN_STREAM("Failed to load URDF model from parameter '" << urdf_param_name << "', will skip joint type checking");
      }
    }

    std::vector<double> joint_kp, joint_kd, def_joint_pos;
    nh_.param<std::vector<double>>("joint_kp", joint_kp, std::vector<double>());
    nh_.param<std::vector<double>>("joint_kd", joint_kd, std::vector<double>());
    nh_.param<std::vector<double>>("def_joint_pos", def_joint_pos, std::vector<double>());

    if (joint_kp.empty() || joint_kd.empty() || def_joint_pos.empty())
    {
      ROS_WARN("Joint parameters (joint_kp, joint_kd, def_joint_pos) not found or empty. Using default values.");
    }

    gazebo::physics::Joint_V joints = model_->GetJoints();
    for (auto &joint : joints)
    {
      std::string joint_name = joint->GetName();

      bool is_revolute = false;
      if (urdf_loaded)
      {
        auto it = urdf_model.joints_.find(joint_name);
        if (it != urdf_model.joints_.end())
        {
          urdf::JointConstSharedPtr urdf_joint = it->second;
          if (urdf_joint->type == urdf::Joint::REVOLUTE || urdf_joint->type == urdf::Joint::CONTINUOUS)
          {
            is_revolute = true;
          }
        }
        else
        {
          ROS_DEBUG_STREAM("Joint '" << joint_name << "' not found in URDF model, skipping");
          continue;
        }
      }
      else
      {
        ROS_WARN_STREAM_THROTTLE(1.0, "URDF model not loaded, cannot check joint types. "
                                          << "Skipping joint: " << joint_name);
        continue;
      }

      if (!is_revolute)
      {
        ROS_DEBUG_STREAM("Skipping non-revolute joint: " << joint_name);
        continue;
      }

      JointData joint_data;
      joint_data.joint = joint;
      joint_data.position = 0.0;
      joint_data.velocity = 0.0;
      joint_data.effort = 0.0;
      joint_data.pos_des = 0.0;
      joint_data.vel_des = 0.0;
      joint_data.tau_ff = 0.0;
      joint_data.kp = 0.0;
      joint_data.kd = 0.0;

      joints_.push_back(joint_data);
      joint_name_to_index_[joint_name] = joints_.size() - 1;
      ROS_INFO_STREAM("Added revolute joint: " << joint_name);

      joint_cmd_buffers_[joint_name] = realtime_tools::RealtimeBuffer<HybridJointCommand>();
      joint_params_buffers_[joint_name] = realtime_tools::RealtimeBuffer<std::pair<double, double>>();

      size_t joint_index = joints_.size() - 1;

      HybridJointCommand init_cmd;
      init_cmd.stamp = ros::Time::now();
      init_cmd.pos = 0.0;
      init_cmd.vel = 0.0;
      init_cmd.eff = 0.0;
      init_cmd.kp = 0.0;
      init_cmd.kd = 0.0;

      if (joint_index < def_joint_pos.size())
      {
        init_cmd.pos = def_joint_pos[joint_index];
      }
      if (joint_index < joint_kp.size())
      {
        init_cmd.kp = joint_kp[joint_index];
      }
      if (joint_index < joint_kd.size())
      {
        init_cmd.kd = joint_kd[joint_index];
      }

      joint_cmd_buffers_[joint_name].writeFromNonRT(init_cmd);
      joint_params_buffers_[joint_name].writeFromNonRT(std::pair<double, double>(init_cmd.kp, init_cmd.kd));

      ROS_INFO_STREAM("  initial command with pos: " << init_cmd.pos << ", kp: " << init_cmd.kp << ", kd: " << init_cmd.kd);
    }

    ROS_INFO_STREAM("Initialized " << joints_.size() << " revolute joints");

    // Parse IMU configuration
    XmlRpc::XmlRpcValue xml_rpc_value;
    if (nh_.getParam("gazebo/imus", xml_rpc_value))
    {
      parseImu(xml_rpc_value);
    }
    else
    {
      ROS_WARN("No IMU specified in parameter 'gazebo/imus'");
    }

    if (nh_.getParam("gazebo/contacts", xml_rpc_value))
    {
      parseContacts(xml_rpc_value);
    }
    else
    {
      ROS_WARN("No contacts specified in parameter 'gazebo/contacts'");
    }

    contact_manager_ = model_->GetWorld()->Physics()->GetContactManager();
    if (contact_manager_ != nullptr)
    {
      contact_manager_->SetNeverDropContacts(true); // NOTE: If false, we need to select view->contacts in gazebo GUI to
                                                    // avoid returning nothing when calling ContactManager::GetContacts()
    }

    joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>(joint_state_topic_, 1);
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>(imu_topic_, 1);
    contact_state_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(contact_state_topic_, 1);
    joint_cmd_sub_ = nh_.subscribe<sensor_msgs::JointState>(joint_cmd_topic_, 1, &RobotGazeboSim::jointCmdCallback, this);
    joint_params_sub_ = nh_.subscribe<std_msgs::Float64MultiArray>(joint_params_topic_, 1, &RobotGazeboSim::jointParamsCallback, this);

    ROS_INFO_STREAM("RobotGazeboSim plugin loaded for model: " << model_->GetName());
    ROS_INFO_STREAM("  Robot namespace: " << robot_namespace_);
    ROS_INFO_STREAM("  Joint state topic: " << joint_state_topic_);
    ROS_INFO_STREAM("  IMU topic: " << imu_topic_);
    ROS_INFO_STREAM("  Contact state topic: " << contact_state_topic_);
    ROS_INFO_STREAM("  Joint command topic: " << joint_cmd_topic_);
    ROS_INFO_STREAM("  Joint params topic: " << joint_params_topic_);
    ROS_INFO_STREAM("  Update rate: " << update_rate_ << " Hz");
    // ROS_INFO_STREAM("  Gazebo delay param: " << delay_ << " s");

    last_update_time_ = model_->GetWorld()->SimTime();

    // Listen to the update event
    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        boost::bind(&RobotGazeboSim::onUpdate, this));
  }

  void RobotGazeboSim::onUpdate()
  {
    gazebo::common::Time current_time = model_->GetWorld()->SimTime();
    gazebo::common::Time step_time = current_time - last_update_time_;

    // Throttle update rate
    // if (step_time < update_period_) {
    //   return;
    // }

    updateJoints();

    // Update IMU data
    for (auto &imu : imu_datas_)
    {
      ignition::math::Pose3d pose = imu.link_ptr->WorldPose();
      imu.ori[0] = pose.Rot().X();
      imu.ori[1] = pose.Rot().Y();
      imu.ori[2] = pose.Rot().Z();
      imu.ori[3] = pose.Rot().W();

      ignition::math::Vector3d rate = imu.link_ptr->RelativeAngularVel();
      imu.angular_vel[0] = rate.X();
      imu.angular_vel[1] = rate.Y();
      imu.angular_vel[2] = rate.Z();

      ignition::math::Vector3d gravity = {0., 0., -9.81};
      ignition::math::Vector3d accel = imu.link_ptr->RelativeLinearAccel() -
                                       pose.Rot().RotateVectorReverse(gravity);
      imu.linear_acc[0] = accel.X();
      imu.linear_acc[1] = accel.Y();
      imu.linear_acc[2] = accel.Z();
    }

    // Update contact states
    updateContacts();

    last_update_time_ = current_time;

    publishJointStates();
    publishImuData();
    publishContactStates();
  }

  void RobotGazeboSim::updateJoints()
  {
    for (auto &joint_data : joints_)
    {
      joint_data.position = joint_data.joint->Position(0);
      joint_data.velocity = joint_data.joint->GetVelocity(0);
      joint_data.effort = joint_data.joint->GetForce(0);

      auto cmd_buffer_it = joint_cmd_buffers_.find(joint_data.joint->GetName());
      if (cmd_buffer_it != joint_cmd_buffers_.end())
      {
        auto *cmd_buffer_ptr = cmd_buffer_it->second.readFromRT();
        if (cmd_buffer_ptr != nullptr)
        {
          joint_data.pos_des = cmd_buffer_ptr->pos;
          joint_data.vel_des = cmd_buffer_ptr->vel;
          joint_data.tau_ff = cmd_buffer_ptr->eff;
        }
      }

      auto params_buffer_it = joint_params_buffers_.find(joint_data.joint->GetName());
      if (params_buffer_it != joint_params_buffers_.end())
      {
        auto *params_buffer_ptr = params_buffer_it->second.readFromRT();
        if (params_buffer_ptr != nullptr)
        {
          joint_data.kp = params_buffer_ptr->first;
          joint_data.kd = params_buffer_ptr->second;
        }
      }

      double torque = joint_data.kp * (joint_data.pos_des - joint_data.position) +
                      joint_data.kd * (joint_data.vel_des - joint_data.velocity) +
                      joint_data.tau_ff;

      joint_data.joint->SetForce(0, torque);
    }
  }

  void RobotGazeboSim::publishJointStates()
  {
    sensor_msgs::JointState joint_state_msg;
    joint_state_msg.header.stamp = ros::Time::now();

    for (const auto &joint_data : joints_)
    {
      joint_state_msg.name.push_back(joint_data.joint->GetName());
      joint_state_msg.position.push_back(joint_data.position);
      joint_state_msg.velocity.push_back(joint_data.velocity);
      joint_state_msg.effort.push_back(joint_data.effort);
    }

    joint_state_pub_.publish(joint_state_msg);
  }

  void RobotGazeboSim::publishImuData()
  {
    if (imu_datas_.empty())
    {
      return;
    }

    // Publish first IMU
    auto &imu = imu_datas_.front();

    sensor_msgs::Imu imu_msg;
    imu_msg.header.stamp = ros::Time::now();
    imu_msg.header.frame_id = imu.frame_id;

    imu_msg.orientation.x = imu.ori[0];
    imu_msg.orientation.y = imu.ori[1];
    imu_msg.orientation.z = imu.ori[2];
    imu_msg.orientation.w = imu.ori[3];

    for (size_t i = 0; i < 9; ++i)
    {
      imu_msg.orientation_covariance[i] = imu.ori_cov[i];
    }

    imu_msg.angular_velocity.x = imu.angular_vel[0];
    imu_msg.angular_velocity.y = imu.angular_vel[1];
    imu_msg.angular_velocity.z = imu.angular_vel[2];

    for (size_t i = 0; i < 9; ++i)
    {
      imu_msg.angular_velocity_covariance[i] = imu.angular_vel_cov[i];
    }

    imu_msg.linear_acceleration.x = imu.linear_acc[0];
    imu_msg.linear_acceleration.y = imu.linear_acc[1];
    imu_msg.linear_acceleration.z = imu.linear_acc[2];

    for (size_t i = 0; i < 9; ++i)
    {
      imu_msg.linear_acceleration_covariance[i] = imu.linear_acc_cov[i];
    }

    imu_pub_.publish(imu_msg);
  }

  void RobotGazeboSim::parseImu(XmlRpc::XmlRpcValue &imu_datas)
  {
    ROS_ASSERT(imu_datas.getType() == XmlRpc::XmlRpcValue::TypeStruct);

    for (auto it = imu_datas.begin(); it != imu_datas.end(); ++it)
    {
      if (!it->second.hasMember("frame_id"))
      {
        ROS_ERROR_STREAM("IMU " << it->first << " has no associated frame id.");
        continue;
      }
      else if (!it->second.hasMember("orientation_covariance_diagonal"))
      {
        ROS_ERROR_STREAM("IMU " << it->first << " has no associated orientation covariance diagonal.");
        continue;
      }
      else if (!it->second.hasMember("angular_velocity_covariance"))
      {
        ROS_ERROR_STREAM("IMU " << it->first << " has no associated angular velocity covariance.");
        continue;
      }
      else if (!it->second.hasMember("linear_acceleration_covariance"))
      {
        ROS_ERROR_STREAM("IMU " << it->first << " has no associated linear acceleration covariance.");
        continue;
      }

      XmlRpc::XmlRpcValue ori_cov = imu_datas[it->first]["orientation_covariance_diagonal"];
      ROS_ASSERT(ori_cov.getType() == XmlRpc::XmlRpcValue::TypeArray);
      ROS_ASSERT(ori_cov.size() == 3);
      for (int i = 0; i < ori_cov.size(); ++i)
      {
        ROS_ASSERT(ori_cov[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      }

      XmlRpc::XmlRpcValue angular_cov = imu_datas[it->first]["angular_velocity_covariance"];
      ROS_ASSERT(angular_cov.getType() == XmlRpc::XmlRpcValue::TypeArray);
      ROS_ASSERT(angular_cov.size() == 3);
      for (int i = 0; i < angular_cov.size(); ++i)
      {
        ROS_ASSERT(angular_cov[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      }

      XmlRpc::XmlRpcValue linear_cov = imu_datas[it->first]["linear_acceleration_covariance"];
      ROS_ASSERT(linear_cov.getType() == XmlRpc::XmlRpcValue::TypeArray);
      ROS_ASSERT(linear_cov.size() == 3);
      for (int i = 0; i < linear_cov.size(); ++i)
      {
        ROS_ASSERT(linear_cov[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      }

      std::string frame_id = imu_datas[it->first]["frame_id"];
      gazebo::physics::LinkPtr link_ptr = model_->GetLink(frame_id);
      if (link_ptr == nullptr)
      {
        ROS_ERROR_STREAM("Link '" << frame_id << "' not found for IMU " << it->first);
        continue;
      }

      ImuData imu_data;
      imu_data.link_ptr = link_ptr;
      imu_data.frame_id = frame_id;
      imu_data.ori[0] = 0.0;
      imu_data.ori[1] = 0.0;
      imu_data.ori[2] = 0.0;
      imu_data.ori[3] = 1.0;
      imu_data.ori_cov[0] = static_cast<double>(ori_cov[0]);
      imu_data.ori_cov[1] = 0.0;
      imu_data.ori_cov[2] = 0.0;
      imu_data.ori_cov[3] = 0.0;
      imu_data.ori_cov[4] = static_cast<double>(ori_cov[1]);
      imu_data.ori_cov[5] = 0.0;
      imu_data.ori_cov[6] = 0.0;
      imu_data.ori_cov[7] = 0.0;
      imu_data.ori_cov[8] = static_cast<double>(ori_cov[2]);

      imu_data.angular_vel[0] = 0.0;
      imu_data.angular_vel[1] = 0.0;
      imu_data.angular_vel[2] = 0.0;
      imu_data.angular_vel_cov[0] = static_cast<double>(angular_cov[0]);
      imu_data.angular_vel_cov[1] = 0.0;
      imu_data.angular_vel_cov[2] = 0.0;
      imu_data.angular_vel_cov[3] = 0.0;
      imu_data.angular_vel_cov[4] = static_cast<double>(angular_cov[1]);
      imu_data.angular_vel_cov[5] = 0.0;
      imu_data.angular_vel_cov[6] = 0.0;
      imu_data.angular_vel_cov[7] = 0.0;
      imu_data.angular_vel_cov[8] = static_cast<double>(angular_cov[2]);

      imu_data.linear_acc[0] = 0.0;
      imu_data.linear_acc[1] = 0.0;
      imu_data.linear_acc[2] = 0.0;
      imu_data.linear_acc_cov[0] = static_cast<double>(linear_cov[0]);
      imu_data.linear_acc_cov[1] = 0.0;
      imu_data.linear_acc_cov[2] = 0.0;
      imu_data.linear_acc_cov[3] = 0.0;
      imu_data.linear_acc_cov[4] = static_cast<double>(linear_cov[1]);
      imu_data.linear_acc_cov[5] = 0.0;
      imu_data.linear_acc_cov[6] = 0.0;
      imu_data.linear_acc_cov[7] = 0.0;
      imu_data.linear_acc_cov[8] = static_cast<double>(linear_cov[2]);

      imu_datas_.push_back(imu_data);
      ROS_INFO_STREAM("Initialized IMU: " << it->first << " on link: " << frame_id);
    }
  }

  void RobotGazeboSim::jointCmdCallback(const sensor_msgs::JointStateConstPtr &msg)
  {
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
      const std::string &joint_name = msg->name[i];
      auto buffer_it = joint_cmd_buffers_.find(joint_name);
      if (buffer_it != joint_cmd_buffers_.end())
      {
        HybridJointCommand cmd;
        cmd.stamp = msg->header.stamp;

        if (i < msg->position.size())
        {
          cmd.pos = msg->position[i];
        }
        if (i < msg->velocity.size())
        {
          cmd.vel = msg->velocity[i];
        }
        if (i < msg->effort.size())
        {
          cmd.eff = msg->effort[i];
        }
        cmd.kp = 0.0;
        cmd.kd = 0.0;

        buffer_it->second.writeFromNonRT(cmd);
      }
    }
  }

  void RobotGazeboSim::jointParamsCallback(const std_msgs::Float64MultiArrayConstPtr &msg)
  {
    if (msg->data.size() < joints_.size() * 2)
    {
      ROS_WARN_STREAM_THROTTLE(1.0, "Joint params message size (" << msg->data.size()
                                                                  << ") is less than expected (" << joints_.size() * 2 << ")");
      return;
    }

    for (size_t i = 0; i < joints_.size(); ++i)
    {
      const std::string &joint_name = joints_[i].joint->GetName();
      auto buffer_it = joint_params_buffers_.find(joint_name);
      if (buffer_it != joint_params_buffers_.end())
      {
        size_t idx = i * 2;
        if (idx + 1 < msg->data.size())
        {
          std::pair<double, double> params(msg->data[idx], msg->data[idx + 1]);
          buffer_it->second.writeFromNonRT(params);
        }
      }
    }
  }

  void RobotGazeboSim::parseContacts(XmlRpc::XmlRpcValue &contact_names)
  {
    ROS_ASSERT(contact_names.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int i = 0; i < contact_names.size(); ++i)
    {
      std::string name = contact_names[i];
      name2contact_.insert(std::make_pair(name, false));
      contact_names_.push_back(name);
      ROS_INFO_STREAM("Added contact sensor: " << name);
    }
    ROS_INFO_STREAM("Initialized " << contact_names_.size() << " contact sensors");
  }

  void RobotGazeboSim::updateContacts()
  {
    if (contact_manager_ == nullptr)
    {
      return;
    }

    for (auto &state : name2contact_)
    {
      state.second = false;
    }

    gazebo::common::Time current_time = model_->GetWorld()->SimTime();
    gazebo::common::Time prev_time = last_update_time_;

    for (const auto &contact : contact_manager_->GetContacts())
    {
      if (static_cast<uint32_t>(contact->time.sec) != static_cast<uint32_t>(prev_time.sec) ||
          static_cast<uint32_t>(contact->time.nsec) != static_cast<uint32_t>(prev_time.nsec))
      {
        continue;
      }

      // Check collision1 link
      std::string link_name = contact->collision1->GetLink()->GetName();
      if (name2contact_.find(link_name) != name2contact_.end())
      {
        name2contact_[link_name] = true;
      }

      // Check collision2 link
      link_name = contact->collision2->GetLink()->GetName();
      if (name2contact_.find(link_name) != name2contact_.end())
      {
        name2contact_[link_name] = true;
      }
    }
  }

  void RobotGazeboSim::publishContactStates()
  {
    if (contact_names_.empty())
    {
      return;
    }

    std_msgs::Float64MultiArray contact_state_msg;
    contact_state_msg.data.resize(contact_names_.size());

    for (size_t i = 0; i < contact_names_.size(); ++i)
    {
      const std::string &name = contact_names_[i];
      auto it = name2contact_.find(name);
      if (it != name2contact_.end())
      {
        contact_state_msg.data[i] = it->second ? 1.0 : 0.0;
      }
      else
      {
        contact_state_msg.data[i] = 0.0;
      }
    }

    contact_state_pub_.publish(contact_state_msg);
  }

} // namespace robot

GZ_REGISTER_MODEL_PLUGIN(robot::RobotGazeboSim)
