#include "rbn100_gazebo_plugins/gazebo_ros_rbn100.h"

namespace gazebo
{

/* void GazeboRosRbn100::prepareMotorPower() 
{
  motors_enabled_ = true;
} */

bool GazeboRosRbn100::prepareJointState()
{
  if (sdf_->HasElement("left_wheel_joint_name"))
  {
    left_wheel_joint_name_ = sdf_->GetElement("left_wheel_joint_name")->Get<std::string>();
  }
  else
  {
    ROS_ERROR_STREAM("prepareJointState: Couldn't find left wheel joint in the model description!"
                     << " Did you specify the correct joint name?" << " [" << node_name_ <<"]");
    return false;
  }
  if (sdf_->HasElement("right_wheel_joint_name"))
  {
    right_wheel_joint_name_ = sdf_->GetElement("right_wheel_joint_name")->Get<std::string>();
  }
  else
  {
    ROS_ERROR_STREAM("prepareJointState: Couldn't find right wheel joint in the model description!"
                     << " Did you specify the correct joint name?" << " [" << node_name_ <<"]");
    return false;
  }
  joints_[LEFT] = model_->GetJoint(left_wheel_joint_name_);
  joints_[RIGHT] = model_->GetJoint(right_wheel_joint_name_);
  if (!joints_[LEFT] || !joints_[RIGHT])
  {
    ROS_ERROR_STREAM("prepareJointState: Couldn't find specified wheel joints in the model! [" << node_name_ <<"]");
    return false;
  }
  joint_state_.header.frame_id = "Joint States";
  joint_state_.name.push_back(left_wheel_joint_name_);
  joint_state_.position.push_back(0);
  joint_state_.velocity.push_back(0);
  joint_state_.effort.push_back(0);
  joint_state_.name.push_back(right_wheel_joint_name_);
  joint_state_.position.push_back(0);
  joint_state_.velocity.push_back(0);
  joint_state_.effort.push_back(0);

  return true;
}

void GazeboRosRbn100::preparePublishTf()
{
  if (sdf_->HasElement("publish_tf"))
  {
    publish_tf_ = sdf_->GetElement("publish_tf")->Get<bool>();
    if (publish_tf_)
    {
      ROS_INFO_STREAM("preparePublishTf: Will publish tf." << " [" << node_name_ <<"]");
    }
    else
    {
      ROS_INFO_STREAM("preparePublishTf: Won't publish tf." << " [" << node_name_ <<"]");
    }
  }
  else
  {
    publish_tf_ = false;
    ROS_INFO_STREAM("preparePublishTf: Couldn't find the 'publish tf' parameter in the model description."
                     << " Won't publish tf." << " [" << node_name_ <<"]");
  }
}

bool GazeboRosRbn100::prepareWheelAndTorque()
{
  if (sdf_->HasElement("wheel_separation"))
  {
    wheel_sep_ = sdf_->GetElement("wheel_separation")->Get<double>();
  }
  else
  {
    ROS_ERROR_STREAM("prepareWheelAndTorque: Couldn't find the wheel separation parameter in the model description!"
                     << " Did you specify it?" << " [" << node_name_ <<"]");
    return false;
  }
  if (sdf_->HasElement("wheel_diameter"))
  {
    wheel_diam_ = sdf_->GetElement("wheel_diameter")->Get<double>();
  }
  else
  {
    ROS_ERROR_STREAM("prepareWheelAndTorque: Couldn't find the wheel diameter parameter in the model description!"
                     << " Did you specify it?" << " [" << node_name_ <<"]");
    return false;
  }
  if (sdf_->HasElement("torque"))
  {
    torque_ = sdf_->GetElement("torque")->Get<double>();
  }
  else
  {
    ROS_ERROR_STREAM("prepareWheelAndTorque: Couldn't find the torque parameter in the model description!"
                     << " Did you specify it?" << " [" << node_name_ <<"]");
    return false;
  }
  return true;
}

bool GazeboRosRbn100::prepareOdom()
{
  if (sdf_->HasElement("odom_name"))
  {
    odom_name_ = sdf_->GetElement("odom_name")->Get<std::string>();
  }
  else
  {
    ROS_ERROR_STREAM("prepareOdom: Couldn't find the odom_name parameter in the model description!"
                     << " Did you specify it?" << " [" << node_name_ <<"]");
    return false;
  } 

  //  放大以获取更多信息提高精度
  odom_.EncoderL = 0xfff * 1e5;
  odom_.EncoderR = 0xfff * 1e5;
  m_per_encoder_ = PI * wheel_diam_ / ENCODER_N;

  odom_pose_[0] = 0.0;
  odom_pose_[1] = 0.0;
  odom_pose_[2] = 0.0;

  return true;
}

bool GazeboRosRbn100::prepareVelocityCommand()
{
  if (sdf_->HasElement("velocity_command_timeout"))
  {
    cmd_vel_timeout_ = sdf_->GetElement("velocity_command_timeout")->Get<double>();
  }
  else
  {
    ROS_ERROR_STREAM("prepareVelocityCommand: Couldn't find the wheel separation parameter in the model description!"
                     << " Did you specify it?" << " [" << node_name_ <<"]");
    return false;
  }
  #if GAZEBO_MAJOR_VERSION >= 9
    // last_cmd_vel_time_ = world_->SimTime();
    last_cmd_vel_time_.Set(ros::Time::now().sec, ros::Time::now().nsec);
  #else
    last_cmd_vel_time_ = world_->GetSimTime();
  #endif
  return true;
}

bool GazeboRosRbn100::prepareCliffSensor()
{
  std::string cliff_sensor_FL_name, cliff_sensor_FR_name, cliff_sensor_BL_name, cliff_sensor_BR_name;
  if (sdf_->HasElement("cliff_sensor_FL_name"))
  {
    cliff_sensor_FL_name = sdf_->GetElement("cliff_sensor_FL_name")->Get<std::string>();
  }
  else
  {
    ROS_ERROR_STREAM("Couldn't find the name of FL cliff sensor in the model description!"
                     << " Did you specify it?" << " [" << node_name_ <<"]");
    return false;
  }
  if (sdf_->HasElement("cliff_sensor_FR_name"))
  {
    cliff_sensor_FR_name = sdf_->GetElement("cliff_sensor_FR_name")->Get<std::string>();
  }
  else
  {
    ROS_ERROR_STREAM("Couldn't find the name of FR cliff sensor in the model description!"
                     << " Did you specify it?" << " [" << node_name_ <<"]");
    return false;
  }
  if (sdf_->HasElement("cliff_sensor_BL_name"))
  {
    cliff_sensor_BL_name = sdf_->GetElement("cliff_sensor_BL_name")->Get<std::string>();
  }
  else
  {
    ROS_ERROR_STREAM("Couldn't find the name of BL cliff sensor in the model description!"
                     << " Did you specify it?" << " [" << node_name_ <<"]");
    return false;
  }
  if (sdf_->HasElement("cliff_sensor_BR_name"))
  {
    cliff_sensor_BR_name = sdf_->GetElement("cliff_sensor_BR_name")->Get<std::string>();
  }
  else
  {
    ROS_ERROR_STREAM("Couldn't find the name of BR cliff sensor in the model description!"
                     << " Did you specify it?" << " [" << node_name_ <<"]");
    return false;
  }
  // dynamic_cast: 多态时把实际指向派生类的基类指针转换成派生类指针，如果指针是智能指针，需要使用pointer cast
  cliff_sensor_FL_ = std::dynamic_pointer_cast<sensors::RaySensor>(
                       sensors::SensorManager::Instance()->GetSensor(cliff_sensor_FL_name));
  cliff_sensor_FR_ = std::dynamic_pointer_cast<sensors::RaySensor>(
                        sensors::SensorManager::Instance()->GetSensor(cliff_sensor_FR_name));
  cliff_sensor_BL_ = std::dynamic_pointer_cast<sensors::RaySensor>(
                        sensors::SensorManager::Instance()->GetSensor(cliff_sensor_BL_name));
  cliff_sensor_BR_ = std::dynamic_pointer_cast<sensors::RaySensor>(
                        sensors::SensorManager::Instance()->GetSensor(cliff_sensor_BR_name));
  if (!cliff_sensor_FL_)
  {
    ROS_ERROR_STREAM("Couldn't find the FL cliff sensor in the model! [" << node_name_ <<"]");
    return false;
  }
  if (!cliff_sensor_FR_)
  {
    ROS_ERROR_STREAM("Couldn't find the FR cliff sensor in the model! [" << node_name_ <<"]");
    return false;
  }
  if (!cliff_sensor_BL_)
  {
    ROS_ERROR_STREAM("Couldn't find the BL cliff sensor in the model! [" << node_name_ <<"]");
    return false;
  }
  if (!cliff_sensor_BR_)
  {
    ROS_ERROR_STREAM("Couldn't find the BR cliff sensor in the model! [" << node_name_ <<"]");
    return false;
  }
  if (sdf_->HasElement("cliff_detection_threshold"))
  {
    cliff_detection_threshold_ = sdf_->GetElement("cliff_detection_threshold")->Get<double>();
  }
  else
  {
    ROS_ERROR_STREAM("Couldn't find the cliff detection threshold parameter in the model description!"
                     << " Did you specify it?" << " [" << node_name_ <<"]");
    return false;
  }
  cliff_sensor_FL_->SetActive(true);
  cliff_sensor_FR_->SetActive(true);
  cliff_sensor_BL_->SetActive(true);
  cliff_sensor_BR_->SetActive(true);

  return true;
}

bool GazeboRosRbn100::prepareBumper()
{
  if (sdf_->HasElement("bumper_name"))
  {
    bumper_name_ = sdf_->GetElement("bumper_name")->Get<std::string>();
  }
  else
  {
    ROS_ERROR_STREAM("Couldn't find the name of bumper sensor in the model description!"
                     << " Did you specify it?" << " [" << node_name_ <<"]");
    return false;
  }

  bumper_ = std::dynamic_pointer_cast<sensors::ContactSensor>(
            sensors::SensorManager::Instance()->GetSensor(bumper_name_));
  if (!bumper_)
  {
    ROS_ERROR_STREAM("Couldn't find the bumpers in the model! [" << node_name_ <<"]");
    return false;
  }
  bumper_->SetActive(true);
  return true;
}

bool GazeboRosRbn100::prepareIMU()
{
  if (sdf_->HasElement("imu_name"))
  {
    imu_name_ = sdf_->GetElement("imu_name")->Get<std::string>();
  }
  else
  {
    ROS_ERROR_STREAM("Couldn't find the name of IMU sensor in the model description!"
                     << " Did you specify it?" << " [" << node_name_ <<"]");
    return false;
  }
  if (!sdf_->HasElement("imu_rate"))
  {
    ROS_DEBUG_NAMED("imu", "imu plugin missing <bumper_rate>, defaults to 0.0"
             " (as fast as possible)");
    imu_rate_ = 0.0;
  }
  else
    imu_rate_ = sdf_->GetElement("imu_rate")->Get<double>();

  #if GAZEBO_MAJOR_VERSION >= 9
    imu_ = std::dynamic_pointer_cast<sensors::ImuSensor>(
            sensors::get_sensor(world_->Name()+"::"+node_name_+"::base_footprint::"+imu_name_));
  #else
    imu_ = std::dynamic_pointer_cast<sensors::ImuSensor>(
            sensors::get_sensor(world_->GetName()+"::"+node_name_+"::base_footprint::"+imu_name));

  #endif
  if (!imu_)
  {
    ROS_ERROR_STREAM("Couldn't find the IMU in the model! [" << node_name_ <<"]");
    return false;
  }
  imu_->SetActive(true);
  return true;
}

/* 
  call backs
 */
void GazeboRosRbn100::motorPowerCB(const rbn100_msgs::MotorPowerPtr &msg)
{
  // if ((msg->state == rbn100_msgs::MotorPower::ON) && (!motors_enabled_))
  if (msg->state == rbn100_msgs::MotorPower::ON)
  {
    motors_enabled_ = true;
    // set_motor_enable(true);
    ROS_INFO_STREAM("motorPowerCB: Motors fired up. [" << node_name_ << "]");
  }
  else if (msg->state == rbn100_msgs::MotorPower::OFF)
  {
    motors_enabled_ = false;
    // set_motor_enable(false);
    ROS_INFO_STREAM("motorPowerCB: Motors taking a rest. [" << node_name_ << "]");
  }
}

void GazeboRosRbn100::resetOdomCB(const std_msgs::EmptyConstPtr &msg)
{
  ROS_INFO_STREAM("Reset Odom CB!");

  odom_.EncoderL = 0xfff * 1e5;
  odom_.EncoderR = 0xfff * 1e5;

  odom_pose_[0] = 0.0;
  odom_pose_[1] = 0.0;
  odom_pose_[2] = 0.0;
}

void GazeboRosRbn100::cmdVelCB(const geometry_msgs::TwistConstPtr &msg)
{
  #if GAZEBO_MAJOR_VERSION >= 9
    // last_cmd_vel_time_ = world_->SimTime();
    last_cmd_vel_time_.Set(ros::Time::now().sec, ros::Time::now().nsec);
  #else
    last_cmd_vel_time_ = world_->GetSimTime();
  #endif

  // 线速度、角速度、轮间距和两轮速度的关系
  wheel_speed_cmd_[LEFT] = msg->linear.x - msg->angular.z * (wheel_sep_) / 2;
  wheel_speed_cmd_[RIGHT] = msg->linear.x + msg->angular.z * (wheel_sep_) / 2;
}

void GazeboRosRbn100::bumperAutoStopMotorCB(const std_msgs::BoolPtr &msg)
{
   bumper_auto_stop_motor_flag = msg->data;
   ROS_INFO_STREAM("Enable flag of bumper auto brake: "<< bumper_auto_stop_motor_flag);
}

void GazeboRosRbn100::cliffAutoStopMotorCB(const std_msgs::BoolPtr &msg)
{
   cliff_auto_stop_motor_flag = msg->data;
   ROS_INFO_STREAM("Enable flag for cliff auto brake: "<< cliff_auto_stop_motor_flag);
}

void GazeboRosRbn100::setupRosApi(std::string& model_name)
{
  std::string base_prefix;
  gazebo_ros_->node()->param("base_prefix", base_prefix, std::string("mobile_base"));
  
  /* 
    publisher（以指定频率往共享发布队列中发布数据，队列长度为1保证数据都是最新的）
   */
  // pub of joint_states
  std::string joint_states_topic = "joint_states";
  joint_state_pub_ = gazebo_ros_->node()->advertise<sensor_msgs::JointState>(joint_states_topic, 1);
  ROS_INFO("%s: Advertise joint_states[%s]!", gazebo_ros_->info(), joint_states_topic.c_str());

  // pub of odom
  std::string odom_topic = odom_name_;
  // odom_pub_ = gazebo_ros_->node()->advertise<nav_msgs::Odometry>(odom_topic, 1);
  odom_pub_ = gazebo_ros_->node()->advertise<rbn100_msgs::Encoder>(odom_topic, 1);
  ROS_INFO("%s: Advertise Odometry[%s]!", gazebo_ros_->info(), odom_topic.c_str());

  // pub of cliff
  std::string cliff_topic = base_prefix + "/events/cliff";
  cliff_event_pub_ = gazebo_ros_->node()->advertise<rbn100_msgs::CliffEvent>(cliff_topic, 1);
  ROS_INFO("%s: Advertise Cliff[%s]!", gazebo_ros_->info(), cliff_topic.c_str());

  // pub of bumper
  std::string bumper_topic = base_prefix + "/events/" + bumper_name_;
  bumper_event_pub_ = gazebo_ros_->node()->advertise<rbn100_msgs::BumperEvent>(bumper_topic, 1);
  ROS_INFO("%s: Advertise Bumper[%s]!", gazebo_ros_->info(), bumper_topic.c_str());

  // pub of IMU
  std::string imu_topic = base_prefix + "/sensors/" + imu_name_;
  imu_pub_ = gazebo_ros_->node()->advertise<sensor_msgs::Imu>(imu_topic, 1);
  ROS_INFO("%s: Advertise IMU[%s]!", gazebo_ros_->info(), imu_topic.c_str());

  // pub of motor power state
  // std::string motor_power_state_topic = base_prefix + "/events/motor_power_state";
  // motor_power_state_pub_ = gazebo_ros_->node()->advertise<rbn100_msgs::MotorPower>(motor_power_state_topic, 1);
  // ROS_INFO("%s: Advertise MotorPower[%s]!", gazebo_ros_->info(), motor_power_state_topic.c_str());

  /* 
    subscriber（从发布队列中取数据放入共享订阅队列供回调函数使用，没有订阅频率，快慢看机器，
                回调函数尽量简短，以保证信息处理及时，spin和spinonce都是单线程顺序查看
                队列中的所有信息，多线程可以分topic查看）
   */
  // sub of motor power
  std::string motor_power_topic = base_prefix + "/commands/motor_power";
  motor_power_sub_ = gazebo_ros_->node()->subscribe(motor_power_topic, 10, &GazeboRosRbn100::motorPowerCB, this);
  ROS_INFO("%s: Try to subscribe to %s!", gazebo_ros_->info(), motor_power_topic.c_str());

  // sub of odom reset
  std::string odom_reset_topic = base_prefix + "/commands/reset_odometry";
  odom_reset_sub_ = gazebo_ros_->node()->subscribe(odom_reset_topic, 10, &GazeboRosRbn100::resetOdomCB, this);
  ROS_INFO("%s: Try to subscribe to %s!", gazebo_ros_->info(), odom_reset_topic.c_str());

  // sub of cmd_vel
  std::string cmd_vel_topic = base_prefix + "/commands/velocity";
  cmd_vel_sub_ = gazebo_ros_->node()->subscribe(cmd_vel_topic, 100, &GazeboRosRbn100::cmdVelCB, this);
  ROS_INFO("%s: Try to subscribe to %s!", gazebo_ros_->info(), cmd_vel_topic.c_str());

  // sub of enable bumper auto stop motor
  std::string bumper_auto_stop_motor_topic = base_prefix + "/commands/bumper_auto_stop_motor";
  bumper_auto_stop_motor_sub_ = gazebo_ros_->node()->subscribe(bumper_auto_stop_motor_topic, 10, &GazeboRosRbn100::bumperAutoStopMotorCB, this);
  ROS_INFO("%s: Try to subscribe to %s!", gazebo_ros_->info(), bumper_auto_stop_motor_topic.c_str());
  
  // sub of enable cliff auto stop motor
  std::string cliff_auto_stop_motor_topic = base_prefix + "/commands/cliff_auto_stop_motor";
  cliff_auto_stop_motor_sub_ = gazebo_ros_->node()->subscribe(cliff_auto_stop_motor_topic, 10, &GazeboRosRbn100::cliffAutoStopMotorCB, this);
  ROS_INFO("%s: Try to subscribe to %s!", gazebo_ros_->info(), cliff_auto_stop_motor_topic.c_str());
}
}
