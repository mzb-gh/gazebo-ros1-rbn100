/* 
  插件类实现主函数
 */

#include <cmath>
#include <cstring>
#include <boost/bind.hpp>
#include <sensor_msgs/JointState.h>
#include <tf/LinearMath/Quaternion.h>
#include "rbn100_gazebo_plugins/gazebo_ros_rbn100.h"
#if GAZEBO_MAJOR_VERSION >= 9
  // #include <ignition/math.hh>
  #include <ignition/math/Vector3.hh>
  #include <ignition/math/Quaternion.hh>
#else
  #include <gazebo/math/gzmath.hh>
#endif

namespace gazebo
{

// constructor
GazeboRosRbn100::GazeboRosRbn100() : shutdown_requested_(false)
{
  // Initialize variables
  wheel_speed_cmd_[LEFT] = 0.0;
  wheel_speed_cmd_[RIGHT] = 0.0;
  cliff_detected_FL_ = false;
  cliff_detected_FR_ = false;
  cliff_detected_BL_ = false;
  cliff_detected_BR_ = false;
  bumper_was_pressed_ = false;
  motors_enabled_ = true;
  bumper_auto_stop_motor_flag = true;
  cliff_auto_stop_motor_flag = true;

  console_log_rate = 1;
}

// deconstructor
GazeboRosRbn100::~GazeboRosRbn100()
{
//  rosnode_->shutdown();
  shutdown_requested_ = true;
  // Wait for spinner thread to end
//  ros_spinner_thread_->join();

  //  delete spinner_thread_;
//  delete rosnode_;
}

void GazeboRosRbn100::Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
{
  model_ = parent;
  std::string world_name = model_->GetWorld()->Name();
  if (!model_)
  {
    ROS_ERROR_STREAM("Load: Invalid model pointer! [" << node_name_ << "]");
    return;
  }

  gazebo_ros_ = GazeboRosPtr(new GazeboRos(model_, sdf, "rbn100"));
  sdf_ = sdf;
  // gazebo_ros_->getParameter(this->update_rate_, "update_rate", 0.0);

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("Load: A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  // Get then name of the parent model and use it as node name
  std::string model_name = sdf->GetParent()->Get<std::string>("name");
  gzdbg << "Load: Plugin model name: " << model_name << "\n"; //rbn100
  node_name_ = model_name;
  world_ = parent->GetWorld();

  // prepareMotorPower();
  preparePublishTf();

  if(prepareJointState() == false)
    return;
  if(prepareWheelAndTorque() == false)
    return;

  prepareOdom();

  if(prepareVelocityCommand() == false)
    return;
  if(prepareCliffSensor() == false)
    return;
  if(prepareBumper() == false)
    return;
  if(prepareIMU() == false)
    return;

  setupRosApi(model_name);

  #if GAZEBO_MAJOR_VERSION >= 9
    // prev_update_time_ = world_->SimTime();
    prev_update_time_.Set(ros::Time::now().sec, ros::Time::now().nsec);
  #else
    prev_update_time_ = world_->GetSimTime();
  #endif

  ROS_INFO_STREAM("GazeboRosRbn100 plugin ready to go! [" << node_name_ << "]");
  // Listen to the update event. This event is broadcast every simulation iteration.
  update_connection_ = event::Events::ConnectWorldUpdateEnd(boost::bind(&GazeboRosRbn100::OnUpdate, this));
}

// 1ms，在update中发布话题调用回调函数并更新底盘状态
void GazeboRosRbn100::OnUpdate()
{
  // 查看订阅队列消息，调用回调函数设置标志等
  ros::spinOnce();

  /*
   * Update current time and time step
   */
  common::Time time_now;
  #if GAZEBO_MAJOR_VERSION >= 9
    // time_now = world_->SimTime();
    time_now.Set(ros::Time::now().sec, ros::Time::now().nsec);
  #else
    time_now = world_->GetSimTime();
  #endif

  // if (time_now < prev_update_time_) {
  //   ROS_WARN_NAMED("gazebo_ros_rbn100", "Negative update time difference detected.");
  //   prev_update_time_ = time_now;
  // }
  common::Time step_time = time_now - prev_update_time_;

  // update vel
  propagateVelocityCommands();

  // publish rate control
  // if (this->update_rate_ > 0 && step_time.Double() < (1.0 / this->update_rate_)) {
  //   return;
  // }

  prev_update_time_ = time_now;

  updateJointState();
  updateOdometry(step_time);
  updateIMU();
  updateCliffSensor();
  updateBumper();
  // pubSensorState();
}

/* void GazeboRosRbn100::spin()
{
  ros::Rate r(10);
  while(ros::ok() && !shutdown_requested_)
  {
    r.sleep();
    ROS_INFO_STREAM("------------------------");
    ROS_INFO_STREAM("GazeboRosRbn100:spin");

  }
} */

/* cliff and bumper motor control */
/* void GazeboRosRbn100::set_motor_enable(uint8_t _state)
{
  ROS_INFO_STREAM("Motors enable [" << (int)_state << "]");
  motors_enabled_ = _state;
  motorPowerPub(_state);
} */

/* void GazeboRosRbn100::bumperAutoStopMotorCB(const std_msgs::BoolPtr &msg)
{
   bumper_auto_stop_flag = msg->data;
   ROS_INFO_STREAM("forbidden flag for bumper press event to stop motor power: "<<bumper_auto_stop_flag);
}

void GazeboRosRbn100::cliffAutoStopMotorCB(const std_msgs::BoolPtr &msg)
{
   cliff_auto_stop_flag = msg->data;
   ROS_INFO_STREAM("forbidden flag for cliff event to stop motor power: "<<cliff_auto_stop_flag);
} */

/* void GazeboRosRbn100::motorPowerPub(std::uint8_t _state)
{
  rbn100_msgs::MotorPower msg;
  msg.state = _state;
  motor_power_state_pub_.publish(msg);
}
 */
/* void GazeboRosRbn100::motorPowerCB(const rbn100_msgs::MotorPowerPtr &msg)
{
  if ((msg->state == rbn100_msgs::MotorPower::ON) && (!motors_enabled_))
  {
    // motors_enabled_ = true;
    set_motor_enable(true);
    ROS_INFO_STREAM("Motors fired up. [" << node_name_ << "]");
  }
  else if ((msg->state == rbn100_msgs::MotorPower::OFF) && (motors_enabled_))
  {
    // motors_enabled_ = false;
    set_motor_enable(false);
    ROS_INFO_STREAM("Motors taking a rest. [" << node_name_ << "]");
  }
} */

/* void GazeboRosRbn100::cmdVelCB(const geometry_msgs::TwistConstPtr &msg)
{
  #if GAZEBO_MAJOR_VERSION >= 9
    // last_cmd_vel_time_ = world_->SimTime();
    last_cmd_vel_time_.Set(ros::Time::now().sec, ros::Time::now().nsec);
  #else
    last_cmd_vel_time_ = world_->GetSimTime();
  #endif

  wheel_speed_cmd_[LEFT] = msg->linear.x - msg->angular.z * (wheel_sep_) / 2;
  wheel_speed_cmd_[RIGHT] = msg->linear.x + msg->angular.z * (wheel_sep_) / 2;
} */

/* void GazeboRosRbn100::resetOdomCB(const std_msgs::EmptyConstPtr &msg)
{
  ROS_INFO("Reset Odom!");
  odom_pose_[0] = 0.0;
  odom_pose_[1] = 0.0;
  odom_pose_[2] = 0.0;
} */

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosRbn100);

} // namespace gazebo
