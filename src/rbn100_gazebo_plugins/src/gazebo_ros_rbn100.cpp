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
  bumper_auto_stop_motor_flag = false;
  cliff_auto_stop_motor_flag = false;

  console_log_rate = 1;
  rate_step_ = 0;
  imu_step_ = 0;
  sonar_step_ = 0;
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

  if (!sdf_->HasElement("update_rate"))
  {
    ROS_DEBUG_NAMED("sensor", "sensor plugin missing <update_rate>, defaults to 0.0"
             " (as fast as possible)");
    update_rate_ = 0.0;
  }
  else
    update_rate_ = sdf_->GetElement("update_rate")->Get<double>();

  // prepareMotorPower();
  preparePublishTf();

  if(prepareJointState() == false)
    return;
  if(prepareWheelAndTorque() == false)
    return;
  if(prepareOdom() == false)
    return;
  if(prepareVelocityCommand() == false)
    return;
  if(prepareCliffSensor() == false)
    return;
  if(prepareBumper() == false)
    return;
  if(prepareIMU() == false)
    return;
  // if(prepareUltra() == false)
    // return;

  setupRosApi(model_name);

  // prev_update_time_ = world_->SimTime();
  prev_update_time_.Set(ros::Time::now().sec, ros::Time::now().nsec);

  ROS_INFO_STREAM("GazeboRosRbn100 plugin ready to go! [" << node_name_ << "]");
  // Listen to the update event. This event is broadcast every simulation iteration.
  update_connection_ = event::Events::ConnectWorldUpdateEnd(
      boost::bind(&GazeboRosRbn100::OnUpdate, this));
}

// 1ms，在update中发布话题调用回调函数并更新底盘状态
void GazeboRosRbn100::OnUpdate()
{
  // 查看订阅队列消息，调用回调函数设置标志等
  ros::spinOnce();

  common::Time time_now;
  time_now.Set(ros::Time::now().sec, ros::Time::now().nsec);

  updateJointState();
  /* 
    rate control
   */
  common::Time step_time = time_now - prev_update_time_;
  imu_step_ += step_time;
  rate_step_ += step_time;
  sonar_step_ += step_time;
  
  if(rate_step_.Double() >= (1.0 / update_rate_)){
    updateOdometry(rate_step_);
    propagateVelocityCommands();
    rate_step_ = 0;
  }
  if(imu_step_.Double() >= (1.0 / imu_rate_)){
    updateIMU();
    imu_step_ = 0;
  }
  // if(sonar_step_.Double() >= (1.0 / sonar_rate_)){
  //   updateUltra();
  //   sonar_step_ = 0;
  // }
  updateBumper();
  updateCliffSensor();
  prev_update_time_ = time_now;
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosRbn100);

} // namespace gazebo
