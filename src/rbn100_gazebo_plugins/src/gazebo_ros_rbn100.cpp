#include "rbn100_gazebo_plugins/gazebo_ros_rbn100.h"
#include <tf/LinearMath/Quaternion.h>
#if GAZEBO_MAJOR_VERSION >= 9
  #include <ignition/math/Vector3.hh>
  #include <ignition/math/Quaternion.hh>
#else
  #include <gazebo/math/gzmath.hh>
#endif

namespace gazebo
{

// constructor
GazeboRosRbn100::GazeboRosRbn100(){
  motors_enabled_ = true;
  rate_step_ = imu_step_ = sonar_step_ = camera_step_ = common::Time(0);
}

// deconstructor
GazeboRosRbn100::~GazeboRosRbn100(){}

void GazeboRosRbn100::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  sdf_ = sdf;
  std::string model_name = sdf_->GetParent()->Get<std::string>("name"); // why rbn100???
  node_name_ = model_name;

  model_ = model;
  if (!model){
    ROS_ERROR_STREAM("Load: Invalid model pointer! [" << node_name_ << "]");
    return;
  }

  world_ = model_->GetWorld();

  // Gazebo ros helper class, simplifies the parameter and rosnode handling
  GazeboRosPtr gazebo_ros = GazeboRosPtr(new GazeboRos(model, this->sdf_, "rbn100-sim"));

  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("Load: A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  double update_rate = 0;
  if (!this->sdf_->HasElement("update_rate"))
  {
    ROS_DEBUG_NAMED("sensor", "sensor plugin missing <update_rate>, defaults to 0"
             " (as fast as possible)");
  }else{
    update_rate = this->sdf_->Get<double>("update_rate");
  }
  update_period_ = 1.0 / update_rate;

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
  // if(prepareIMU() == false)
  //   return;
  if(prepareUltra() == false)
    return;
  if(prepareStereoCamera() == false)
    return;

  setupRosApi(gazebo_ros);

  prev_update_time_ = world_->SimTime();
  // prev_update_time_.Set(ros::Time::now().sec, ros::Time::now().nsec);

  ROS_INFO_STREAM("GazeboRosRbn100 plugin ready to go! [" << model_name << "]");
  // Listen to the update event. This event is broadcast every simulation iteration.
  update_connection_ = event::Events::ConnectWorldUpdateEnd(std::bind(&GazeboRosRbn100::OnUpdate, this));
}

void GazeboRosRbn100::OnUpdate()
{
  // 遍历处理回调队列消息
  ros::spinOnce();

  common::Time time_now = world_->SimTime();
  
  // bumper and cliff event
  updateBumper();
  updateCliffSensor();
  
  // rate control
  common::Time step_time = time_now - prev_update_time_;
  rate_step_ += step_time;
  sonar_step_ += step_time;
  camera_step_ += step_time;

  // get sonar data
  if(sonar_step_.Double() >= sonar_period_){
    updateUltra();
    sonar_step_ = common::Time(0);
  }

  // get camera data
  if(camera_step_.Double() >= camera_period_){
    updateStereoCamera();
    camera_step_ = common::Time(0);
  }
  
  if(rate_step_.Double() >= update_period_){
    // get encoder data
    updateOdometry(rate_step_);
    // get wheel speed
    updateWheelSpeed();
    // get joints states
    updateJointState();
    // send vel cmd
    propagateVelocityCommands();
    rate_step_ = common::Time(0);
  }

  prev_update_time_ = time_now;
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosRbn100);

} // namespace gazebo