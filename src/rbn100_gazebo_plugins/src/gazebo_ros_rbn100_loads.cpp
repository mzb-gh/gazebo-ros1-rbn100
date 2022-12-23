#include "rbn100_gazebo_plugins/gazebo_ros_rbn100.h"

namespace gazebo
{

/* void GazeboRosRbn100::prepareMotorPower() 
{
  motors_enabled_ = true;
} */

bool GazeboRosRbn100::prepareJointState(){
  std::string left_wheel_joint_name, right_wheel_joint_name;
  if (sdf_->HasElement("left_wheel_joint_name")){
    left_wheel_joint_name = sdf_->GetElement("left_wheel_joint_name")->Get<std::string>();
  }else{
    ROS_ERROR_STREAM("prepareJointState: Couldn't find left wheel joint in the model description!"
                     << " Did you specify the correct joint name?" << " [" << node_name_ <<"]");
    return false;
  }
  if (sdf_->HasElement("right_wheel_joint_name")){
    right_wheel_joint_name = sdf_->GetElement("right_wheel_joint_name")->Get<std::string>();
  }else{
    ROS_ERROR_STREAM("prepareJointState: Couldn't find right wheel joint in the model description!"
                     << " Did you specify the correct joint name?" << " [" << node_name_ <<"]");
    return false;
  }
  wheel_joints_[LEFT] = model_->GetJoint(left_wheel_joint_name);
  wheel_joints_[RIGHT] = model_->GetJoint(right_wheel_joint_name);
  if (!wheel_joints_[LEFT] || !wheel_joints_[RIGHT]){
    ROS_ERROR_STREAM("prepareJointState: Couldn't find specified wheel joints in the model! [" << node_name_ <<"]");
    return false;
  }
  return true;
}

void GazeboRosRbn100::preparePublishTf(){
  if(sdf_->HasElement("publish_tf")){
    publish_tf_ = sdf_->GetElement("publish_tf")->Get<bool>();
    if (publish_tf_){
      ROS_INFO_STREAM("preparePublishTf: Will publish tf." << " [" << node_name_ <<"]");
    }else{
      ROS_INFO_STREAM("preparePublishTf: Won't publish tf." << " [" << node_name_ <<"]");
    }
  }else{
    publish_tf_ = false;
    ROS_INFO_STREAM("preparePublishTf: Couldn't find the 'publish tf' parameter in the model description."
                     << " Won't publish tf." << " [" << node_name_ <<"]");
  }
}

bool GazeboRosRbn100::prepareWheelAndTorque(){
  if (sdf_->HasElement("wheel_separation")){
    wheel_sep_ = sdf_->GetElement("wheel_separation")->Get<double>();
  }else{
    ROS_ERROR_STREAM("prepareWheelAndTorque: Couldn't find the wheel separation parameter in the model description!"
                     << " Did you specify it?" << " [" << node_name_ <<"]");
    return false;
  }
  if (sdf_->HasElement("wheel_diameter")){
    wheel_diam_ = sdf_->GetElement("wheel_diameter")->Get<double>();
  }else{
    ROS_ERROR_STREAM("prepareWheelAndTorque: Couldn't find the wheel diameter parameter in the model description!"
                     << " Did you specify it?" << " [" << node_name_ <<"]");
    return false;
  }
  if (sdf_->HasElement("torque")){
    torque_ = sdf_->GetElement("torque")->Get<double>();
  }else{
    ROS_ERROR_STREAM("prepareWheelAndTorque: Couldn't find the torque parameter in the model description!"
                     << " Did you specify it?" << " [" << node_name_ <<"]");
    return false;
  }
  return true;
}

bool GazeboRosRbn100::prepareOdom(){
  if (sdf_->HasElement("odom_topic")){
    odom_topic_ = sdf_->GetElement("odom_topic")->Get<std::string>();
  }else{
    ROS_ERROR_STREAM("prepareOdom: Couldn't find the odom_topic parameter in the model description!"
                     << " Did you specify it?" << " [" << node_name_ <<"]");
    return false;
  } 

  odom_msg_.EncoderL = 0;
  odom_msg_.EncoderR = 0;
  m_per_encoder_ = PI * wheel_diam_ / ENCODER_N;

  return true;
}

bool GazeboRosRbn100::prepareVelocityCommand(){
  if (sdf_->HasElement("velocity_command_timeout")){
    cmd_vel_timeout_ = sdf_->GetElement("velocity_command_timeout")->Get<double>();
  }else{
    ROS_ERROR_STREAM("prepareVelocityCommand: Couldn't find the velocity_command_timeout parameter in the model description!"
                     << " Did you specify it?" << " [" << node_name_ <<"]");
    return false;
  }
  last_cmd_vel_time_ = world_->SimTime();
  wheel_speed_cmd_[LEFT] = wheel_speed_cmd_[RIGHT] = 0.0;

  return true;
}

bool GazeboRosRbn100::prepareCliffSensor(){
  std::string cliff_sensor_FL_name, cliff_sensor_FR_name, cliff_sensor_BL_name, cliff_sensor_BR_name;

  if (sdf_->HasElement("cliff_FL_sensor_name")){
    cliff_sensor_FL_name = sdf_->GetElement("cliff_FL_sensor_name")->Get<std::string>();
  }else{
    ROS_ERROR_STREAM("Couldn't find the name of FL cliff sensor in the model description!"
                     << " Did you specify it?" << " [" << node_name_ <<"]");
    return false;
  }
  if (sdf_->HasElement("cliff_FR_sensor_name")){
    cliff_sensor_FR_name = sdf_->GetElement("cliff_FR_sensor_name")->Get<std::string>();
  }else{
    ROS_ERROR_STREAM("Couldn't find the name of FR cliff sensor in the model description!"
                     << " Did you specify it?" << " [" << node_name_ <<"]");
    return false;
  }
  if (sdf_->HasElement("cliff_BL_sensor_name")){
    cliff_sensor_BL_name = sdf_->GetElement("cliff_BL_sensor_name")->Get<std::string>();
  }else{
    ROS_ERROR_STREAM("Couldn't find the name of BL cliff sensor in the model description!"
                     << " Did you specify it?" << " [" << node_name_ <<"]");
    return false;
  }
  if (sdf_->HasElement("cliff_BR_sensor_name")){
    cliff_sensor_BR_name = sdf_->GetElement("cliff_BR_sensor_name")->Get<std::string>();
  }else{
    ROS_ERROR_STREAM("Couldn't find the name of BR cliff sensor in the model description!"
                     << " Did you specify it?" << " [" << node_name_ <<"]");
    return false;
  }
  // dynamic_cast: 多态时把实际指向派生类的基类指针转换成派生类指针，如果指针是智能指针，需要使用dynamic_pointer_cast
  // Instance:静态局部变量的方式创建单例类
  cliff_FL_sensor_ = std::dynamic_pointer_cast<sensors::RaySensor>(
                       sensors::SensorManager::Instance()->GetSensor(cliff_sensor_FL_name));
  cliff_FR_sensor_ = std::dynamic_pointer_cast<sensors::RaySensor>(
                        sensors::SensorManager::Instance()->GetSensor(cliff_sensor_FR_name));
  cliff_BL_sensor_ = std::dynamic_pointer_cast<sensors::RaySensor>(
                        sensors::SensorManager::Instance()->GetSensor(cliff_sensor_BL_name));
  cliff_BR_sensor_ = std::dynamic_pointer_cast<sensors::RaySensor>(
                        sensors::SensorManager::Instance()->GetSensor(cliff_sensor_BR_name));
  if (!cliff_FL_sensor_){
    ROS_ERROR_STREAM("Couldn't find the FL cliff sensor in the model! [" << node_name_ <<"]");
    return false;
  }
  if (!cliff_FR_sensor_){
    ROS_ERROR_STREAM("Couldn't find the FR cliff sensor in the model! [" << node_name_ <<"]");
    return false;
  }
  if (!cliff_BL_sensor_){
    ROS_ERROR_STREAM("Couldn't find the BL cliff sensor in the model! [" << node_name_ <<"]");
    return false;
  }
  if (!cliff_BR_sensor_){
    ROS_ERROR_STREAM("Couldn't find the BR cliff sensor in the model! [" << node_name_ <<"]");
    return false;
  }
  if (sdf_->HasElement("cliff_detection_threshold")){
    cliff_detection_threshold_ = sdf_->GetElement("cliff_detection_threshold")->Get<double>();
  }else{
    ROS_ERROR_STREAM("Couldn't find the cliff detection threshold parameter in the model description!"
                     << " Did you specify it?" << " [" << node_name_ <<"]");
    return false;
  }
  cliff_FL_sensor_->SetActive(true);
  cliff_FR_sensor_->SetActive(true);
  cliff_BL_sensor_->SetActive(true);
  cliff_BR_sensor_->SetActive(true);

  cliff_detected_FL_ = cliff_detected_FR_ = cliff_detected_BL_ = cliff_detected_BR_ = false;
  cliff_auto_stop_motor_flag = false;

  return true;
}

bool GazeboRosRbn100::prepareBumper(){
  std::string bumper_name;
  if (sdf_->HasElement("bumper_sensor_name")){
    bumper_name = sdf_->GetElement("bumper_sensor_name")->Get<std::string>();
  }else{
    ROS_ERROR_STREAM("Couldn't find the name of bumper sensor in the model description!"
                     << " Did you specify it?" << " [" << node_name_ <<"]");
    return false;
  }

  contact_bumper_ = std::dynamic_pointer_cast<sensors::ContactSensor>(
            sensors::SensorManager::Instance()->GetSensor(bumper_name));
  if (!contact_bumper_){
    ROS_ERROR_STREAM("Couldn't find the bumpers in the model! [" << node_name_ <<"]");
    return false;
  }
  ROS_INFO_STREAM("[load] bumper sensor name: " << contact_bumper_->Name());
  contact_bumper_->SetActive(true);
  bumper_was_pressed_ = false;
  bumper_auto_stop_motor_flag = false;
  return true;
}

/* bool GazeboRosRbn100::prepareIMU(){
  if (sdf_->HasElement("imu_name")){
    imu_name_ = sdf_->GetElement("imu_name")->Get<std::string>();
  }else{
    ROS_ERROR_STREAM("Couldn't find the name of IMU sensor in the model description!"
                     << " Did you specify it?" << " [" << node_name_ <<"]");
    return false;
  }
  if (!sdf_->HasElement("imu_rate")){
    ROS_DEBUG_NAMED("imu", "imu plugin missing <bumper_rate>, defaults to 0.0"
             " (as fast as possible)");
    imu_rate_ = 0.0;
  }else
    imu_rate_ = sdf_->GetElement("imu_rate")->Get<double>();

  imu_period_ = 1.0 / imu_rate_;

  imu_ = std::dynamic_pointer_cast<sensors::ImuSensor>(
      sensors::get_sensor(world_->Name()+"::"+node_name_+"::base_footprint::"+imu_name_));
  if (!imu_){
    ROS_ERROR_STREAM("Couldn't find the IMU in the model! [" << node_name_ <<"]");
    return false;
  }
  imu_->SetActive(true);
  return true;
} */

bool GazeboRosRbn100::prepareUltra(){
  std::string sonar_FL_name, sonar_front_name, sonar_FR_name, sonar_back_name;
  if(sdf_->HasElement("sonar_FL_sensor_name")){
    sonar_FL_name = sdf_->GetElement("sonar_FL_sensor_name")->Get<std::string>();
  }else{
    ROS_WARN_STREAM("Couldn't found sonar_Fl_name element in model description, please check!");
    return false;
  }

  if(sdf_->HasElement("sonar_front_sensor_name")){
    sonar_front_name = sdf_->GetElement("sonar_front_sensor_name")->Get<std::string>();
  }else{
    ROS_WARN_STREAM("Couldn't find sonar_front_name element in model description, please check!");
    return false;
  }
  
  if(sdf_->HasElement("sonar_FR_sensor_name")){
    sonar_FR_name = sdf_->GetElement("sonar_FR_sensor_name")->Get<std::string>();
  }else{
    ROS_WARN_STREAM("Couldn't find sonar_FR_name element in model decription, please check!");
    return false;
  }

  if(sdf_->HasElement("sonar_back_sensor_name")){
    sonar_back_name = sdf_->GetElement("sonar_back_sensor_name")->Get<std::string>();
  }else{
    ROS_WARN_STREAM("Couldn't find sonar_back_name element in model description, please check!");
    return false;
  }

  if(sdf_->HasElement("sonar_noise")){
    sonar_noise_ = sdf_->GetElement("sonar_noise")->Get<double>();
  }else{
    ROS_WARN_STREAM("Couldn't find sonar_noise element in model description, please check");
  }
 
  double sonar_rate;
  if(sdf_->HasElement("sonar_rate")){
    sonar_rate = sdf_->GetElement("sonar_rate")->Get<int>();
  }else{
    ROS_WARN_STREAM("Couldn't find sonar_rate element in model description, please check");
  }
  sonar_period_ = 1.0 / sonar_rate;

  sonar_FL_sensor_ = std::dynamic_pointer_cast<sensors::RaySensor>(
      sensors::SensorManager::Instance()->GetSensor(sonar_FL_name));
  sonar_front_sensor_ = std::dynamic_pointer_cast<sensors::RaySensor>(
      sensors::SensorManager::Instance()->GetSensor(sonar_front_name));
  sonar_FR_sensor_ = std::dynamic_pointer_cast<sensors::RaySensor>(
      sensors::SensorManager::Instance()->GetSensor(sonar_FR_name));
  sonar_back_sensor_ = std::dynamic_pointer_cast<sensors::RaySensor>(
      sensors::SensorManager::Instance()->GetSensor(sonar_back_name));

  if(!sonar_FL_sensor_){
    ROS_ERROR_STREAM("Couldn't find the sonar_FL sensor in the model, please check!");
    return false;
  }
  if(!sonar_front_sensor_){
    ROS_ERROR_STREAM("Couldn't find the sonar_front sensor in the model, please check!");
    return false;
  }
  if(!sonar_FR_sensor_){
    ROS_ERROR_STREAM("Couldn't find the sonar_FR sensor in the model, please check!");
    return false;
  }
  if(!sonar_back_sensor_){
    ROS_ERROR_STREAM("Couldn't find the sonar_back sensor in the model, please check!");
    return false;
  }

  samples_ = sonar_FL_sensor_->LaserShape()->GetSampleCount() * 
      sonar_FL_sensor_->LaserShape()->GetVerticalSampleCount();
  range_max_ = sonar_FL_sensor_->RangeMax();
  range_min_ = sonar_FL_sensor_->RangeMin();
  
  sonar_FL_sensor_->SetActive(true);
  sonar_front_sensor_->SetActive(true);
  sonar_FR_sensor_->SetActive(true);
  sonar_back_sensor_->SetActive(true);

  return true;
}

bool GazeboRosRbn100::prepareStereoCamera(){
  std::string camera_left_name, camera_right_name;
  if(sdf_->HasElement("camera_left_sensor_name")){
    camera_left_name = sdf_->GetElement("camera_left_sensor_name")->Get<std::string>();
  }else{
    ROS_ERROR_STREAM("image: lack of camera_left_name element.");
    return false;
  }
  if(sdf_->HasElement("camera_right_sensor_name")){
    camera_right_name = sdf_->GetElement("camera_right_sensor_name")->Get<std::string>();
  }else{
    ROS_ERROR_STREAM("image: lack of camera_right_name element.");
    return false;
  }
  
  #ifdef ENABLE_PUBLIC_CAMERAS
    camera_left_topic_ = camera_right_topic_ = "";
    if(sdf_->HasElement("camera_left_topic"))
      camera_left_topic_ = sdf_->GetElement("camera_left_topic")->Get<std::string>();
    if(sdf_->HasElement("camera_right_topic"))
      camera_right_topic_ = sdf_->GetElement("camera_right_topic")->Get<std::string>();

    if(sdf_->HasElement("camera_left_frame")){
      camera_left_frame_ = sdf_->GetElement("camera_left_frame")->Get<std::string>();
    }else{
      ROS_ERROR_STREAM("image: lack of camera_left_frame element.");
      return false;
    }
    if(sdf_->HasElement("camera_right_frame")){
      camera_right_frame_ = sdf_->GetElement("camera_right_frame")->Get<std::string>();
    }else{
      ROS_ERROR_STREAM("image: lack of camera_right_frame element.");
      return false;
    }
  #endif

  double camera_rate;
  if(sdf_->HasElement("camera_rate")){
    camera_rate = sdf_->GetElement("camera_rate")->Get<double>();
  }else{
    ROS_INFO_STREAM("image: lack of camera_rate element, rate default to 25");
    camera_rate = 25;
  }
  camera_period_ = 1.0 / camera_rate;

  sensors::WideAngleCameraSensorPtr left_camera, right_camera;
  sensors::SensorManager *smanager = sensors::SensorManager::Instance();
  left_camera = std::dynamic_pointer_cast<sensors::WideAngleCameraSensor>(
    smanager->GetSensor(camera_left_name));
  right_camera = std::dynamic_pointer_cast<sensors::WideAngleCameraSensor>(
    smanager->GetSensor(camera_right_name));

  if (!left_camera) {
    ROS_WARN_STREAM("RealSensePlugin: fisheeye Camera left has not been found");
    return false;
  }
  if (!right_camera) {
    ROS_WARN_STREAM("RealSensePlugin: fisheye Camera right has not been found");
    return false;
  }

  left_camera->SetActive(true);
  right_camera->SetActive(true);

  leftCam_ = left_camera->Camera();
  rightCam_ = right_camera->Camera();

  newLeftImgFrameConn_ = leftCam_->ConnectNewImageFrame(std::bind(
      &GazeboRosRbn100::OnNewCameraFrameLeft, this));
  newRightImgFrameConn_ = rightCam_->ConnectNewImageFrame(std::bind(
      &GazeboRosRbn100::OnNewCameraFrameRight, this));

  return true;
}

void GazeboRosRbn100::setupRosApi(const GazeboRosPtr& gazebo_ros){
  // std::string base_prefix;
  // gazebo_ros_->node()->param("base_prefix", base_prefix, std::string("mobile_base"));
  
/* 
  publisher（以指定频率往共享发布队列中发布数据，队列长度为1保证数据都是最新的）
*/
  // pub of joint_states
  std::string joint_states_topic = "joint_states";
  joint_state_pub_ = gazebo_ros->node()->advertise<sensor_msgs::JointState>(joint_states_topic, 1);
  ROS_INFO("%s: Advertise joint_states[%s]!", gazebo_ros->info(), joint_states_topic.c_str());

  // pub of odom
  std::string odom_topic = odom_topic_;
  odom_pub_ = gazebo_ros->node()->advertise<rbn100_msgs::Encoder>(odom_topic, 1);
  ROS_INFO("%s: Advertise Odometry[%s]!", gazebo_ros->info(), odom_topic.c_str());

  // pub of cliff
  std::string cliff_topic = "/events/cliff";
  cliff_event_pub_ = gazebo_ros->node()->advertise<rbn100_msgs::CliffEvent>(cliff_topic, 1);
  ROS_INFO("%s: Advertise Cliff[%s]!", gazebo_ros->info(), cliff_topic.c_str());

  // pub of bumper
  std::string bumper_topic = "/events/bumper";
  bumper_event_pub_ = gazebo_ros->node()->advertise<rbn100_msgs::BumperEvent>(bumper_topic, 1);
  ROS_INFO("%s: Advertise Bumper[%s]!", gazebo_ros->info(), bumper_topic.c_str());

  // pub of ultra
  std::string ultra_topic = "/simultra";
  ultra_pub_ = gazebo_ros->node()->advertise<rbn100_msgs::Ultra>(ultra_topic, 1);
  ROS_INFO("%s: Advertise Ultra[%s]!", gazebo_ros->info(), ultra_topic.c_str());

  // pub of camera
  #ifdef ENABLE_PUBLIC_CAMERAS
    itnode_ = new image_transport::ImageTransport(*(gazebo_ros->node()));
    leftImg_pub_ = itnode_->advertiseCamera(camera_left_topic_, 1);
    ROS_INFO("%s: Advertise Camera[%s]!", gazebo_ros->info(), camera_left_topic_.c_str());
    rightImg_pub_ = itnode_->advertiseCamera(camera_right_topic_, 1);
    ROS_INFO("%s: Advertise Camera[%s]!", gazebo_ros->info(), camera_right_topic_.c_str());
  #endif
  std::string camera_topic = "/camera/stereo_image";
  stereo_image_pub_ = gazebo_ros->node()->advertise<rbn100_msgs::StereoImage>(camera_topic, 1);
  ROS_INFO("%s: Advertise Camera[%s]!", gazebo_ros->info(), camera_topic.c_str());

  std::string wheel_speed_topic = "/wheel_speed";
  wheel_speed_pub_ = gazebo_ros->node()->advertise<rbn100_msgs::WheelSpeed>(wheel_speed_topic, 1);
  ROS_INFO("%s: Advertise WheelSpeed[%s]!", gazebo_ros->info(), wheel_speed_topic.c_str());

/* 
  subscriber（从发布队列中取数据放入共享订阅队列供回调函数使用，没有订阅频率，快慢看机器，
              回调函数尽量简短，以保证信息处理及时，spin和spinonce都是单线程顺序查看
              队列中的所有信息，多线程可以分topic查看）
*/
  // sub of motor power
  std::string motor_power_topic = "/commands/motor_power";
  motor_power_sub_ = gazebo_ros->node()->subscribe(motor_power_topic, 10, &GazeboRosRbn100::motorPowerCB, this);
  ROS_INFO("%s: Try to subscribe to %s!", gazebo_ros->info(), motor_power_topic.c_str());

  // sub of odom reset
  std::string odom_reset_topic = "/commands/reset_odometry";
  odom_reset_sub_ = gazebo_ros->node()->subscribe(odom_reset_topic, 10, &GazeboRosRbn100::resetOdomCB, this);
  ROS_INFO("%s: Try to subscribe to %s!", gazebo_ros->info(), odom_reset_topic.c_str());

  // sub of cmd_vel
  std::string cmd_vel_topic = "/commands/velocity";
  cmd_vel_sub_ = gazebo_ros->node()->subscribe(cmd_vel_topic, 100, &GazeboRosRbn100::cmdVelCB, this);
  ROS_INFO("%s: Try to subscribe to %s!", gazebo_ros->info(), cmd_vel_topic.c_str());

  // sub of enable bumper auto stop motor
  std::string bumper_auto_stop_motor_topic = "/commands/bumper_auto_stop_motor";
  bumper_auto_stop_motor_sub_ = gazebo_ros->node()->subscribe(bumper_auto_stop_motor_topic, 10, &GazeboRosRbn100::bumperAutoStopMotorCB, this);
  ROS_INFO("%s: Try to subscribe to %s!", gazebo_ros->info(), bumper_auto_stop_motor_topic.c_str());
  
  // sub of enable cliff auto stop motor
  std::string cliff_auto_stop_motor_topic = "/commands/cliff_auto_stop_motor";
  cliff_auto_stop_motor_sub_ = gazebo_ros->node()->subscribe(cliff_auto_stop_motor_topic, 10, &GazeboRosRbn100::cliffAutoStopMotorCB, this);
  ROS_INFO("%s: Try to subscribe to %s!", gazebo_ros->info(), cliff_auto_stop_motor_topic.c_str());
}

/* 
  call backs
 */
void GazeboRosRbn100::motorPowerCB(const rbn100_msgs::MotorPowerPtr &msg){
  // if ((msg->state == rbn100_msgs::MotorPower::ON) && (!motors_enabled_))
  if (msg->state == rbn100_msgs::MotorPower::ON){
    motors_enabled_ = true;
    ROS_INFO_STREAM("motorPowerCB: Motors fired up. [" << node_name_ << "]");
  }else if (msg->state == rbn100_msgs::MotorPower::OFF){
    motors_enabled_ = false;
    ROS_INFO_STREAM("motorPowerCB: Motors taking a rest. [" << node_name_ << "]");
  }
}

void GazeboRosRbn100::resetOdomCB(const std_msgs::EmptyConstPtr &msg){
  ROS_INFO_STREAM("Reset Odom CB!");

  odom_msg_.EncoderL = 0;
  odom_msg_.EncoderR = 0;
}

void GazeboRosRbn100::cmdVelCB(const geometry_msgs::TwistConstPtr &msg){
  last_cmd_vel_time_ = world_->SimTime();

  // 小车线速度、角速度、轮间距和两轮速度的关系
  wheel_speed_cmd_[LEFT] = msg->linear.x - msg->angular.z * (wheel_sep_) / 2;
  wheel_speed_cmd_[RIGHT] = msg->linear.x + msg->angular.z * (wheel_sep_) / 2;
}

void GazeboRosRbn100::bumperAutoStopMotorCB(const std_msgs::BoolPtr &msg){
   bumper_auto_stop_motor_flag = msg->data;
   ROS_INFO_STREAM("Enable flag of bumper auto brake: "<< bumper_auto_stop_motor_flag);
}

void GazeboRosRbn100::cliffAutoStopMotorCB(const std_msgs::BoolPtr &msg){
   cliff_auto_stop_motor_flag = msg->data;
   ROS_INFO_STREAM("Enable flag for cliff auto brake: "<< cliff_auto_stop_motor_flag);
}

} // end of namespace gazebo
