#include "rbn100_gazebo_plugins/gazebo_ros_rbn100.h"

sensor_msgs::CameraInfo cameraInfoLeft(const sensor_msgs::Image &image,
                                   float horizontal_fov) {
  sensor_msgs::CameraInfo info_msg;

  info_msg.header = image.header;
  info_msg.distortion_model = "customized_fisheye";
  info_msg.height = image.height;
  info_msg.width = image.width;

  float focal = image.width / horizontal_fov;

  info_msg.K[0] = 378.332247;
  info_msg.K[2] = 320.818260;
  info_msg.K[4] = 378.445198;
  info_msg.K[5] = 198.263230;
  info_msg.K[8] = 1;

  info_msg.D.resize(5);
  info_msg.D[0] = -0.294920;
  info_msg.D[1] = 0.076752;
  info_msg.D[2] = 0.001398;
  info_msg.D[3] = -0.000121;

  info_msg.R[0] = 0.999714;
  info_msg.R[1] = -0.000584;
  info_msg.R[2] = -0.023904;
  info_msg.R[3] = 0.000537;
  info_msg.R[4] = 0.999998;
  info_msg.R[5] = -0.001981;
  info_msg.R[6] = 0.023905;
  info_msg.R[7] = 0.001967;
  info_msg.R[8] = 0.999712;

  info_msg.P[0] = 346.888892;
  info_msg.P[2] = 344.434708;
  info_msg.P[5] = 346.888892;
  info_msg.P[6] = 200.464249;
  info_msg.P[10] = 1;

  //    info_msg.roi.do_rectify = true;

  return info_msg;
}
sensor_msgs::CameraInfo cameraInfoRight(const sensor_msgs::Image &image,
                                   float horizontal_fov) {
  sensor_msgs::CameraInfo info_msg;

  info_msg.header = image.header;
  info_msg.distortion_model = "customized_fisheye";
  info_msg.height = image.height;
  info_msg.width = image.width;

  float focal = image.width / horizontal_fov;

  info_msg.K[0] = 378.767104;
  info_msg.K[2] = 316.847613;
  info_msg.K[4] = 379.132544;
  info_msg.K[5] = 199.721362;
  info_msg.K[8] = 1;

  info_msg.D.resize(5);
  info_msg.D[0] = -0.295472;
  info_msg.D[1] = 0.077876;
  info_msg.D[2] = 0.000952;
  info_msg.D[3] = -0.000338;

  info_msg.R[0] = 0.999452;
  info_msg.R[1] = -0.000464;
  info_msg.R[2] = -0.033105;
  info_msg.R[3] = 0.000529;
  info_msg.R[4] = 0.999998;
  info_msg.R[5] = 0.001966;
  info_msg.R[6] = 0.033104;
  info_msg.R[7] = -0.001982;
  info_msg.R[8] = 0.999450;

  info_msg.P[0] = 346.888892;
  info_msg.P[2] = 344.434708;
  // info_msg.P[3] = -41.011652;
  info_msg.P[5] = 346.888892;
  info_msg.P[6] = 200.464249;
  info_msg.P[10] = 1;

  //    info_msg.roi.do_rectify = true;

  return info_msg;
}

namespace gazebo 
{

/*
 * Propagate velocity commands
 * TODO: Check how to simulate disabled motors, e.g. set MaxForce to zero, but then damping is important!
 */
void GazeboRosRbn100::propagateVelocityCommands()
{
  if (((prev_update_time_ - last_cmd_vel_time_).Double() > cmd_vel_timeout_))
  {
    wheel_speed_cmd_[LEFT] = 0.0;
    wheel_speed_cmd_[RIGHT] = 0.0;
    ROS_INFO_STREAM_THROTTLE(5, "[update] vel cmd time out, vel set to 0");
  }
  if(!motors_enabled_){
    wheel_speed_cmd_[LEFT] = 0.0;
    wheel_speed_cmd_[RIGHT] = 0.0;
    ROS_INFO_STREAM_THROTTLE(5, "[update] motors down, vel set to 0");
  }
  wheel_joints_[LEFT]->SetVelocity(0, wheel_speed_cmd_[LEFT] / (wheel_diam_ / 2.0));
  wheel_joints_[RIGHT]->SetVelocity(0, wheel_speed_cmd_[RIGHT] / (wheel_diam_ / 2.0));
}

void GazeboRosRbn100::updateJointState()
{
  sensor_msgs::JointState joint_state_msg;
  // std::string baselink_frame = gazebo_ros_->resolveTF("base_link");
  joint_state_msg.header.frame_id = "joint_state";
  joint_state_msg.header.stamp = ros::Time::now();

  joint_state_msg.name.resize(WHEEL_JOINT_NUM);
  joint_state_msg.position.resize(WHEEL_JOINT_NUM);
  joint_state_msg.velocity.resize(WHEEL_JOINT_NUM);
  joint_state_msg.effort.resize(WHEEL_JOINT_NUM);

  joint_state_msg.name[LEFT] = wheel_joints_[LEFT]->GetName();
  joint_state_msg.name[RIGHT] = wheel_joints_[RIGHT]->GetName();
  joint_state_msg.position[LEFT] = wheel_joints_[LEFT]->Position(0);
  joint_state_msg.position[RIGHT] = wheel_joints_[RIGHT]->Position(0);
  joint_state_msg.velocity[LEFT] = wheel_joints_[LEFT]->GetVelocity(0);
  joint_state_msg.velocity[RIGHT] = wheel_joints_[RIGHT]->GetVelocity(0);

  joint_state_pub_.publish(joint_state_msg);
}

void GazeboRosRbn100::updateOdometry(common::Time& step_time)
{
  odom_msg_.header.frame_id = odom_topic_;
  odom_msg_.header.stamp = ros::Time::now();

  double d1 = step_time.Double() * (wheel_diam_ / 2) * wheel_joints_[LEFT]->GetVelocity(0);
  double d2 = step_time.Double() * (wheel_diam_ / 2) * wheel_joints_[RIGHT]->GetVelocity(0);
  
  odom_msg_.EncoderL += (uint64_t)(d1 / m_per_encoder_ * 1e5);
  odom_msg_.EncoderR += (uint64_t)(d2 / m_per_encoder_ * 1e5);
  odom_pub_.publish(odom_msg_);
}

/*
 * Cliff sensors
 * Check each sensor separately
 */
void GazeboRosRbn100::updateCliffSensor()
{
  rbn100_msgs::CliffEvent cliff_event;
  cliff_event.header.frame_id = "cliff";
  cliff_event.header.stamp = ros::Time::now();
  // FL cliff sensor
  if ((cliff_detected_FL_ == false) &&
      (cliff_FL_sensor_->Range(0) >= cliff_detection_threshold_))
  {
    ROS_INFO_STREAM("cliff_FL detected");
    if(cliff_auto_stop_motor_flag){
      ROS_INFO_STREAM("enable cliff stop motor");
      motors_enabled_ = false;
    }
    cliff_detected_FL_ = true;
    cliff_event.which = rbn100_msgs::CliffEvent::FL;
    cliff_event.state = rbn100_msgs::CliffEvent::CLIFF;
    // convert distance back to an AD reading
    cliff_event.bottom = (int)(76123.0f * atan2(0.995f, cliff_FL_sensor_->Range(0)));
    ROS_INFO_STREAM("[cliff] bottom and range[0]: " << cliff_event.bottom << " " << cliff_FL_sensor_->Range(0));
    cliff_event_pub_.publish(cliff_event);
  }
  else if ((cliff_detected_FL_ == true) && 
           (cliff_FL_sensor_->Range(0) < cliff_detection_threshold_))
  {
    ROS_INFO_STREAM("cliff_FL normal");
    cliff_detected_FL_ = false;
    cliff_event.which = rbn100_msgs::CliffEvent::FL;
    cliff_event.state = rbn100_msgs::CliffEvent::FLOOR;
    cliff_event.bottom = (int)(76123.0f * atan2(0.995f, cliff_FL_sensor_->Range(0)));
    cliff_event_pub_.publish(cliff_event);
  }
  // FR cliff sensor
  if ((cliff_detected_FR_ == false) &&
      (cliff_FR_sensor_->Range(0) >= cliff_detection_threshold_))
  {
    ROS_INFO_STREAM("cliff_FR detected");
    if(cliff_auto_stop_motor_flag){
      ROS_INFO_STREAM("enable cliff stop motor");
      motors_enabled_ = false;
    }
    cliff_detected_FR_ = true;
    cliff_event.which = rbn100_msgs::CliffEvent::FR;
    cliff_event.state = rbn100_msgs::CliffEvent::CLIFF;
    cliff_event.bottom = (int)(76123.0f * atan2(0.995f, cliff_FR_sensor_->Range(0)));
    cliff_event_pub_.publish(cliff_event);
  }
  else if ((cliff_detected_FR_ == true) &&
          (cliff_FR_sensor_->Range(0) < cliff_detection_threshold_))
  {
    ROS_INFO_STREAM("cliff_FR normal");
    cliff_detected_FR_ = false;
    cliff_event.which = rbn100_msgs::CliffEvent::FR;
    cliff_event.state = rbn100_msgs::CliffEvent::FLOOR;
    cliff_event.bottom = (int)(76123.0f * atan2(0.995f, cliff_FR_sensor_->Range(0)));
    cliff_event_pub_.publish(cliff_event);
  }
  // BL cliff sensor
  if ((cliff_detected_BL_ == false) &&
      (cliff_BL_sensor_->Range(0) >= cliff_detection_threshold_))
  {
    ROS_INFO_STREAM("cliff_BL detected");
    if(cliff_auto_stop_motor_flag){
      ROS_INFO_STREAM("enable cliff stop motor");
      motors_enabled_ = false;
    }
    cliff_detected_BL_ = true;
    cliff_event.which = rbn100_msgs::CliffEvent::BL;
    cliff_event.state = rbn100_msgs::CliffEvent::CLIFF;
    cliff_event.bottom = (int)(76123.0f * atan2(0.995f, cliff_BL_sensor_->Range(0)));
    cliff_event_pub_.publish(cliff_event);
  }
  else if ((cliff_detected_BL_ == true) &&
          (cliff_BL_sensor_->Range(0) < cliff_detection_threshold_))
  {
    ROS_INFO_STREAM("cliff_BL normal");
    cliff_detected_BL_ = false;
    cliff_event.which = rbn100_msgs::CliffEvent::BL;
    cliff_event.state = rbn100_msgs::CliffEvent::FLOOR;
    cliff_event.bottom = (int)(76123.0f * atan2(0.995f, cliff_BL_sensor_->Range(0)));
    cliff_event_pub_.publish(cliff_event);
  }
  // BR cliff sensor
  if ((cliff_detected_BR_ == false) &&
      (cliff_BR_sensor_->Range(0) >= cliff_detection_threshold_))
  {
    ROS_INFO_STREAM("cliff_BR detected");
    if(cliff_auto_stop_motor_flag){
      ROS_INFO_STREAM("enable cliff stop motor");
      motors_enabled_ = false;
    }
    cliff_detected_BR_ = true;
    cliff_event.which = rbn100_msgs::CliffEvent::BR;
    cliff_event.state = rbn100_msgs::CliffEvent::CLIFF;
    cliff_event.bottom = (int)(76123.0f * atan2(0.995f, cliff_BR_sensor_->Range(0)));
    cliff_event_pub_.publish(cliff_event);
  }
  else if ((cliff_detected_BR_ == true) &&
          (cliff_BR_sensor_->Range(0) < cliff_detection_threshold_))
  {
    ROS_INFO_STREAM("cliff_BR normal");
    cliff_detected_BR_ = false;
    cliff_event.which = rbn100_msgs::CliffEvent::BR;
    cliff_event.state = rbn100_msgs::CliffEvent::FLOOR;
    cliff_event.bottom = (int)(76123.0f * atan2(0.995f, cliff_BR_sensor_->Range(0)));
    cliff_event_pub_.publish(cliff_event);
  }
}

void GazeboRosRbn100::updateUltra(){
  rbn100_msgs::Ultra ultra_msg;
  ultra_msg.header.stamp = ros::Time::now();
  // every iteration, init distance to max
  double range_FL, range_front, range_FR, range_back;
  range_FL = range_front = range_FR = range_back = range_max_;
  // read range of all samples
  for (int i = 0; i < samples_; i++){
    // read range of a ray
    double ray_FL = sonar_FL_sensor_->LaserShape()->GetRange(i);
    double ray_front = sonar_front_sensor_->LaserShape()->GetRange(i);
    double ray_FR = sonar_FR_sensor_->LaserShape()->GetRange(i);
    double ray_back = sonar_back_sensor_->LaserShape()->GetRange(i);
    // get the minest value
    if(ray_FL < range_FL)
      range_FL = ray_FL;
    if(ray_front < range_front)
      range_front = ray_front;
    if(ray_FR < range_FR)
      range_FR = ray_FR;
    if(ray_back < range_back)
      range_back = ray_back;
  }
  // add noise and limit to [min, max] range
  range_FL = range_FL + GaussianKernel(0, sonar_noise_);
  if(range_FL > range_max_){
    range_FL = range_max_;
  }else if(range_FL < range_min_){
    range_FL = range_min_;
  }

  range_front = range_front + GaussianKernel(0, sonar_noise_);
  if(range_front > range_max_){
    range_front = range_max_;
  }else if(range_front < range_min_){
    range_front = range_min_;
  }

  range_FR = range_FR + GaussianKernel(0, sonar_noise_);
  if(range_FR > range_max_){
    range_FR = range_max_;
  }else if(range_front < range_min_){
    range_FR = range_min_;
  }
  
  range_back = range_back + GaussianKernel(0, sonar_noise_);
  if(range_back > range_max_){
    range_back = range_max_;
  }else if(range_back < range_min_){
    range_back = range_min_;
  }

  ultra_msg.distance[0] = range_FL;
  ultra_msg.distance[1] = range_front;
  ultra_msg.distance[2] = range_FR;
  ultra_msg.distance[3] = range_back;

  ultra_pub_.publish(ultra_msg);
}

void GazeboRosRbn100::updateBumper()
{
  // 初始化当前状态标志
  bumper_is_pressed_ = false;
  // 判断当前状态
  msgs::Contacts contacts = contact_bumper_->Contacts();
  if(contacts.contact_size() >= 1){
    bumper_is_pressed_ = true;
  }
  // 基于topic特性，和历史状态标志一起完成事件机制
  // 当前触发+历史不触发才算一次触发事件，其他状态无动作
  if (bumper_is_pressed_ && !bumper_was_pressed_)
  {
    ROS_INFO_STREAM("bumper triggered");
    if(bumper_auto_stop_motor_flag){
      ROS_INFO_STREAM("enable bumper stop motor");
      motors_enabled_ = false;
    }
    // 更改历史状态为触发
    bumper_was_pressed_ = true;

    rbn100_msgs::BumperEvent bumper_event;
    bumper_event.state = rbn100_msgs::BumperEvent::PRESSED;
    bumper_event.bumper = rbn100_msgs::BumperEvent::body;
    bumper_event.header.frame_id = "bumper";
    bumper_event.header.stamp = ros::Time::now();
    bumper_event_pub_.publish(bumper_event);
  } // 当前不触发+历史触发才算一次正常事件，其他状态无动作
  else if (!bumper_is_pressed_ && bumper_was_pressed_)
  {
    ROS_INFO_STREAM("bumper normal");
    // 更改历史状态为正常
    bumper_was_pressed_ = false;

    rbn100_msgs::BumperEvent bumper_event;
    bumper_event.state = rbn100_msgs::BumperEvent::RELEASED;
    bumper_event.bumper = rbn100_msgs::BumperEvent::body;
    bumper_event.header.frame_id = "bumper";
    bumper_event.header.stamp = ros::Time::now();
    bumper_event_pub_.publish(bumper_event);
  }
}

void GazeboRosRbn100::updateWheelSpeed(){
  rbn100_msgs::WheelSpeed wheel_speed_msg;
  wheel_speed_msg.timeStamp = ros::Time::now().toNSec();
  wheel_speed_msg.leftSpeed = wheel_speed_cmd_[LEFT];
  wheel_speed_msg.rightSpeed = wheel_speed_cmd_[RIGHT];
  wheel_speed_pub_.publish(wheel_speed_msg);
}

void GazeboRosRbn100::OnNewCameraFrameLeft(){
  common::Time current_time = world_->SimTime();
  leftImg_msg_.header.stamp.sec = current_time.sec;
  leftImg_msg_.header.stamp.nsec = current_time.nsec;

  std::string format = sensor_msgs::image_encodings::MONO8;
  leftImg_msg_.encoding = format;
  int step = leftCam_->ImageDepth();
  leftImg_msg_.step = step;
  int height = leftCam_->ImageHeight();
  int width = leftCam_->ImageWidth();
  if(leftCam_->ImageFormat() != "L8")
    ROS_ERROR_STREAM("other formats is not supported but L8, please contact the developer.");

  sensor_msgs::fillImage(leftImg_msg_,
                         format,
                         height,
                         width,
                         step * width,
                         leftCam_->ImageData());
                        //  reinterpret_cast<const void *>(leftCam_->ImageData()));
  #ifdef ENABLE_PUBLIC_CAMERAS
    leftImg_msg_.header.frame_id = camera_left_frame_;
    auto camera_info_msg = cameraInfoLeft(leftImg_msg_, leftCam_->HFOV().Radian());
    leftImg_pub_.publish(leftImg_msg_, camera_info_msg);
    leftImg_msg_.data.clear();
  #endif
}

void GazeboRosRbn100::OnNewCameraFrameRight(){
  common::Time current_time = world_->SimTime();
  rightImg_msg_.header.stamp.sec = current_time.sec;
  rightImg_msg_.header.stamp.nsec = current_time.nsec;

  std::string format = sensor_msgs::image_encodings::MONO8;
  if(rightCam_->ImageFormat() != "L8")
    ROS_ERROR_STREAM("other formats is not supported but L8, please contact the developer.");

  sensor_msgs::fillImage(rightImg_msg_,
                         format,
                         rightCam_->ImageHeight(),
                         rightCam_->ImageWidth(),
                         rightCam_->ImageDepth() * rightCam_->ImageWidth(),
                         rightCam_->ImageData());
  #ifdef ENABLE_PUBLIC_CAMERAS
    rightImg_msg_.header.frame_id = camera_right_frame_;
    auto camera_info_msg = cameraInfoRight(rightImg_msg_, rightCam_->HFOV().Radian());
    rightImg_pub_.publish(rightImg_msg_, camera_info_msg);
    rightImg_msg_.data.clear();
  #endif
}

void GazeboRosRbn100::updateStereoCamera(){
    rbn100_msgs::StereoImage cam_msg;
    cam_msg.header.frame_id = "stereo_camera";
    cam_msg.header.stamp = ros::Time::now();
    cam_msg.width = leftImg_msg_.width;
    cam_msg.height = leftImg_msg_.height;
    int size = leftImg_msg_.step * leftImg_msg_.height;
    
    cam_msg.left.resize(size);
    cam_msg.right.resize(size);
    memcpy(&cam_msg.left[0], &leftImg_msg_.data[0], size);
    memcpy(&cam_msg.right[0], &rightImg_msg_.data[0], size);
    stereo_image_pub_.publish(cam_msg);
}

// Utility for adding noise
double GazeboRosRbn100::GaussianKernel(double mu, double sigma)
{
  // using Box-Muller transform to generate two independent standard
  // normally disbributed normal variables see wikipedia

  // normalized uniform random variable
  double U = ignition::math::Rand::DblUniform();

  // normalized uniform random variable
  double V = ignition::math::Rand::DblUniform();

  double X = sqrt(-2.0 * ::log(U)) * cos(2.0*M_PI * V);
  // double Y = sqrt(-2.0 * ::log(U)) * sin(2.0*M_PI * V);

  // there are 2 indep. vars, we'll just use X
  // scale to our mu and sigma
  X = sigma * X + mu;
  return X;
}

}
