#include "rbn100_gazebo_plugins/gazebo_ros_rbn100.h"

namespace gazebo 
{

/*
 * Propagate velocity commands
 * TODO: Check how to simulate disabled motors, e.g. set MaxForce to zero, but then damping is important!
 */
void GazeboRosRbn100::propagateVelocityCommands()
{
  // .double从纳秒值得到妙
  if (((prev_update_time_ - last_cmd_vel_time_).Double() > cmd_vel_timeout_) || !motors_enabled_)
  {
    wheel_speed_cmd_[LEFT] = 0.0;
    wheel_speed_cmd_[RIGHT] = 0.0;
    ROS_INFO_STREAM_ONCE("vel cmd has been set to 0");
  }
  joints_[LEFT]->SetVelocity(0, wheel_speed_cmd_[LEFT] / (wheel_diam_ / 2.0));
  joints_[RIGHT]->SetVelocity(0, wheel_speed_cmd_[RIGHT] / (wheel_diam_ / 2.0));
}

void GazeboRosRbn100::updateJointState()
{
  /*
   * Joint states
   */
  std::string baselink_frame = gazebo_ros_->resolveTF("base_link");
  joint_state_.header.stamp = ros::Time::now();
  joint_state_.header.frame_id = baselink_frame;  //base_link

  #if GAZEBO_MAJOR_VERSION >= 9
    joint_state_.position[LEFT] = joints_[LEFT]->Position(0);
    joint_state_.position[RIGHT] = joints_[RIGHT]->Position(0);
  #else
    joint_state_.position[LEFT] = joints_[LEFT]->GetAngle(0).Radian();
    joint_state_.position[RIGHT] = joints_[RIGHT]->GetAngle(0).Radian();
  #endif

  joint_state_.velocity[LEFT] = joints_[LEFT]->GetVelocity(0);
  joint_state_.velocity[RIGHT] = joints_[RIGHT]->GetVelocity(0);

  joint_state_pub_.publish(joint_state_);
}

/*
 * Odometry (encoders & IMU)
 */
void GazeboRosRbn100::updateOdometry(common::Time& step_time)
{
  std::string odom_frame = gazebo_ros_->resolveTF("odom");
  std::string base_frame = gazebo_ros_->resolveTF("base_footprint");
  odom_.header.stamp = ros::Time::now();
  odom_.header.frame_id = odom_frame;
  // odom_.child_frame_id = base_frame;

  // Distance travelled by main wheels
  double d1, d2;
  double dr, da;
  d1 = d2 = 0;
  dr = da = 0;
  d1 = step_time.Double() * (wheel_diam_ / 2) * joints_[LEFT]->GetVelocity(0);
  d2 = step_time.Double() * (wheel_diam_ / 2) * joints_[RIGHT]->GetVelocity(0);
  /* 
    计算码盘值
   */
  odom_.EncoderL += (uint64_t)(d1 / m_per_encoder_ * 1e5);
  odom_.EncoderR += (uint64_t)(d2 / m_per_encoder_ * 1e5);
  odom_pub_.publish(odom_);

  // // Can see NaN values here, just zero them out if needed
  // if (std::isnan(d1))
  // {
  //   ROS_WARN_STREAM_THROTTLE(0.1, "Gazebo ROS Rbn100 plugin: NaN in d1. Step time: " << step_time.Double()
  //                            << ", WD: " << wheel_diam_ << ", velocity: " << joints_[LEFT]->GetVelocity(0));
  //   d1 = 0;
  // }
  // if (std::isnan(d2))
  // {
  //   ROS_WARN_STREAM_THROTTLE(0.1, "Gazebo ROS Rbn100 plugin: NaN in d2. Step time: " << step_time.Double()
  //                            << ", WD: " << wheel_diam_ << ", velocity: " << joints_[RIGHT]->GetVelocity(0));
  //   d2 = 0;
  // }
  // dr = (d1 + d2) / 2;
  // da = (d2 - d1) / wheel_sep_; // ignored

  // // Just as in the Rbn100 driver, the angular velocity is taken directly from the IMU
  // vel_angular_ = imu_->AngularVelocity();

  // // Compute odometric pose
  // odom_pose_[0] += dr * cos( odom_pose_[2] );
  // odom_pose_[1] += dr * sin( odom_pose_[2] );

  // #if GAZEBO_MAJOR_VERSION >= 9
  //   odom_pose_[2] += vel_angular_.Z() * step_time.Double();
  // #else
  //   odom_pose_[2] += vel_angular_.z * step_time.Double();
  // #endif

  // // Compute odometric instantaneous velocity
  // odom_vel_[0] = dr / step_time.Double();
  // odom_vel_[1] = 0.0;

  // #if GAZEBO_MAJOR_VERSION >= 9
  //   odom_vel_[2] = vel_angular_.Z();

  // #else
  //   odom_vel_[2] = vel_angular_.z;
  // #endif

  // odom_.pose.pose.position.x = odom_pose_[0];
  // odom_.pose.pose.position.y = odom_pose_[1];
  // odom_.pose.pose.position.z = 0;

  // tf::Quaternion qt;
  // qt.setEuler(0,0,odom_pose_[2]);
  // odom_.pose.pose.orientation.x = qt.getX();
  // odom_.pose.pose.orientation.y = qt.getY();
  // odom_.pose.pose.orientation.z = qt.getZ();
  // odom_.pose.pose.orientation.w = qt.getW();

  // odom_.pose.covariance[0]  = 0.1;
  // odom_.pose.covariance[7]  = 0.1;
  // odom_.pose.covariance[14] = 1e6;
  // odom_.pose.covariance[21] = 1e6;
  // odom_.pose.covariance[28] = 1e6;
  // odom_.pose.covariance[35] = 0.05;

  // odom_.twist.twist.linear.x = odom_vel_[0];
  // odom_.twist.twist.linear.y = 0;
  // odom_.twist.twist.linear.z = 0;
  // odom_.twist.twist.angular.x = 0;
  // odom_.twist.twist.angular.y = 0;
  // odom_.twist.twist.angular.z = odom_vel_[2];
  // // odom_pub_.publish(odom_);

  // if (publish_tf_)
  // {
  //   odom_tf_.header = odom_.header;
  //   odom_tf_.child_frame_id = odom_.child_frame_id;
  //   odom_tf_.transform.translation.x = odom_.pose.pose.position.x;
  //   odom_tf_.transform.translation.y = odom_.pose.pose.position.y;
  //   odom_tf_.transform.translation.z = odom_.pose.pose.position.z;
  //   odom_tf_.transform.rotation = odom_.pose.pose.orientation;
  //   tf_broadcaster_.sendTransform(odom_tf_);
  // }
}

/*
 * Publish IMU data
 */
void GazeboRosRbn100::updateIMU()
{
  imu_msg_.header.frame_id = imu_name_;
  imu_msg_.header.stamp = ros::Time::now();

  #if GAZEBO_MAJOR_VERSION >= 9
    ignition::math::Quaterniond quat = imu_->Orientation();
    imu_msg_.orientation.x = quat.X();
    imu_msg_.orientation.y = quat.Y();
    imu_msg_.orientation.z = quat.Z();
    imu_msg_.orientation.w = quat.W();
  #else
    math::Quaternion quat = imu_->Orientation();
    imu_msg_.orientation.x = quat.x;
    imu_msg_.orientation.y = quat.y;
    imu_msg_.orientation.z = quat.z;
    imu_msg_.orientation.w = quat.w;
  #endif

  imu_msg_.orientation_covariance[0] = 1e6;
  imu_msg_.orientation_covariance[4] = 1e6;
  imu_msg_.orientation_covariance[8] = 0.05;

  #if GAZEBO_MAJOR_VERSION >= 9
    imu_msg_.angular_velocity.x = vel_angular_.X();
    imu_msg_.angular_velocity.y = vel_angular_.Y();
    imu_msg_.angular_velocity.z = vel_angular_.Z();
  #else
    imu_msg_.angular_velocity.x = vel_angular_.x;
    imu_msg_.angular_velocity.y = vel_angular_.y;
    imu_msg_.angular_velocity.z = vel_angular_.z;
  #endif


  imu_msg_.angular_velocity_covariance[0] = 1e6;
  imu_msg_.angular_velocity_covariance[4] = 1e6;
  imu_msg_.angular_velocity_covariance[8] = 0.05;

  #if GAZEBO_MAJOR_VERSION >= 9
    ignition::math::Vector3d lin_acc = imu_->LinearAcceleration();
    imu_msg_.linear_acceleration.x = lin_acc.X();
    imu_msg_.linear_acceleration.y = lin_acc.Y();
    imu_msg_.linear_acceleration.z = lin_acc.Z();
  #else
    math::Vector3 lin_acc = imu_->LinearAcceleration();
    imu_msg_.linear_acceleration.x = lin_acc.x;
    imu_msg_.linear_acceleration.y = lin_acc.y;
    imu_msg_.linear_acceleration.z = lin_acc.z;
  #endif

  imu_pub_.publish(imu_msg_);
}

/*
 * Cliff sensors
 * Check each sensor separately
 */
void GazeboRosRbn100::updateCliffSensor()
{
  // FL cliff sensor
  if ((cliff_detected_FL_ == false) &&
      (cliff_sensor_FL_->Range(0) >= cliff_detection_threshold_))
  {
    ROS_INFO_STREAM("cliff_FL detected");
    if(cliff_auto_stop_motor_flag){
      // set_motor_enable(false);
      ROS_INFO_STREAM("enable cliff stop motor");
      motors_enabled_ = false;
    }
    cliff_detected_FL_ = true;
    cliff_event_.which = rbn100_msgs::CliffEvent::FL;
    cliff_event_.state = rbn100_msgs::CliffEvent::CLIFF;
    // convert distance back to an AD reading
    cliff_event_.bottom = (int)(76123.0f * atan2(0.995f, cliff_sensor_FL_->Range(0)));
    cliff_event_pub_.publish(cliff_event_);
  }
  else if ((cliff_detected_FL_ == true) && 
           (cliff_sensor_FL_->Range(0) < cliff_detection_threshold_))
  {
    ROS_INFO_STREAM("cliff_FL normal");
    cliff_detected_FL_ = false;
    cliff_event_.which = rbn100_msgs::CliffEvent::FL;
    cliff_event_.state = rbn100_msgs::CliffEvent::FLOOR;
    cliff_event_.bottom = (int)(76123.0f * atan2(0.995f, cliff_sensor_FL_->Range(0)));
    cliff_event_pub_.publish(cliff_event_);
  }
  // FR cliff sensor
  if ((cliff_detected_FR_ == false) &&
      (cliff_sensor_FR_->Range(0) >= cliff_detection_threshold_))
  {
    ROS_INFO_STREAM("cliff_FR detected");
    if(cliff_auto_stop_motor_flag){
      // set_motor_enable(false);
      ROS_INFO_STREAM("enable cliff stop motor");
      motors_enabled_ = false;
    }
    cliff_detected_FR_ = true;
    cliff_event_.which = rbn100_msgs::CliffEvent::FR;
    cliff_event_.state = rbn100_msgs::CliffEvent::CLIFF;
    cliff_event_.bottom = (int)(76123.0f * atan2(0.995f, cliff_sensor_FR_->Range(0)));
    cliff_event_pub_.publish(cliff_event_);
  }
  else if ((cliff_detected_FR_ == true) &&
          (cliff_sensor_FR_->Range(0) < cliff_detection_threshold_))
  {
    ROS_INFO_STREAM("cliff_FR normal");
    cliff_detected_FR_ = false;
    cliff_event_.which = rbn100_msgs::CliffEvent::FR;
    cliff_event_.state = rbn100_msgs::CliffEvent::FLOOR;
    cliff_event_.bottom = (int)(76123.0f * atan2(0.995f, cliff_sensor_FR_->Range(0)));
    cliff_event_pub_.publish(cliff_event_);
  }
  // BL cliff sensor
  if ((cliff_detected_BL_ == false) &&
      (cliff_sensor_BL_->Range(0) >= cliff_detection_threshold_))
  {
    ROS_INFO_STREAM("cliff_BL detected");
    if(cliff_auto_stop_motor_flag){
      // set_motor_enable(false);
      ROS_INFO_STREAM("enable cliff stop motor");
      motors_enabled_ = false;
    }
    cliff_detected_BL_ = true;
    cliff_event_.which = rbn100_msgs::CliffEvent::BL;
    cliff_event_.state = rbn100_msgs::CliffEvent::CLIFF;
    cliff_event_.bottom = (int)(76123.0f * atan2(0.995f, cliff_sensor_BL_->Range(0)));
    cliff_event_pub_.publish(cliff_event_);
  }
  else if ((cliff_detected_BL_ == true) &&
          (cliff_sensor_BL_->Range(0) < cliff_detection_threshold_))
  {
    ROS_INFO_STREAM("cliff_BL normal");
    cliff_detected_BL_ = false;
    cliff_event_.which = rbn100_msgs::CliffEvent::BL;
    cliff_event_.state = rbn100_msgs::CliffEvent::FLOOR;
    cliff_event_.bottom = (int)(76123.0f * atan2(0.995f, cliff_sensor_BL_->Range(0)));
    cliff_event_pub_.publish(cliff_event_);
  }
  // BR cliff sensor
  if ((cliff_detected_BR_ == false) &&
      (cliff_sensor_BR_->Range(0) >= cliff_detection_threshold_))
  {
    ROS_INFO_STREAM("cliff_BR detected");
    if(cliff_auto_stop_motor_flag){
      // set_motor_enable(false);
      ROS_INFO_STREAM("enable cliff stop motor");
      motors_enabled_ = false;
    }
    cliff_detected_BR_ = true;
    cliff_event_.which = rbn100_msgs::CliffEvent::BR;
    cliff_event_.state = rbn100_msgs::CliffEvent::CLIFF;
    cliff_event_.bottom = (int)(76123.0f * atan2(0.995f, cliff_sensor_BR_->Range(0)));
    cliff_event_pub_.publish(cliff_event_);
  }
  else if ((cliff_detected_BR_ == true) &&
          (cliff_sensor_BR_->Range(0) < cliff_detection_threshold_))
  {
    ROS_INFO_STREAM("cliff_BR normal");
    cliff_detected_BR_ = false;
    cliff_event_.which = rbn100_msgs::CliffEvent::BR;
    cliff_event_.state = rbn100_msgs::CliffEvent::FLOOR;
    cliff_event_.bottom = (int)(76123.0f * atan2(0.995f, cliff_sensor_BR_->Range(0)));
    cliff_event_pub_.publish(cliff_event_);
  }
}

void GazeboRosRbn100::updateUltra(){
  // set time to now
  ultra_msg_.head.stamp = ros::Time::now();
  // every iteration, init distance to max
  for (int i = 0; i < SONAR_NUM; i++)
  {
    ultra_msg_.distance[i] = sonar_sensor_FL_->RangeMax();
  }
  // read range of all samples
  for (int i = 0; i < samples_; i++){
    // read range of a ray
    double ray_FL = sonar_sensor_FL_->LaserShape()->GetRange(i);
    double ray_front = sonar_sensor_front_->LaserShape()->GetRange(i);
    double ray_FR = sonar_sensor_FR_->LaserShape()->GetRange(i);
    double ray_back = sonar_sensor_back_->LaserShape()->GetRange(i);
    // get the minest value
    if(ray_FL < ultra_msg_.distance[0])
      ultra_msg_.distance[0] = ray_FL;
    if(ray_front < ultra_msg_.distance[1])
      ultra_msg_.distance[1] = ray_front;
    if(ray_FR < ultra_msg_.distance[2])
      ultra_msg_.distance[2] = ray_FR;
    if(ray_back < ultra_msg_.distance[3])
      ultra_msg_.distance[3] = ray_back;
  }
  // add noise and limit to [min, max] range
  for (int i = 0; i < SONAR_NUM; i++)
  {
    int noise_range = ultra_msg_.distance[i] + GaussianKernel(0, sonar_noise_);
    if(noise_range > sonar_sensor_FL_->RangeMax())
      ultra_msg_.distance[i] = sonar_sensor_FL_->RangeMax();
    else if(noise_range < sonar_sensor_FL_->RangeMin())
      ultra_msg_.distance[i] = sonar_sensor_FL_->RangeMin();
    else
      ultra_msg_.distance[i] = noise_range;
  }
  ultra_pub_.publish(ultra_msg_);
}

/*
 * Bumpers
 */
void GazeboRosRbn100::updateBumper()
{
  // reset flags
  bumper_is_pressed_ = false;

  //parse contact
  msgs::Contacts contacts;
  contacts = bumper_->Contacts();
  // ROS_INFO_STREAM("contact size: " << contacts.contact_size());
  if(contacts.contact_size() >= 1){
    bumper_is_pressed_ = true;
  }

  // check for bumper state change
  if (bumper_is_pressed_ && !bumper_was_pressed_)
  {
    ROS_INFO_STREAM("bumper triggered");
    if(bumper_auto_stop_motor_flag){
      ROS_INFO_STREAM("enable bumper stop motor");
      // set_motor_enable(false);
      motors_enabled_ = false;
    }
    bumper_was_pressed_ = true;
    bumper_event_.state = rbn100_msgs::BumperEvent::PRESSED;
    bumper_event_.bumper = rbn100_msgs::BumperEvent::body;
    bumper_event_.stamp = ros::Time::now();
    bumper_event_pub_.publish(bumper_event_);
  }
  else if (!bumper_is_pressed_ && bumper_was_pressed_)
  {
    ROS_INFO_STREAM("bumper normal");
    bumper_was_pressed_ = false;
    bumper_event_.state = rbn100_msgs::BumperEvent::RELEASED;
    bumper_event_.bumper = rbn100_msgs::BumperEvent::body;
    bumper_event_.stamp = ros::Time::now();
    bumper_event_pub_.publish(bumper_event_);
  }
}

/* 
  motor state
 */
/* void GazeboRosRbn100::updateMotorState()
{
  rbn100_msgs::MotorPower msg;
  msg.state = motors_enabled_;
  motor_power_state_pub_.publish(msg);
} */

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
