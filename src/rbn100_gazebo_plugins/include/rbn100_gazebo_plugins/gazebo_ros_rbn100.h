#ifndef GAZEBO_ROS_RBN100_H
#define GAZEBO_ROS_RBN100_H

#include <string>

#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
// #include <gazebo/physics/physics.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
// #include <gazebo/sensors/sensors.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/sensors/ContactSensor.hh>
#include <gazebo/sensors/WideAngleCameraSensor.hh>
#include <gazebo/rendering/Camera.hh>

#include <gazebo_plugins/gazebo_ros_utils.h>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/fill_image.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_transport/image_transport.h>
// #include <tf/transform_broadcaster.h>
// #include <tf/LinearMath/Quaternion.h>

#include <rbn100_msgs/MotorPower.h>
#include <rbn100_msgs/CliffEvent.h>
#include <rbn100_msgs/BumperEvent.h>
#include <rbn100_msgs/Encoder.h>
#include <rbn100_msgs/Ultra.h>
#include <rbn100_msgs/StereoImage.h>
#include <rbn100_msgs/WheelSpeed.h>

namespace gazebo
{
enum {LEFT, RIGHT};
#define WHEEL_JOINT_NUM 2
#define ENCODER_N 5600
#define PI 3.1415926
#define SONAR_NUM 4
// #define ENABLE_PUBLIC_CAMERAS

class GazeboRosRbn100 : public ModelPlugin
{
public:
  GazeboRosRbn100();
  ~GazeboRosRbn100();

  /// Called when plugin is loaded
  void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);
  
  // Called at every iteration end
  void OnUpdate();
  void OnNewCameraFrameLeft();
  void OnNewCameraFrameRight();

private:
  void cmdVelCB(const geometry_msgs::TwistConstPtr &msg);
  void motorPowerCB(const rbn100_msgs::MotorPowerPtr &msg);
  void resetOdomCB(const std_msgs::EmptyConstPtr &msg);
  void bumperAutoStopMotorCB(const std_msgs::BoolPtr &msg);
  void cliffAutoStopMotorCB(const std_msgs::BoolPtr &msg);

  bool prepareOdom();
  void preparePublishTf();
  bool prepareWheelAndTorque();
  bool prepareJointState();
  bool prepareIMU();
  bool prepareCliffSensor();
  bool prepareBumper();
  bool prepareUltra();
  bool prepareVelocityCommand();
  bool prepareStereoCamera();
  //set publisher and subscriber
  void setupRosApi(const GazeboRosPtr& gazebo_ros);

  void updateJointState();
  void updateOdometry(common::Time& step_time);
  void updateIMU();
  void propagateVelocityCommands();
  void updateCliffSensor();
  void updateBumper();
  void updateUltra();
  void updateStereoCamera();
  void updateWheelSpeed();

  double GaussianKernel(double mu, double sigma);

/* 
  information about model and world, 
  can easily be used by prepare function in Load.
*/
  physics::WorldPtr world_;
  physics::ModelPtr model_;
  sdf::ElementPtr sdf_; 
  std::string node_name_;
  
  bool publish_tf_;

  physics::JointPtr wheel_joints_[2];
  
  double torque_;
  double wheel_sep_;
  double wheel_diam_;

  // accmulate in all sim
  rbn100_msgs::Encoder odom_msg_;
  double m_per_encoder_;
  std::string odom_topic_;

  double cmd_vel_timeout_;
  common::Time last_cmd_vel_time_;
  double wheel_speed_cmd_[2];

  sensors::RaySensorPtr cliff_FL_sensor_, cliff_FR_sensor_, cliff_BL_sensor_, cliff_BR_sensor_;
  float cliff_detection_threshold_;
  bool cliff_detected_FL_, cliff_detected_FR_, cliff_detected_BL_, cliff_detected_BR_;
  int floor_dist_;

  sensors::ContactSensorPtr contact_bumper_;
  bool bumper_was_pressed_, bumper_is_pressed_;

  double sonar_period_;
  sensors::RaySensorPtr sonar_FL_sensor_, sonar_front_sensor_, sonar_FR_sensor_, sonar_back_sensor_;
  int samples_;
  double range_max_, range_min_;

  double camera_period_;
  rendering::CameraPtr leftCam_, rightCam_;
  event::ConnectionPtr newLeftImgFrameConn_, newRightImgFrameConn_;
  #ifdef ENABLE_PUBLIC_CAMERAS
    image_transport::ImageTransport *itnode_;
    image_transport::CameraPublisher leftImg_pub_, rightImg_pub_;
    std::string camera_left_topic_, camera_right_topic_,
                camera_left_frame_, camera_right_frame_;
  #endif

  ros::Publisher joint_state_pub_, odom_pub_, ultra_pub_, imu_pub_, 
                 cliff_event_pub_, bumper_event_pub_, stereo_image_pub_, wheel_speed_pub_;
  ros::Subscriber motor_power_sub_, cmd_vel_sub_, bumper_auto_stop_motor_sub_, 
                  cliff_auto_stop_motor_sub_, odom_reset_sub_;
  
  bool motors_enabled_;
  bool cliff_auto_stop_motor_flag, bumper_auto_stop_motor_flag;
  
  common::Time prev_update_time_;

  event::ConnectionPtr update_connection_;

  sensor_msgs::Image leftImg_msg_, rightImg_msg_;
  rbn100_msgs::Ultra ultra_msg_;
  double odom_pose_[3];
  double odom_vel_[3];
  double *pose_cov_[36];
  double *twist_cov_[36];
  // double update_rate_, imu_rate_, sonar_rate_;
  double update_period_, imu_period_;
  double sonar_noise_;
  common::Time rate_step_, imu_step_, sonar_step_, camera_step_;

};

} // namespace gazebo

#endif /* GAZEBO_ROS_RBN100_H */
