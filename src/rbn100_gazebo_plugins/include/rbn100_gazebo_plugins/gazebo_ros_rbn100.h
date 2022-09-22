#ifndef GAZEBO_ROS_RBN100_H
#define GAZEBO_ROS_RBN100_H

#include <cmath>
#include <cstring>
#include <string>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Time.hh>
#if GAZEBO_MAJOR_VERSION >= 9
  // #include <ignition/math.hh>
  #include <ignition/math/Vector3.hh>
  #include <ignition/math/Quaternion.hh>
#else
  #include <gazebo/math/gzmath.hh>
#endif
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>  //for cliff and bumper control
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Quaternion.h>
#include <rbn100_msgs/MotorPower.h>
#include <rbn100_msgs/CliffEvent.h>
#include <rbn100_msgs/BumperEvent.h>
#include <rbn100_msgs/Encoder.h>

namespace gazebo
{

enum {LEFT= 0, RIGHT=1};
#define ENCODER_N 5600
#define PI 3.1415926
//  放大以保留更多数据提高精度
#define MAX_ENCODER 0xFFFFFF * 1e5

class GazeboRosRbn100 : public ModelPlugin
{
public:
  /// Constructor
  GazeboRosRbn100();
  /// Destructor
  ~GazeboRosRbn100();
  /// Called when plugin is loaded
  void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);
  /// Called by the world update start event
  // Called at every iteration end
  void OnUpdate();

private:
  /*
   * Methods
   */
  /// Callback for incoming velocity commands
  void cmdVelCB(const geometry_msgs::TwistConstPtr &msg);
  /// Callback for incoming velocity commands
  void motorPowerCB(const rbn100_msgs::MotorPowerPtr &msg);
  /// Callback for resetting the odometry data
  void resetOdomCB(const std_msgs::EmptyConstPtr &msg);
  /* bumper and cliff motor control */
  // Callback for incoming forbidden bumper press to stop motor power
  void bumperAutoStopMotorCB(const std_msgs::BoolPtr &msg);
  // Callback for incoming forbidden cliff event press to stop motor power
  void cliffAutoStopMotorCB(const std_msgs::BoolPtr &msg);

  // void set_motor_enable(std::uint8_t _state);
  ///motor power state publisher
  void motorStatePub(std::uint8_t _state);
  //  void OnContact(const std::string &name, const physics::Contact &contact); necessary?
  /// Spin method for the spinner thread
  // void spin();

  /* 
    internal functions for load,
    read params from urdf
   */
  // init odom pose
  bool prepareOdom();
  // init motor flag
  // void prepareMotorPower();
  // get publish TF info
  void preparePublishTf();
  // get wheel info
  bool prepareWheelAndTorque();
  // obtain the object of joints and set jointstate msg
  bool prepareJointState();
  // obtain the object of imu sensor and active them
  bool prepareIMU();
  // obtain the object of cliff sensor and active them
  bool prepareCliffSensor();
  // obtain the object of bumper sensor and active it
  bool prepareBumper();
  // velocity timeout
  bool prepareVelocityCommand();
  //advertise or subscribe
  void setupRosApi(std::string& model_name);
  
  /* 
   internal functions for update
   */
  void updateJointState();
  void updateOdometry(common::Time& step_time);
  void updateIMU();
  void propagateVelocityCommands();
  void updateCliffSensor();
  void updateBumper();
  // void pubSensorState();

  /*
   *  Parameters
   */
  /// ROS node handles (relative & private)
  // ros::NodeHandle nh_, nh_priv_;
  /// node name
  std::string node_name_;

  /// TF Prefix
  std::string tf_prefix_;
  /// extra thread for triggering ROS callbacks
  //  boost::shared_ptr<boost::thread> ros_spinner_thread_; necessary?

  /// flag for shutting down the spinner thread
  bool shutdown_requested_;
  /// pointer to the model
  physics::ModelPtr model_;
  /// pointer to simulated world
  physics::WorldPtr world_;
  /// Pointers to Gazebo's joints
  physics::JointPtr joints_[2];
  // pointer to the gazebo ros node
  // simplify parameter and rosnode handle
  GazeboRosPtr gazebo_ros_;
  sdf::ElementPtr sdf_;
  // pointer to the update event connection 
  // (triggers the OnUpdate callback when event update event is received)
  event::ConnectionPtr update_connection_;
  /// rate control
  // double update_rate_;
  /// Simulation time on previous update
  common::Time prev_update_time_;
  /// ROS subscriber for motor power commands
  ros::Subscriber motor_power_sub_;
  /// Flag indicating if the motors are turned on or not
  bool motors_enabled_;
  /// Left wheel's joint name
  std::string left_wheel_joint_name_;
  /// Right wheel's joint name
  std::string right_wheel_joint_name_;
  /// ROS publisher for joint state messages
  ros::Publisher joint_state_pub_;
  /// ROS message for joint sates
  sensor_msgs::JointState joint_state_;
  /// ROS subscriber for velocity commands
  ros::Subscriber cmd_vel_sub_;
  /// Simulation time of the last velocity command (used for time out)
  common::Time last_cmd_vel_time_;
  /// Time out for velocity commands in seconds
  double cmd_vel_timeout_;
  /// Speeds of the wheels
  double wheel_speed_cmd_[2];
  /// Max. torque applied to the wheels
  double torque_;
  /// Separation between the wheels
  double wheel_sep_;
  /// Diameter of the wheels
  double wheel_diam_;
  /// Vector for pose
  double odom_pose_[3];
  /// Vector for velocity
  double odom_vel_[3];
  /// Pointer to pose covariance matrix
  double *pose_cov_[36];
  /// Pointer to twist covariance matrix
  double *twist_cov_[36];
  /// ROS publisher for odometry messages
  ros::Publisher odom_pub_;
  /// ROS message for odometry data
  // nav_msgs::Odometry odom_;
  /// Flag for (not) publish tf transform for odom -> robot
  bool publish_tf_;
  /// TF transform publisher for the odom frame
  tf::TransformBroadcaster tf_broadcaster_;
  /// TF transform for the odom frame
  geometry_msgs::TransformStamped odom_tf_;
  /// Pointer to FL cliff sensor
  sensors::RaySensorPtr cliff_sensor_FL_;
  /// Pointer to FR cliff sensor
  sensors::RaySensorPtr cliff_sensor_FR_;
  /// Pointer to BL cliff sensor
  sensors::RaySensorPtr cliff_sensor_BL_;
  /// Pointer to BR right sensor
  sensors::RaySensorPtr cliff_sensor_BR_;
  /// ROS publisher for cliff detection events
  ros::Publisher cliff_event_pub_;
  /// rbn100 ROS message for cliff event
  rbn100_msgs::CliffEvent cliff_event_;
  // trigger event control
  /// Cliff flag for the FL sensor
  bool cliff_detected_FL_;
  /// Cliff flag for the FR sensor
  bool cliff_detected_FR_;
  /// Cliff flag for the BL sensor
  bool cliff_detected_BL_;
  /// Cliff flag for the BR sensor
  bool cliff_detected_BR_;
  /// measured distance in meter for detecting a cliff
  float cliff_detection_threshold_;
  
  // flag of enable cliff and bumper stop motor or not
  bool cliff_auto_stop_motor_flag;
  bool bumper_auto_stop_motor_flag;
  
  /// Maximum distance to floor
  int floot_dist_;
  /// Pointer to bumper sensor
  sensors::ContactSensorPtr bumper_;
  /// ROS publisher for bumper events
  ros::Publisher bumper_event_pub_;
  /// rbn100 ROS message for bumper event
  rbn100_msgs::BumperEvent bumper_event_;
  /// Flag for bumper's last state
  bool bumper_was_pressed_;
  /// Flag for bumper's current state
  bool bumper_is_pressed_;
  /// Pointer to IMU sensor model
  sensors::ImuSensorPtr imu_;
  /// Storage for the angular velocity reported by the IMU
  #if GAZEBO_MAJOR_VERSION >= 9
    ignition::math::Vector3d vel_angular_;
  #else
    math::Vector3 vel_angular_;
  #endif

  /// ROS publisher for IMU data
  ros::Publisher imu_pub_;
  /// ROS message for publishing IMU data
  sensor_msgs::Imu imu_msg_;
  /// ROS subscriber for reseting the odometry data
  ros::Subscriber odom_reset_sub_;

  /// ROS publisher for rbn100 sensor state
  ros::Publisher sensor_state_pub_;

  // ROS subscriber of enable bumper and cliff stop motor
  ros::Subscriber bumper_auto_stop_motor_sub_;
  ros::Subscriber cliff_auto_stop_motor_sub_;
  // ROS publisher for motor power state events
  ros::Publisher motor_power_state_pub_;

  int console_log_rate;
  
  std::string odom_name_;
  std::string bumper_name_;
  std::string imu_name_;

  double update_rate_;
  double imu_rate_;

  common::Time rate_step_;
  common::Time imu_step_;

  rbn100_msgs::Encoder odom_;
  double m_per_encoder_;
};

} // namespace gazebo

#endif /* GAZEBO_ROS_RBN100_H */
