/*
 * Copyright 2013 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

/*
 * Desc: Ros Laser controller.
 * Author: Nathan Koenig
 * Date: 01 Feb 2007
 */

#include <gazebo_plugins/gazebo_ros_laser.h>

#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/transport/Node.hh>
#ifdef ENABLE_PROFILER
#include <ignition/common/Profiler.hh>
#endif

#include <ros/advertise_options.h>
#include <tf/transform_listener.h>

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosLaser)

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosLaser::GazeboRosLaser(){}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosLaser::~GazeboRosLaser()
{
  this->rosnode_->shutdown();
  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosLaser::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  // load plugin, _sdf param is not used
  RayPlugin::Load(_parent, this->sdf);
  // sensor obj can get world name, can't get world obj.
  this->world_ = physics::get_world(_parent->WorldName());
  // save pointers
  this->sdf = _sdf;

  // GAZEBO_SENSORS_USING_DYNAMIC_POINTER_CAST;
  this->parent_ray_sensor_ = std::dynamic_pointer_cast<sensors::RaySensor>(_parent);

  if (!this->parent_ray_sensor_)
    gzthrow("GazeboRosLaser controller requires a Ray Sensor as its parent");

  this->robot_namespace_ =  GetRobotNamespace(_parent, _sdf, "Laser");

  if (!this->sdf->HasElement("frameName"))
  {
    ROS_INFO_NAMED("laser", "Laser plugin missing <frameName>, defaults to /world");
    this->frame_name_ = "/world";
  }
  else
    this->frame_name_ = this->sdf->Get<std::string>("frameName");


  if (!this->sdf->HasElement("topicName"))
  {
    ROS_INFO_NAMED("laser", "Laser plugin missing <topicName>, defaults to /world");
    this->topic_name_ = "/world";
  }
  else
    this->topic_name_ = this->sdf->Get<std::string>("topicName");

  this->laser_connect_count_ = 0;

    // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("laser", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  ROS_INFO_NAMED("laser", "Starting Laser Plugin (ns = %s)", this->robot_namespace_.c_str() );
  // ros callback queue for processing subscription
  // 防止ros处理数据等卡住整个线程
  this->deferred_load_thread_ = boost::thread(boost::bind(&GazeboRosLaser::LoadThread, this));
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosLaser::LoadThread()
{
  this->gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
  this->gazebo_node_->Init();

  // 分离消息的产生和发布，一个线程生产消息放入发布队列，一个线程从队列中发布消息
  // 支持多消息
  this->pmq.startServiceThread();

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  this->tf_prefix_ = tf::getPrefixParam(*this->rosnode_);
  ROS_INFO_NAMED("laser", "Laser Plugin (ns = %s)  <tf_prefix_>, set to \"%s\"",
             this->robot_namespace_.c_str(), this->tf_prefix_.c_str());

  // resolve tf prefix
  this->frame_name_ = tf::resolve(this->tf_prefix_, this->frame_name_);

  if (this->topic_name_ != "")
  {
    ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<sensor_msgs::LaserScan>(
      this->topic_name_, 
      1,
      boost::bind(&GazeboRosLaser::LaserConnect, this),
      boost::bind(&GazeboRosLaser::LaserDisconnect, this),
      ros::VoidPtr(), 
      NULL);
    this->pub_ = this->rosnode_->advertise(ao);
    this->pub_queue_ = this->pmq.addPub<sensor_msgs::LaserScan>();
  }
  // sensor will be activated if someone connects to parentsensor's topic.
  this->parent_ray_sensor_->SetActive(false);
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosLaser::LaserConnect()
{
  this->laser_connect_count_++;
  if (this->laser_connect_count_ == 1)
    this->laser_scan_sub_ = this->gazebo_node_->Subscribe(this->parent_ray_sensor_->Topic(),
                                                          &GazeboRosLaser::OnScan, 
                                                          this);
}

////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosLaser::LaserDisconnect()
{
  this->laser_connect_count_--;
  if (this->laser_connect_count_ == 0)
    this->laser_scan_sub_.reset();
}

////////////////////////////////////////////////////////////////////////////////
// Convert new Gazebo message to ROS message and publish it
void GazeboRosLaser::OnScan(ConstLaserScanStampedPtr &_msg)
{
#ifdef ENABLE_PROFILER
  IGN_PROFILE("GazeboRosLaser::OnScan");
  IGN_PROFILE_BEGIN("fill ROS message");
#endif
  // We got a new message from the Gazebo sensor.  Stuff a
  // corresponding ROS message and publish it.
  sensor_msgs::LaserScan laser_msg;
  laser_msg.header.stamp = ros::Time::now();
  laser_msg.header.frame_id = this->frame_name_;
  laser_msg.angle_min = _msg->scan().angle_min();
  laser_msg.angle_max = _msg->scan().angle_max();
  laser_msg.angle_increment = _msg->scan().angle_step();
  laser_msg.time_increment =  1.0 / 15 / 1536;  // 0 to instantaneous simulator scan
  laser_msg.scan_time = 1.0 / 15;
  laser_msg.range_min = _msg->scan().range_min();
  laser_msg.range_max = _msg->scan().range_max();
  laser_msg.ranges.resize(_msg->scan().ranges_size());
  std::copy(_msg->scan().ranges().begin(),
            _msg->scan().ranges().end(),
            laser_msg.ranges.begin());
  laser_msg.intensities.resize(_msg->scan().intensities_size());
  std::copy(_msg->scan().intensities().begin(),
            _msg->scan().intensities().end(),
            laser_msg.intensities.begin());
  this->pub_queue_->push(laser_msg, this->pub_);
#ifdef ENABLE_PROFILER
  IGN_PROFILE_END();
#endif
}
}