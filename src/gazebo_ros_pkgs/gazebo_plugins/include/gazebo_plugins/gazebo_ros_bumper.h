/*
 * Copyright 2012 Open Source Robotics Foundation
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

#ifndef GAZEBO_ROS_BUMPER_HH
#define GAZEBO_ROS_BUMPER_HH

#ifndef _WIN32
#include <sys/time.h>
#endif

#include <string>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <gazebo_msgs/ContactsState.h>
#include <gazebo_msgs/ContactState.h>

#include <sdf/Element.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/sensors/ContactSensor.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/CommonTypes.hh>

namespace gazebo
{
  /// \brief A Bumper controller
  class GazeboRosBumper : public SensorPlugin
  {
    /// Constructor
    public: GazeboRosBumper();

    /// Destructor
    public: ~GazeboRosBumper();

    /// \brief Load the plugin
    /// \param take in SDF root element
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    /// Update the controller
    private: void OnContact();

    /// \brief pointer to ros node
    private: ros::NodeHandle* rosnode_;
    private: ros::Publisher contact_pub_;

    private: sensors::ContactSensorPtr parentSensor;

    /// \brief set topic name of broadcast
    private: std::string bumper_topic_name_;

    private: std::string frame_name_;

    /// \brief broadcast some string for now.
    private: gazebo_msgs::ContactsState contact_state_msg_;

    /// \brief for setting ROS name space
    private: std::string robot_namespace_;

    // 自定义回调队列，当有消息发布时，ros会把消息放入队列中，等待处理
    private: ros::CallbackQueue contact_queue_;
    private: void ContactQueueThread();
    private: boost::thread callback_queue_thread_;

    // Pointer to the update event connection
    private: event::ConnectionPtr update_connection_;
  };
}

#endif

