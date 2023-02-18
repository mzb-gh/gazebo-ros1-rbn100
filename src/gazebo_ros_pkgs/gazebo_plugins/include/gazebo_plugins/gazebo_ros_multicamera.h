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

#ifndef GAZEBO_ROS_MULTICAMERA_HH
#define GAZEBO_ROS_MULTICAMERA_HH

#include <string>
#include <vector>

// library for processing camera data for gazebo / ros conversions
#include <gazebo_plugins/gazebo_ros_camera_utils.h>
#include <gazebo_plugins/MultiCameraPlugin.h>

#include "rbn100_msgs/StereoImage.h"

namespace gazebo
{
  class GazeboRosMultiCamera : public MultiCameraPlugin
  {
    /// \brief Constructor
    /// \param parent The parent entity, must be a Model or a Sensor
    public: GazeboRosMultiCamera();

    /// \brief Destructor
    public: ~GazeboRosMultiCamera();

    /// \brief Load the plugin
    /// \param take in SDF root element
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    std::vector<GazeboRosCameraUtils*> utils;

    protected: void OnNewFrame(const unsigned char *_image, 
                               GazeboRosCameraUtils* util);
    /// \brief Update the controller
    /// FIXME: switch to function vectors
    protected: virtual void OnNewFrameLeft(const unsigned char *_image,
                                           unsigned int _width, 
                                           unsigned int _height,
                                           unsigned int _depth, 
                                           const std::string &_format);
    protected: virtual void OnNewFrameRight(const unsigned char *_image,
                                            unsigned int _width, 
                                            unsigned int _height,
                                            unsigned int _depth, 
                                            const std::string &_format);

    protected: virtual void UpdateChild();  // custom
    /// Bookkeeping flags that will be passed into the underlying
    /// GazeboRosCameraUtils objects to let them share state about the parent
    /// sensor.
    // private: boost::shared_ptr<int> image_connect_count_;
    // private: boost::shared_ptr<boost::mutex> image_connect_count_lock_;
    // private: boost::shared_ptr<bool> was_active_;

    //custom
    // 自定义消息
    private: rbn100_msgs::StereoImage stereo_image_msg_;
    // 自定义句柄
    private: ros::NodeHandle* stereo_node_;
    // 图像句柄
    private: image_transport::ImageTransport* stereo_itnode_;
    // ros发布者
    private: ros::Publisher stereo_image_pub_;
    // 图像订阅者
    private:  image_transport::Subscriber stereo_left_image_sub_;
    private: image_transport::Subscriber stereo_right_image_sub_;
    // 自定义话题发布设置：话题被订阅时调用的函数
    protected: void StereoConnect();
    // 自定义话题发布设置：话题不被订阅时调用的函数
    protected: void StereoDisconnect();
    // 在StereoConnect里面订阅双目话题的回调函数
    protected: void leftImageCB(const sensor_msgs::ImageConstPtr&);
    protected: void rightImageCB(const sensor_msgs::ImageConstPtr&);
    // publish rate control
    private: int rate;
  };
}
#endif