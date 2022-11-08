/*
* RosSimSensorNode.h
*
* Created on: Aug 26, 2021
* Author: ubuntu
*/

#include <mind_os/mind_os.h>
#include <ros/ros.h>

// ros stuff
#include "rbn100_msgs/Ultra.h"
#include "rbn100_msgs/CliffEvent.h"
#include "rbn100_msgs/BumperEvent.h"
#include <sensor_msgs/LaserScan.h>
#include "rbn100_msgs/StereoImage.h"
//ros package for ros Image msg(byte stream) to opencv(cv::Mat)
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
// 使用opencv编译时需使用pkg-config指定头文件和库的位置
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/PointCloud2.h>
//ros package for ros PointCloud2 msg(byte stream) to pcl(pcl::pointcloud)
#include <pcl_conversions/pcl_conversions.h>
#include "rbn100_msgs/Encoder.h"
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>

//mos stuff
#include "msg/sensor/UltraData.h"
#include "msg/sensor/CliffEvent.h"
#include "msg/sensor/BumperEvent.h"
#include "msg/sensor/LaserScan.h"
#include "msg/sensor/ImgMsg.h"
#include "msg/sensor/TofDepthData.h"
#include "msg/sensor/EncoderMsg.h"
#include "msg/sensor/ImuMsg.h"
#include "msg/nav/SpeedControl.h"

#include <glog/logging.h>

// static const std::string OPENCV_WINDOW1 = "Image window"; //定义输入图像显示窗口名称
// static const std::string OPENCV_WINDOW2 = "Gray window"; //定义输出图像显示窗口名称

class RosSimSensorNode : public mind_os::NodePlugin
{
    // ros node handle
    ros::NodeHandle ros_nh_;
    
    // ros sub
    ros::Subscriber ultraSub_;
    ros::Subscriber cliffEventSub_;
    ros::Subscriber bumperEventSub_;
    ros::Subscriber laserSub_;
    ros::Subscriber stereoSub_;
    ros::Subscriber depthImageSub_;
    ros::Subscriber pointCloud2Sub_;
    ros::Subscriber encoderSub_;
    ros::Subscriber imuSub_;
    // mos sub
    mind_os::Subscriber cmd_vel_sub_;
    
    //ros pub
    ros::Publisher cmd_vel_pub_;
    // mos pub
    mind_os::Publisher ultraPub_;
    mind_os::Publisher cliffEventPub_;
    mind_os::Publisher bumperEventPub_;
    mind_os::Publisher laserPub_;
    mind_os::Publisher stereoTXPub_;
    mind_os::Publisher stereoIRPub_;
    mind_os::Publisher tofGrayImgPub_;
    mind_os::Publisher pclPub_;
    mind_os::Publisher encoderPub_;
    mind_os::Publisher imuPub_;

public:
    RosSimSensorNode(){
        // 创建图像显示窗口
        // cv::namedWindow(OPENCV_WINDOW1);
        // cv::namedWindow(OPENCV_WINDOW2);
    }

    void onLoaded() override
    {
        mind_os::NodeHandle& mos_nh = getPrivateNodeHandle();

        // sonar
        ultraPub_ = mos_nh.advertise<sensors_msgs::UltraData>("/ultra");
        ultraSub_ = ros_nh_.subscribe("/mobile_base/sensors/ultra", 
                                      10, 
                                      &RosSimSensorNode::publishUltra,
                                      this);
        
        // cliff
        cliffEventPub_ = mos_nh.advertise<sensors_msgs::CliffEvent>("/cliff_event");
        cliffEventSub_ = ros_nh_.subscribe("/mobile_base/events/cliff",
                                           10,
                                           &RosSimSensorNode::publishCliffEvent,
                                           this);
        
        // bumper
        bumperEventPub_ = mos_nh.advertise<sensors_msgs::BumperEvent>("/bumper_event");
        bumperEventSub_ = ros_nh_.subscribe("/mobile_base/events/bumper", 
                                            10, 
                                            &RosSimSensorNode::publishBumperEvent,
                                            this);
        
        // laser
        laserPub_ = mos_nh.advertise<sensors_msgs::LaserScan>("/scan");        
        laserSub_ = ros_nh_.subscribe("/simscan", 
                                      100, 
                                      &RosSimSensorNode::publishLaser,
                                      this);
        // camera
        stereoTXPub_ = mos_nh.advertise<sensors_msgs::ImgMsg>("img_tx");
        stereoIRPub_ = mos_nh.advertise<sensors_msgs::ImgMsg>("img_ir");
        stereoSub_ = ros_nh_.subscribe("/stereo_image",
                                        100,
                                        &RosSimSensorNode::publishStereo,
                                        this);

        // point cloud
        pclPub_ = mos_nh.advertise<sensors_msgs::TofDepthData>("/tofpointclound");
        pointCloud2Sub_ = ros_nh_.subscribe("/tof/depth/points", 
                                            100, 
                                            &RosSimSensorNode::publishPCL,
                                            this);
        // encoder
        encoderPub_ = mos_nh.advertise<sensors_msgs::EncoderMsg>("/encoder");
        encoderSub_ = ros_nh_.subscribe("/odom",
                                        10,
                                        &RosSimSensorNode::publishEncoder,
                                        this);
        // imu
        imuPub_ = mos_nh.advertise<sensors_msgs::ImuMsg>("/imu");
        imuSub_ = ros_nh_.subscribe("/mobile_base/sensors/imu",
                                    10,
                                    &RosSimSensorNode::publishImu,
                                    this);

        // vel
        cmd_vel_pub_ = ros_nh_.advertise<geometry_msgs::TwistConstPtr>("/cmd_vel", 1);
        cmd_vel_sub_ = mos_nh.subscribe<nav_msgs::SpeedControl>("/cmd_vel",
                                                                &RosSimSensorNode::publishCmdVel,
                                                                this);
    }

    void onUnloaded() override
    {
        LOG(INFO) << "Stop.";
    }

    void publishUltra(const rbn100_msgs::Ultra::ConstPtr &msg){
        sensors_msgs::UltraData ultraMsg;
        for (int i = 0; i < msg->distance.size(); i++)
        {
            ultraMsg.distance[i] = msg->distance[i];
        }
        ultraMsg.time = msg->head.stamp.toNSec();
        ultraPub_.publish(ultraMsg);
    }

    void publishCliffEvent(const rbn100_msgs::CliffEvent::ConstPtr msg){
        sensors_msgs::CliffEvent cliffEventMsg;
        cliffEventMsg.frameId = msg->header.seq;
        cliffEventMsg.sensorId = (sensors_msgs::CliffEvent::CliffSide)msg->which;
        cliffEventMsg.status = (sensors_msgs::CliffEvent::CliffStatus)msg->state;
        cliffEventMsg.timeStamp = msg->header.stamp.toNSec();
        cliffEventPub_.publish(cliffEventMsg);
    }
    
    void publishBumperEvent(const rbn100_msgs::BumperEvent::ConstPtr& msg){
        sensors_msgs::BumperEvent bumperEventMsg;
        bumperEventMsg.frameId = msg->header.seq;
        bumperEventMsg.sensorId = (sensors_msgs::BumperEvent::BumperSide)msg->bumper;
        bumperEventMsg.status = (sensors_msgs::BumperEvent::BumperStatus)msg->state;
        bumperEventMsg.timeStamp = msg->header.stamp.toNSec();
        bumperEventPub_.publish(bumperEventMsg);
    }

    void publishLaser(const sensor_msgs::LaserScan::ConstPtr& msg){
        auto laserMsg_ = std::make_shared<sensors_msgs::LaserScan>();
        laserMsg_->header.seq = msg->header.seq;
        laserMsg_->header.stamp = msg->header.stamp.toNSec();
        laserMsg_->angle_min = msg->angle_min;
        laserMsg_->angle_max = msg->angle_max;
        laserMsg_->angle_increment = msg->angle_increment;
        laserMsg_->time_increment = msg->time_increment;
        laserMsg_->scan_time = msg->scan_time;
        laserMsg_->range_min = msg->range_min;
        laserMsg_->range_max = msg->range_max;
        memcpy(laserMsg_->ranges, msg->ranges.data(), msg->ranges.size());
        memcpy(laserMsg_->intensities, msg->intensities.data(), msg->intensities.size());
        laserPub_.publish(laserMsg_);
    }

    void publishStereo(const rbn100_msgs::StereoImage::ConstPtr& msg){
        std::shared_ptr<sensors_msgs::ImgMsg> stereo_msg = std::make_shared<sensors_msgs::ImgMsg>();
        stereo_msg->sysTimeStamp = ros::Time::now().toNSec();
        stereo_msg->timeStamp = msg->header.stamp.toNSec();
        stereo_msg->imseeId = 0;
        stereo_msg->cameraWidth = msg->width;
        stereo_msg->cameraHeight = msg->height;
        stereo_msg->cameraFps = 25;
        stereo_msg->cameraIndex = msg->header.seq;
        stereo_msg->imgSize = msg->height * msg->width;
        stereo_msg->channel = 1;
        stereo_msg->timeExpose = 0;
        memcpy(stereo_msg->left, &msg->left[0], msg->step * msg->height);
        memcpy(stereo_msg->right, &msg->right[0], msg->step * msg->height);
        if(msg->header.seq % 2 == 1){
            stereo_msg->txIrStatus.txIrState = sensors_msgs::ImgMsg::TX_ON;
            stereo_msg->txIrStatus.txLightValue = 1;
            stereo_msg->txIrStatus.irLightValue = 0;
            stereo_msg->txIrStatus.envLightValue = 1;
            stereoTXPub_.publish(stereo_msg);
        }else{
            stereo_msg->txIrStatus.txIrState = sensors_msgs::ImgMsg::IR_ON;
            stereo_msg->txIrStatus.txLightValue = 0;
            stereo_msg->txIrStatus.irLightValue = 1;
            stereo_msg->txIrStatus.envLightValue = 1;
            stereoIRPub_.publish(stereo_msg);
        }
    }

    void publishPCL(const sensor_msgs::PointCloud2::ConstPtr &msg){
        std::shared_ptr<sensors_msgs::TofDepthData> pclData = std::make_shared<sensors_msgs::TofDepthData>();
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(*msg, *temp_cloud);
        int pointSize = temp_cloud->points.size();
        for (int i = 0; i < pointSize; i++){
            pclData->data[i].X = temp_cloud->points.at(i).x;
            pclData->data[i].Y = temp_cloud->points.at(i).y;
            pclData->data[i].Z = temp_cloud->points.at(i).z;
            pclData->data[i].noise = 0;
            pclData->data[i].grayValue = temp_cloud->points.at(i).r;
            pclData->data[i].depthConfidence = 0;
        }
        pclData->tofID = msg->header.seq;
        pclData->exposureTime = 0;
        pclData->width = msg->width;
        pclData->height = msg->height;
        pclData->frameIndex = msg->header.seq;
        pclData->time_stamp = msg->header.stamp.toNSec();
        
        pclPub_.publish(pclData);
    }

    void publishEncoder(const rbn100_msgs::Encoder::ConstPtr &msg){
        sensors_msgs::EncoderMsg encoder;
        encoder.EncodeL = msg->EncoderL;
        encoder.EncodeR = msg->EncoderR;
        encoder.timeStamp = msg->header.stamp.toNSec();
        encoderPub_.publish(encoder);
    }

    void publishImu(const sensor_msgs::Imu::ConstPtr &msg){
        sensors_msgs::ImuMsg imu;
        imu.sysTimeStamp = ros::Time::now().toNSec();
        imu.timeStamp = msg->header.stamp.toNSec();
        imu.accel[0] = msg->linear_acceleration.x;
        imu.accel[1] = msg->linear_acceleration.y;
        imu.accel[2] = msg->linear_acceleration.z;
        imu.gyro[0] = msg->angular_velocity.x;
        imu.gyro[1] = msg->angular_velocity.y;
        imu.gyro[2] = msg->angular_velocity.z;
        imuPub_.publish(imu);
    }

    void publishCmdVel(mind_os::ConstPtr<nav_msgs::SpeedControl>& msg){
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = msg->Speed_Angle;
        cmd_vel.angular.z = msg->Speed_Angle;
        cmd_vel_pub_.publish(cmd_vel);
    }

};  //end class RosSimSensorNode

REGISTER_PLUGIN(RosSimSensorNode)
