/*
* RosSubscriberNode.h
*
* Created on: Aug 26, 2021
* Author: ubuntu
*/

#include <mind_os/mind_os.h>
#include <ros/ros.h>

#include "rbn100_msgs/Ultra.h"
#include "rbn100_msgs/BumperEvent.h"
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
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

#include "msg/sensor/UltraData.h"
#include "msg/sensor/BumperEvent.h"
#include "msg/sensor/LaserScan.h"
#include "msg/sensor/TofGrayImg.h"
#include "msg/sensor/TofPointCloud.h"
#include "msg/sensor/EncoderMsg.h"
#include "msg/sensor/ImuMsg.h"

#include <glog/logging.h>

// static const std::string OPENCV_WINDOW1 = "Image window"; //定义输入图像显示窗口名称
// static const std::string OPENCV_WINDOW2 = "Gray window"; //定义输出图像显示窗口名称

class RosSubscriberNode : public mind_os::NodePlugin
{
    // ros node handle
    ros::NodeHandle ros_nh_;
    
    // ros sub
    ros::Subscriber ultraSub_;
    ros::Subscriber bumperEventSub_;
    ros::Subscriber laserSub_;
    ros::Subscriber depthImageSub_;
    ros::Subscriber pointCloud2Sub_;
    ros::Subscriber encoderSub_;
    ros::Subscriber imuSub_;
    
    // mos pub
    mind_os::Publisher ultraPub_;
    mind_os::Publisher bumperEventPub_;
    mind_os::Publisher laserPub_;
    mind_os::Publisher tofGrayImgPub_;
    mind_os::Publisher pclPub_;
    mind_os::Publisher encoderPub_;
    mind_os::Publisher imuPub_;

public:
    RosSubscriberNode(){
        // 创建图像显示窗口
        // cv::namedWindow(OPENCV_WINDOW1);
        // cv::namedWindow(OPENCV_WINDOW2);
    }

    void onLoaded() override
    {
        mind_os::NodeHandle& mos_nh = getPrivateNodeHandle();

        // sonar
        ultraPub_ = mos_nh.advertise<sensors_msgs::UltraData>("/ultra");
        ultraSub_ = ros_nh_.subscribe<rbn100_msgs::Ultra>("/mobile_base/sensors/ultra", 
                                                          100, 
                                                          &RosSubscriberNode::publishUltra, 
                                                          this);
        
        // cliff ignore
        
        // bumper
        bumperEventPub_ = mos_nh.advertise<sensors_msgs::BumperEvent>("/bumper_event");
        bumperEventSub_ = ros_nh_.subscribe<rbn100_msgs::BumperEvent>("/mobile_base/events/bumper", 
                                                                      10, 
                                                                      &RosSubscriberNode::publishBumperEvent, 
                                                                      this);
        
        // laser
        laserPub_ = mos_nh.advertise<sensors_msgs::LaserScan>("/scan");        
        laserSub_ = ros_nh_.subscribe<sensor_msgs::LaserScan>("/simscan", 
                                                              100, 
                                                              &RosSubscriberNode::publishLaser, 
                                                              this);
        // camera ignore

        /* 
            tof
         */
        // gray img
        tofGrayImgPub_ = mos_nh.advertise<sensors_msgs::TofGrayImg>("/tofgrayimg");
        depthImageSub_ = ros_nh_.subscribe<sensor_msgs::Image>("/tof/depth/image_raw",
                                                               100,
                                                               &RosSubscriberNode::publishTofGrayImg,
                                                               this);
        // point cloud
        pclPub_ = mos_nh.advertise<sensors_msgs::TofPoinCloud>("/tofpointclound");
        pointCloud2Sub_ = ros_nh_.subscribe<sensor_msgs::PointCloud2>("/tof/depth/points", 
                                                                      100, 
                                                                      &RosSubscriberNode::publishPCL, 
                                                                      this);       
        // encoder
        encoderPub_ = mos_nh.advertise<sensors_msgs::EncoderMsg>("/encoder");
        encoderSub_ = ros_nh_.subscribe<rbn100_msgs::Encoder>("/odom",
                                                              100,
                                                              &RosSubscriberNode::publishEncoder,
                                                              this);
        // imu
        imuPub_ = mos_nh.advertise<sensors_msgs::ImuMsg>("/imu");
        imuSub_ = ros_nh_.subscribe<sensor_msgs::Imu>("/mobile_base/sensors/imu",
                                                      100,
                                                      &RosSubscriberNode::publishImu,
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
    
    void publishBumperEvent(const rbn100_msgs::BumperEvent::ConstPtr& msg){
        sensors_msgs::BumperEvent bumperEventMsg;
        bumperEventMsg.sensorId = (sensors_msgs::BumperEvent::BumperSide)msg->bumper;
        bumperEventMsg.status = (sensors_msgs::BumperEvent::BumperStatus)msg->state;
        bumperEventMsg.timeStamp = msg->stamp.toNSec();
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

    void publishTofGrayImg(const sensor_msgs::Image::ConstPtr &msg){
        std::shared_ptr<sensors_msgs::TofGrayImg> tofGray = std::make_shared<sensors_msgs::TofGrayImg>();
        cv_bridge::CvImagePtr cv_ptr;

        // ros byte stream to cv image
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);

        // cv::imshow(OPENCV_WINDOW1, cv_ptr->image);
        
        // convert data type, cvtcolor convert channels
        // float is 0-1.0, convert to uchar need scale to 255
        cv::Mat cvGrayImage;
        cv_ptr->image.convertTo(cvGrayImage, CV_8U, 255);

        // cv::imshow(OPENCV_WINDOW2, cvGrayImage);
        // cv::waitKey();
        
        int width = cvGrayImage.cols;
        int height = cvGrayImage.rows;
        uchar* pdata = cvGrayImage.data;
        for (int i = 0; i < height; i++){
            for (int j = 0; j < width; j++){
                tofGray->data[i*width + j] = pdata[j];
            }          
            pdata += width;
        }
        tofGray->tofID = msg->header.seq;
        tofGray->width = msg->width;
        tofGray->height = msg->height;
        tofGray->frameIndex = msg->header.seq;
        tofGray->time_stamp = msg->header.stamp.toNSec();

        tofGrayImgPub_.publish(tofGray);
    }

    void publishPCL(const sensor_msgs::PointCloud2::ConstPtr &msg){
        std::shared_ptr<sensors_msgs::TofPoinCloud> pclData = std::make_shared<sensors_msgs::TofPoinCloud>();
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *temp_cloud);
        int pointSize = temp_cloud->points.size();
        for (int i = 0; i < pointSize; i++){
            pclData->data[i].c = 0;
            pclData->data[i].X = temp_cloud->points.at(i).x;
            pclData->data[i].Y = temp_cloud->points.at(i).y;
            pclData->data[i].Z = temp_cloud->points.at(i).z;
            ROS_INFO_STREAM("point data: " << temp_cloud->points.at(i));
        }
        pclData->tofID = msg->header.seq;
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

};  //end class RosSubscriberNode

REGISTER_PLUGIN(RosSubscriberNode)
