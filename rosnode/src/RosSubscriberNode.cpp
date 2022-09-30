/*
* RosSubscriberNode.h
*
* Created on: Aug 26, 2021
* Author: ubuntu
*/

#include <mind_os/mind_os.h>
#include <sensor_msgs/Range.h>
#include "rbn100_msgs/BumperEvent.h"
#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
#include <glog/logging.h>
#include "msg/sensor/LaserScan.h"
#include "msg/sensor/UltraData.h"
#include "msg/sensor/BumperEvent.h"

class RosSubscriberNode : public mind_os::NodePlugin
{
    // ros node handle
    ros::NodeHandle ros_nh_;
    
    // ros sub
    ros::Subscriber sonarFLSub_;
    ros::Subscriber sonarFrontSub_;
    ros::Subscriber sonarFRSub_;
    ros::Subscriber sonarBackSub_;
    ros::Subscriber bumperEventSub_;
    ros::Subscriber laserSub_;
    
    // ultra msg
    sensors_msgs::UltraData ultraMsg_;
    
    // mos pub
    mind_os::Publisher ultraPub_;
    mind_os::Publisher bumperEventPub_;
    mind_os::Publisher laserPub_;

public:
    RosSubscriberNode(){}

    void onLoaded() override
    {
        mind_os::NodeHandle& mos_nh = getPrivateNodeHandle();

        // sonar
        sonarFRSub_ = ros_nh_.subscribe<sensor_msgs::Range>("/sonar_FR", 100, &RosSubscriberNode::sonarFLCB, this);
        sonarFrontSub_ = ros_nh_.subscribe("/sonar_front", 100, &RosSubscriberNode::sonarFrontCB, this);
        sonarFLSub_ = ros_nh_.subscribe("/sonar_FL", 100, &RosSubscriberNode::sonarFRCB, this);
        sonarBackSub_ = ros_nh_.subscribe("/sonar_back", 100, &RosSubscriberNode::sonarBackCB, this);
        ultraPub_ = mos_nh.advertise<sensors_msgs::UltraData>("/ultra");
        ultraPub_.publish(ultraMsg_);
        
        // cliff ignore
        
        // bumper
        bumperEventPub_ = mos_nh.advertise<sensors_msgs::BumperEvent>("/bumper_event");
        bumperEventSub_ = ros_nh_.subscribe<rbn100_msgs::BumperEvent>("/mobile_base/events/bumper", 10, &RosSubscriberNode::publishBumperEvent, this);
        
        // laser
        laserPub_ = mos_nh.advertise<sensors_msgs::LaserScan>("/scan");        
        laserSub_ = ros_nh_.subscribe<sensor_msgs::LaserScan>("/simscan", 100, &RosSubscriberNode::publishLaser, this);

        // camera ignore

        // tof
        
    }

    void onUnloaded() override
    {
        LOG(INFO) << "Stop.";
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

    void sonarFLCB(const sensor_msgs::Range::ConstPtr& msg){
        ultraMsg_.distance[0] = msg->range;
        ultraMsg_.time = msg->header.stamp.toNSec();
    }
    void sonarFrontCB(const sensor_msgs::Range::ConstPtr& msg){
        ultraMsg_.distance[1] = msg->range;
    }
    void sonarFRCB(const sensor_msgs::Range::ConstPtr& msg){
        ultraMsg_.distance[2] = msg->range;
    }
    void sonarBackCB(const sensor_msgs::Range::ConstPtr& msg){
        ultraMsg_.distance[3] = msg->range;
    }
    
    void publishBumperEvent(const rbn100_msgs::BumperEvent::ConstPtr& msg){
        sensors_msgs::BumperEvent bumperEventMsg;
        bumperEventMsg.sensorId = (sensors_msgs::BumperEvent::BumperSide)msg->bumper;
        bumperEventMsg.status = (sensors_msgs::BumperEvent::BumperStatus)msg->state;
        bumperEventMsg.timeStamp = msg->stamp.toNSec();
        bumperEventPub_.publish(bumperEventMsg);
    }
};

REGISTER_PLUGIN(RosSubscriberNode)
