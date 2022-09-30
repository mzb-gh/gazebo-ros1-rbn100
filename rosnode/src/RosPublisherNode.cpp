/*
 * RosPublisherNode.h
 *
 *  Created on: Aug 26, 2021
 *      Author: ubuntu
 */

#include <mind_os/mind_os.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
#include <glog/logging.h>

class RosPublisherNode : public mind_os::NodePlugin
{
    std::shared_ptr<ros::NodeHandle> nh;
    ros::Publisher pubScan;
    mind_os::util::LoopThread* pubScanTh;

public:
    RosPublisherNode()
    {
        nh = std::make_shared<ros::NodeHandle>();

        pubScanTh = new mind_os::util::LoopThread(15, [this](){
            sensor_msgs::LaserScan msg;
            // pubScan.publish(msg);
        });
    }


    void onLoaded() override
    {
        pubScan = nh->advertise<sensor_msgs::LaserScan>("/scan", 1000);

        // pubScanTh->start();
        LOG(INFO) << "Start.";
    }

    void onUnloaded() override
    {
        pubScanTh->stop();
        LOG(INFO) << "Stop.";
    }
};

REGISTER_PLUGIN(RosPublisherNode)
