#ifndef TRACK_TEST_H_
#define TRACK_TEST_H_

#include <iostream>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/PoseStamped.h"

class trackTest
{
public:
    trackTest();
    void targetCallback_(const sensor_msgs::PointCloudConstPtr _msg);

private:
    ros::NodeHandle node_;

    ros::Publisher chatter_pub;
    
    ros::Subscriber target_sub_;

    geometry_msgs::PoseStamped move_target;

    float fix_dist = 4;
};

#endif
