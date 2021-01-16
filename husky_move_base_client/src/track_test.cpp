#include "track_test.h"
#include <cmath>

trackTest::trackTest(){
    target_sub_ = node_.subscribe("/target_pose", 1, &trackTest::targetCallback_, this);
    chatter_pub = node_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10); //initialize /move_base_simple/goal
}

void trackTest::targetCallback_(const sensor_msgs::PointCloudConstPtr _msg) {
    sensor_msgs::PointCloud _goal_point;
    
    float x_ = _msg->points[0].x;
    float y_ = _msg->points[0].y;
    float z_ = _msg->points[0].z;
    float x_d = 0.0;
    float y_d = 0.0;
    float d_ = pow(pow(x_, 2) + pow(y_, 2), 0.5);
    double r_ = atan2((double)y_, (double)x_);
    if(d_ > fix_dist) {
        x_d = x_ - (fix_dist * x_)/d_;
        y_d = y_ - (fix_dist * y_)/d_;        
    } 
    else {
        x_d = 0.0;
        y_d = 0.0;
    }
     
    ROS_INFO("The position(x,y,z) is %f , %f, %f, d=%f", x_, y_, z_, d_);

    
    ROS_INFO("yaw : %f, x_d : %f, y_d : %f", r_, x_d, y_d);
    //First assign value to "header".
    ros::Time currentTime = ros::Time::now();
    move_target.header.stamp = currentTime;
    move_target.header.frame_id = "base_link";
    //Then assign value to "pose", which has member position and orientation
    move_target.pose.position.x = x_d;
    move_target.pose.position.y = y_d;
    move_target.pose.position.z = 0.0;

    move_target.pose.orientation = tf::createQuaternionMsgFromYaw(r_);

    chatter_pub.publish(move_target);   
    sleep(1);    
}