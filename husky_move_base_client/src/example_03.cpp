#include "ros/ros.h"
#include "track_test.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "track_test");
    trackTest test;
    ros::spin();
    return 0;
}