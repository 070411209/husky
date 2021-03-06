// example_move_base_client: 
// wsn, October, 2016
// client of move_base

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include "xform_utils.h"


geometry_msgs::PoseStamped g_destination_pose;

void set_des_pose() {
    g_destination_pose.header.frame_id="base_link";
    g_destination_pose.header.stamp = ros::Time::now();
    g_destination_pose.pose.position.z=0;
    g_destination_pose.pose.position.x = 1;
    g_destination_pose.pose.position.y = 0;
    g_destination_pose.pose.orientation.x= 0; 
    g_destination_pose.pose.orientation.y= 0; 
    g_destination_pose.pose.orientation.z= 0; 
    g_destination_pose.pose.orientation.w= 1;
}

void navigatorDoneCb(const actionlib::SimpleClientGoalState& state,
        const move_base_msgs::MoveBaseResultConstPtr& result) {
    ROS_INFO(" navigatorDoneCb: server responded with state [%s]", state.toString().c_str());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "example_navigator_action_client"); // name this node 
    ros::NodeHandle nh; //standard ros node handle    
    set_des_pose(); //define values for via points
    tf::TransformListener tfListener;
    geometry_msgs::PoseStamped current_pose;
    move_base_msgs::MoveBaseGoal move_base_goal;
    XformUtils xform_utils; //instantiate an object of XformUtils
    
    bool tferr=true;
    ROS_INFO("waiting for tf between map and base_link...");
    tf::StampedTransform tfBaseLinkWrtMap; 
    while (tferr) {
        tferr=false;
        try {
                //try to lookup transform, link2-frame w/rt base_link frame; this will test if
                // a valid transform chain has been published from base_frame to link2
                tfListener.lookupTransform("odom","base_link", ros::Time(0), tfBaseLinkWrtMap);
            } catch(tf::TransformException &exception) {
                ROS_WARN("%s; retrying...", exception.what());
                tferr=true;
                ros::Duration(0.5).sleep(); // sleep for half a second
                ros::spinOnce();                
            }   
    }
    ROS_INFO("tf is good; current pose is:");
    current_pose = xform_utils.get_pose_from_stamped_tf(tfBaseLinkWrtMap);
    xform_utils.printStampedPose(current_pose);

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> navigator_ac("move_base", true);
    
    // attempt to connect to the server:
    ROS_INFO("waiting for move_base server: ");
    bool server_exists = false;
    while ((!server_exists)&&(ros::ok())) {
        server_exists = navigator_ac.waitForServer(ros::Duration(0.5)); // 
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ROS_INFO("retrying...");
    }
    ROS_INFO("connected to move_base action server"); // if here, then we connected to the server; 

    move_base_msgs::MoveBaseGoal goal;

    //we'll send a goal to the robot to move 2 meters forward
    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = -1.0;
    goal.target_pose.pose.position.y = 0.2;
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI);

    //geometry_msgs/PoseStamped target_pose
    move_base_goal.target_pose = g_destination_pose;         
    
    ROS_INFO("example sending goal: ");
    xform_utils.printStampedPose(g_destination_pose);
    navigator_ac.sendGoal(goal, &navigatorDoneCb); 

        
    bool finished_before_timeout = navigator_ac.waitForResult(ros::Duration(120.0));
    //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
    if (!finished_before_timeout) {
        ROS_WARN("giving up waiting on result ");
        return 1;
    }
        
    return 0;
}

