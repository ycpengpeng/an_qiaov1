//
// Created by pengpeng on 11/26/20.
//

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Altitude.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <nav_msgs/Path.h>

#include <cstdlib>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <math.h>
#include "string"
#include <time.h>
#include <queue>
#include <vector>

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <chrono>



int main(int argc, char** argv)
{
    ros::init(argc, argv, "an_qiao_pub_test");
    ros::NodeHandle nh;
    ros::Rate loop_rate(40);

    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("zph", 1);


  //  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 1, stateCallback);
  //  ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1,          positionCallback);

  //  ros::Subscriber velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity_local", 1,        velocity_sub_cb);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

   // pubTargetPoint = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("/set_point", 1);

   // local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1);


    nav_msgs::Path path;
    path.header.frame_id=1;
    geometry_msgs::PoseStamped pose;

    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1;
    path.poses.push_back(pose);

    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 3;
    path.poses.push_back(pose);


    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 5;
    path.poses.push_back(pose);


    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 7;
    path.poses.push_back(pose);



    while(ros::ok())
    {
        path_pub.publish(path);
        loop_rate.sleep();

    }

    return 0;


}