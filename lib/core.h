//
// Created by pengpeng on 11/18/20.
//

#ifndef AN_QIAOV1_CORE_H
#define AN_QIAOV1_CORE_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
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

#define LOOPRATE 40.0

extern trajectory_msgs::JointTrajectoryPoint TargetPointMsg;
extern ros::Publisher pubTargetPoint;
extern geometry_msgs::PoseStamped dronePoseCurrent;

bool hover_sec(int hover_sec);

bool go_to_point(int pointnumber);
void setoffbpva();


extern double TargetPoint[6][10];




#endif //AN_QIAOV1_CORE_H
