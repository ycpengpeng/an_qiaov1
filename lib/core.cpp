//
// Created by pengpeng on 11/19/20.
//
#include "core.h"

ros::Publisher pubTargetPoint;
ros::Publisher local_pos_pub;
trajectory_msgs::JointTrajectoryPoint TargetPointMsg;
geometry_msgs::PoseStamped dronePoseCurrent;
int hover_atpoint=200;