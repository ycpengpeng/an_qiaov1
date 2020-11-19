//
// Created by pengpeng on 11/18/20.
//

#include "core.h"


double TargetPoint[6][10] =
        {
                ///x,   y,   z,yaw,vx,vy,vz,ax,ay,az
                {0.0,1.0,1.0, 0, 0, 0, 0, 0, 0, 0},//dot_1
                {0.0,4.0,1.0, 0, 0, 0, 0, 0, 0, 0},  //dot_3
                {0.0,7.0,1.0, 0,0, 0, 0, 0, 0, 0},//dot_5
                {3.0,0.0,1.0, 0,0, 0, 0, 0, 0, 0},//dot_7
                {3.0,0.0,1.0, 0,0, 0, 0, 0, 0, 0},//dot_9
                {3.0,0.0,1.0, 0,0, 0, 0, 0, 0, 0},  //dot_13
        };



int hover_count=0;
int hover_flag=1;

void setHoverPva()
{
    TargetPointMsg.positions.clear();
    TargetPointMsg.velocities.clear();
    TargetPointMsg.accelerations.clear();
    TargetPointMsg.effort.clear();

    TargetPointMsg.positions.push_back(1.1);
    TargetPointMsg.positions.push_back(1.2);
    TargetPointMsg.positions.push_back(1.3);
    TargetPointMsg.positions.push_back(0);

    TargetPointMsg.velocities.push_back(0);
    TargetPointMsg.velocities.push_back(0);
    TargetPointMsg.velocities.push_back(0);

    TargetPointMsg.accelerations.push_back(0);
    TargetPointMsg.accelerations.push_back(0);
    TargetPointMsg.accelerations.push_back(0);

    TargetPointMsg.effort.push_back(-3);


}

bool hover_sec(int hover_sec)
{
    if (hover_flag==1)
    {
        hover_count=hover_sec*LOOPRATE;
        hover_flag=0;
    }
    hover_count--;
    setHoverPva();
    pubTargetPoint.publish(TargetPointMsg);
    if(hover_count<0)
    {
        hover_flag=1;
        return true;
    }
    return false;
}

void setPointPva(int pointnumber)
{
    TargetPointMsg.positions.clear();
    TargetPointMsg.velocities.clear();
    TargetPointMsg.accelerations.clear();
    TargetPointMsg.effort.clear();

    TargetPointMsg.positions.push_back(TargetPoint[pointnumber][0]);
    TargetPointMsg.positions.push_back(TargetPoint[pointnumber][1]);
    TargetPointMsg.positions.push_back(TargetPoint[pointnumber][2]);
    TargetPointMsg.positions.push_back(0);

    TargetPointMsg.velocities.push_back(0);
    TargetPointMsg.velocities.push_back(0);
    TargetPointMsg.velocities.push_back(0);

    TargetPointMsg.accelerations.push_back(0);
    TargetPointMsg.accelerations.push_back(0);
    TargetPointMsg.accelerations.push_back(0);

    TargetPointMsg.effort.push_back(pointnumber);

}


bool isArrivedTarget(int pointnumber)
{
    if(fabs(dronePoseCurrent.pose.position.x-TargetPointMsg.positions[0])<0.05 &&
       fabs(dronePoseCurrent.pose.position.y-TargetPointMsg.positions[1])<0.05 &&
       fabs(dronePoseCurrent.pose.position.z-TargetPointMsg.positions[2])<0.05)
    {
        return true;
    }
    else
    {
        return false;
    }
}


bool go_to_point(int pointnumber)
{
    setPointPva(pointnumber);
    pubTargetPoint.publish(TargetPointMsg);

    if(isArrivedTarget(pointnumber))
    {
        ROS_INFO("POINTNUMBER:%d",pointnumber);
        ROS_INFO("----CURRENT Y:%F   %f", dronePoseCurrent.pose.position.y  ,TargetPointMsg.positions[1]);
        ROS_INFO("ABS:%F",fabs(dronePoseCurrent.pose.position.y-TargetPointMsg.positions[1]));
        return true;
    }
    return false;
}


void setoffbpva()
{
    TargetPointMsg.positions.clear();
    TargetPointMsg.velocities.clear();
    TargetPointMsg.accelerations.clear();
    TargetPointMsg.effort.clear();

    TargetPointMsg.positions.push_back(0);
    TargetPointMsg.positions.push_back(0);
    TargetPointMsg.positions.push_back(0.001);
    TargetPointMsg.positions.push_back(0);

    TargetPointMsg.velocities.push_back(0);
    TargetPointMsg.velocities.push_back(0);
    TargetPointMsg.velocities.push_back(0);

    TargetPointMsg.accelerations.push_back(0);
    TargetPointMsg.accelerations.push_back(0);
    TargetPointMsg.accelerations.push_back(0);

    TargetPointMsg.effort.push_back(0);



}



