//
// Created by pengpeng on 11/18/20.
//

#include "core.h"

MatrixXd p_t, v_t, a_t;
VectorXd yaw_t;
Vector3d current_p(0,0,0);
Vector3d current_v(0,0,0);

Vector3d current_a(0,0,0);

Vector3d planned_p,planned_v,planned_a;
Vector3d last_planned_p(0,0,0);
double last_planned_az=0;
double planned_yaw;
unsigned int t_number=1;
geometry_msgs::PoseStamped pose;
int go_to_point_count=0;
ros::Time last_hover_time;
int time_flag=1;

double TargetPoint[6][10] =
        {
                ///x,   y,   z,yaw,vx,vy,vz,ax,ay,az
                {0.0,0.0,1.0, 0, 0, 0, 0, 0, 0, 0},//dot_1
                {0.0,0.0,3.0, 0, 0, 0, 0, 0, 0, 0},  //dot_3
                {0.0,0.0,5.0, 0,0, 0, 0, 0, 0, 0},//dot_5
                {0.0,0.0,7.0, 0,0, 0, 0, 0, 0, 0},//dot_7
                {3.0,0.0,1.0, 0,0, 0, 0, 0, 0, 0},//dot_9
                {3.0,0.0,1.0, 0,0, 0, 0, 0, 0, 0},  //dot_13
        };



int hover_count=0;
int hover_flag=1;

/*void setHoverPva()
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


}*/

bool hover_sec(int hover_sec)
{
    static geometry_msgs::PoseStamped hover_position;
    if (hover_flag==1)
    {
        hover_count=hover_sec*LOOPRATE;

        hover_flag=0;

        hover_position=dronePoseCurrent;
    }
    hover_count--;
    hover_position.header.stamp = ros::Time::now();
    local_pos_pub.publish(hover_position);

/*    setHoverPva();
    pubTargetPoint.publish(TargetPointMsg);*/

    if(hover_count<0)
    {
        hover_flag=1;
        return true;
    }
    return false;
}

/*
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
*/


/*bool isArrivedTarget(int pointnumber)
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
}*/


void motion_primitives_with_table(Vector3d p0,Vector3d v0,Vector3d a0,Vector3d pf,Vector3d vf,Vector3d af,unsigned int &t_num,
                                  double yawf)
{

    double T1, T2, T3, T;
    double delt_x, delt_y, delt_z;
    //ROS_INFO("pf(0):  %f,pf(1):   %f,pf(2):   %f",pf(0),pf(1),pf(2));
    delt_x=(pf(0)-p0(0));
    delt_y=(pf(1)-p0(1));
    delt_z=(pf(2)-p0(2));

    double v_set=0.5;//速度设为0.5m/s

    T1 = fabs(delt_x/v_set);
    T2 =fabs(delt_y/v_set);
    T3 = fabs(delt_z/v_set);
    T = T1 > T2 ? T1 : T2;
    T = T > T3 ? T : T3;
    T = T < 0.5 ? 0.5 : T;

    t_num=T/delta_t;  //number of dots
    p_t = Eigen::MatrixXd::Zero(t_num, 3);
    v_t = Eigen::MatrixXd::Zero(t_num, 3);
    a_t = Eigen::MatrixXd::Zero(t_num, 3);
    yaw_t = Eigen::VectorXd::Zero(t_num);

    //t = Eigen::VectorXd::Zero(t_num);

    for(int times = 0; times < t_num; times++)
    {
        p_t(times, 0) =p0(0)+delt_x/(T/delta_t)*times;
    }
    for(int times = 0; times < t_num; times++)
    {
        p_t(times, 1) =p0(1)+delt_y/(T/delta_t)*times;
    }
    for(int times = 0; times < t_num; times++)
    {
        p_t(times, 2) =p0(2)+delt_z/(T/delta_t)*times;
    }
    for(int times=0;times<t_num;times++)
    {
        yaw_t(times)=yawf;
    }

    //ROS_INFO("p_t(t_num-1,0):  %f,p_t(t_num-1,1):  %f,p_t(t_num-1,2):  %f",p_t(t_num-1,0),p_t(t_num-1,1),p_t(t_num-1,2));

}


bool go_to_point(int pointnumber)
{

    planned_p << TargetPoint[pointnumber][0], TargetPoint[pointnumber][1],TargetPoint[pointnumber][2];

    planned_yaw = TargetPoint[pointnumber][3];
    planned_v << TargetPoint[pointnumber][4], TargetPoint[pointnumber][5], TargetPoint[pointnumber][6];
    planned_a << TargetPoint[pointnumber][7], TargetPoint[pointnumber][8], TargetPoint[pointnumber][9];

    if(last_planned_p==planned_p&&last_planned_az==planned_a(2))
    {
    }
    else
    {
        ROS_INFO("motion primitives---");

        motion_primitives_with_table(current_p,current_v,current_a,planned_p,planned_v,planned_a,t_number,planned_yaw);
        ROS_INFO("T_NUMBER  %d",t_number);
        go_to_point_count=0;
    }

    last_planned_p=planned_p;
    last_planned_az=planned_a(2);

    if(go_to_point_count<t_number)
    {
       // ROS_INFO("T_NUMBER  %d",t_number);
        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = p_t(go_to_point_count,0);
        pose.pose.position.y = p_t(go_to_point_count,1);
        pose.pose.position.z = p_t(go_to_point_count,2);

        pose.pose.orientation.w=cos(yaw_t(go_to_point_count)/2);
        pose.pose.orientation.x=0;
        pose.pose.orientation.y=0;
        pose.pose.orientation.z=sin(yaw_t(go_to_point_count)/2);
        //ROS_INFO("Y::%f",pose.pose.position.y);
        local_pos_pub.publish(pose);
    }
    if(go_to_point_count>=t_number)
    {
        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = planned_p(0);
        pose.pose.position.y = planned_p(1);
        pose.pose.position.z = planned_p(2);

        pose.pose.orientation.w=cos(planned_yaw/2);
        pose.pose.orientation.x=0;
        pose.pose.orientation.y=0;
        pose.pose.orientation.z=sin(planned_yaw/2);
        local_pos_pub.publish(pose);

    }
    go_to_point_count++;

    if(fabs(dronePoseCurrent.pose.position.x-planned_p(0))<0.05 &&
       fabs(dronePoseCurrent.pose.position.y-planned_p(1))<0.05 &&
       fabs(dronePoseCurrent.pose.position.z-planned_p(2))<0.05)
    {

        ROS_INFO_THROTTLE(2,"HOVER AT POINT %d",pointnumber);
        if(time_flag==1)
        {
            last_hover_time=ros::Time::now();
            time_flag=0;
        }
        if(ros::Time::now() - last_hover_time > ros::Duration(5.0))
        {
       //     hover_atpoint=LOOPRATE*5.0;
            time_flag=1;
            return true;
        }
    //    hover_atpoint--;

    }
    return false;





/*    setPointPva(pointnumber);
    pubTargetPoint.publish(TargetPointMsg);

    if(isArrivedTarget(pointnumber))
    {
       // ROS_INFO("POINTNUMBER:%d",pointnumber);
        //ROS_INFO("----CURRENT Y:%F   %f", dronePoseCurrent.pose.position.y  ,TargetPointMsg.positions[1]);
        ROS_INFO_THROTTLE(3,"Arrived at point %d",pointnumber);
        if(hover_atpoint<0)
        {
            hover_atpoint=LOOPRATE*5.0;
            return true;
        }
        hover_atpoint--;

    }*/
//    return false;
}


/*void setoffbpva()
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



}*/



