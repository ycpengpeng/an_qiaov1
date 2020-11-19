//
// Created by pengpeng on 11/4/20.
//

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <Eigen/Eigen>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

#include <core.h>


using namespace Eigen;

Vector3d current_p;
Quaterniond current_att;
mavros_msgs::State current_state;
ros::Publisher pva_pub;

int stateStep=0;


void positionCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{

    dronePoseCurrent.pose.position.x = msg->pose.position.x;
    dronePoseCurrent.pose.position.y = msg->pose.position.y;
    dronePoseCurrent.pose.position.z = msg->pose.position.z;
    dronePoseCurrent.pose.orientation.x=msg->pose.orientation.x;
    dronePoseCurrent.pose.orientation.y=msg->pose.orientation.y;
    dronePoseCurrent.pose.orientation.z=msg->pose.orientation.z;
    dronePoseCurrent.pose.orientation.w=msg->pose.orientation.w;

    //ROS_INFO("X:%f Y:%f  Z:%f",-current_p(0),current_p(1),current_p(2));
}


void stateCallback(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "an_qiao_simu_testv1");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 1, stateCallback);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, positionCallback);


    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    pubTargetPoint = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("/set_point", 1);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1);


    ros::Rate loop_rate(LOOPRATE);

    double take_off_height = 1.5;
    double take_off_acc = 0.1;
    double take_off_time=sqrt(take_off_height/take_off_acc)*2.0;
    double delt_t = 1.0 / LOOPRATE;
    double take_off_send_times = take_off_time / delt_t;
    int counter = 0;
    int take_off_init=1;

    Vector3d take_off_position;
    ros::Time last_time=ros::Time::now();
    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        loop_rate.sleep();
    }
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ros::Time last_request = ros::Time::now();
    geometry_msgs::PoseStamped pose;

    double yaw_set = 3.1415926;
    int mode=0;
    double z_sp=0;
    double vz_sp=0;
    ROS_INFO("Arm and ta1111keoff");
    int offb_init=1;

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
        if(current_state.mode != "OFFBOARD" || !current_state.armed)
        {
            offb_init=1;
            local_pos_pub.publish(dronePoseCurrent);

            continue;
        }

        //每次进入off_board后先在原地悬停5秒钟
        if(offb_init==1)
        {
            if(hover_sec(5)==0)
            {
                ROS_INFO_THROTTLE(5,"HOVER!!!");
                continue;
            }
            for(int i=0;i<=5;i++)
            {
                TargetPoint[i][9]+=0.001;
            }
        }
        offb_init=0;

        switch (stateStep)
        {
            case 0:
            {
                ROS_INFO_ONCE("CASE 0  hover!!!");
                if(hover_sec(5))
                {
                    stateStep++;
                }
                break;
            }
            case 1:
            {
                ROS_INFO_ONCE("CASE 1!!!  go to point 0");

                if(go_to_point(0))
                {
                    stateStep++;
                }
                break;
            }
            case 2:
            {
                ROS_INFO_ONCE("CASE 2  hover!!!");


                if(hover_sec(10))
                {
                    stateStep++;
                }
                break;
            }
            case 3:
            {
                ROS_INFO_ONCE("CASE 3!!!   go to point 1");

                if(go_to_point(1))
                {
                    stateStep++;
                }
                break;
            }

            case 4:
            {
                ROS_INFO_ONCE("CASE 4  hover!!!");
               // ROS_INFO_ONCE("CURRENT Y:%F", dronePoseCurrent.pose.position.y  );

                if(hover_sec(10))
                {
                    stateStep++;
                }
                break;
            }

            case 5:
            {
                ROS_INFO_ONCE("CASE 5  go to point 2!!!");

                if(go_to_point(2))
                {
                    stateStep=5;
                }
                break;
            }


        }


    }
    return 0;
}

