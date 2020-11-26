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





Quaterniond current_att;
mavros_msgs::State current_state;
ros::Publisher pva_pub;
Vector3d last_current_v(0,0,0);
Vector3d last_a(0,0,0);
ros::Time last_time;
int stateStep=1;
int offb_flag=1;

int numberofpoint;
nav_msgs::Path path_point;
int init_read_path=1;
double motion_primitive_flag=0;

void positionCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{

    dronePoseCurrent.pose.position.x = msg->pose.position.x;
    dronePoseCurrent.pose.position.y = msg->pose.position.y;
    dronePoseCurrent.pose.position.z = msg->pose.position.z;
    dronePoseCurrent.pose.orientation.x=msg->pose.orientation.x;
    dronePoseCurrent.pose.orientation.y=msg->pose.orientation.y;
    dronePoseCurrent.pose.orientation.z=msg->pose.orientation.z;
    dronePoseCurrent.pose.orientation.w=msg->pose.orientation.w;

    current_p(0)=msg->pose.position.x;
    current_p(1)=msg->pose.position.y;
    current_p(2)=msg->pose.position.z;


    //ROS_INFO("X:%f Y:%f  Z:%f",-current_p(0),current_p(1),current_p(2));
}

void velocity_sub_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    current_v << msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z;
    current_a=(current_v-last_current_v)/(ros::Time::now().toSec()-last_time.toSec());
    if(current_a(0)>100||current_a(0)<-100)
    {
        current_a=last_a;
    }
    last_time=ros::Time::now();
    last_current_v=current_v;
    last_a=current_a;
}

void path_cb(const nav_msgs::Path::ConstPtr& msg)
{
    if(init_read_path==1)
    {


        path_point=*msg;
        numberofpoint= path_point.poses.size();
        ROS_INFO("----- numberofpoint %d",numberofpoint);
        init_read_path=0;
        ROS_INFO("-------read path point success----");

    }

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

    ros::Subscriber velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity_local", 1, velocity_sub_cb);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    pubTargetPoint = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("/set_point", 1);

    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1);

    ros::Subscriber path_sub=nh.subscribe<nav_msgs::Path>("/quayside/target_point", 1, path_cb);

    cur_point_pub= nh.advertise<std_msgs::String>("/cur_point_name", 1);



    ros::Rate loop_rate(LOOPRATE);



    int counter = 0;
    int take_off_init=1;

    Vector3d take_off_position;
    last_time=ros::Time::now();
    // wait for FCU connection

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ros::Time last_request = ros::Time::now();


    double yaw_set = 3.1415926;
    int mode=0;
    double z_sp=0;
    double vz_sp=0;
    ROS_INFO("an_qiao_simu_testv1 is running");

    while(init_read_path==1)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    int offb_init=0;
    int point=0;

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
        if(current_state.mode != "OFFBOARD" || !current_state.armed)
        {
            offb_init=1;
      //      ROS_INFO("22");
            ros::spinOnce();
            dronePoseCurrent.header.stamp = ros::Time::now();
            local_pos_pub.publish(dronePoseCurrent);
            offb_flag=0;
            continue;
        }

        //每次进入off_board后先在原地悬停5秒钟
        if(offb_init==1)
        {
            if(hover_sec(5)==0)
            {
                ROS_INFO_THROTTLE(5,"into offboard HOVER!!!");
                continue;
            }

/*            for(int i=0;i<=5;i++)
            {
                TargetPoint[i][9]+=0.001;
            }*/

            motion_primitive_flag+=0.001;
            ROS_INFO("finish into offboard HOVER!!!");
            ROS_ERROR("go to point %d",point);
        }
        offb_init=0;

        if(point>=numberofpoint)
        {
            point=numberofpoint;
        }

        if(go_to_point(point))
        {

            point++;
            ROS_ERROR("go to point %d",point);

        }


/*        switch (stateStep)
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
                ROS_ERROR_ONCE("  ------go to point 0");

                if(go_to_point(0))
                {
                    stateStep++;
                }
                break;
            }

            case 2:
            {
                ROS_ERROR_ONCE("   -----------go to point 1");

                if(go_to_point(1))
                {
                    stateStep++;
                }
                break;
            }


            case 3:
            {
                ROS_ERROR_ONCE(" -------- go to point 2!!!");

                if(go_to_point(2))
                {
                    stateStep++;
                }
                break;
            }

            case 4:
            {
                ROS_ERROR_ONCE(" ---------- go to point 3!!!");

                if(go_to_point(3))
                {
                    stateStep=4;
                }
                break;
            }


        }*/


    }
    return 0;
}

