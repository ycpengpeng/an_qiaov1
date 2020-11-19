//
// Created by pengpeng on 11/19/20.
//

#ifndef AN_QIAOV1_AN_QIAO_ACTUATOR_H
#define AN_QIAOV1_AN_QIAO_ACTUATOR_H



//
// Created by pengpeng on 11/1/20.
//

#ifndef PVA_TRACKER_PUB_ZUAN_QUAN_H
#define PVA_TRACKER_PUB_ZUAN_QUAN_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/OpticalFlowRad.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <iostream>
#include <string>
#include <fstream>
#include <cmath>
#include <Eigen/Eigen>
#include <algorithm>
#include <chrono>
#include <cstdint>

#define PI 3.1415926
#define LOOPRATE 40
#define delta_t 0.025




typedef struct pva_table{
    int dim_num;
    int *dim_size;
    uint64_t *dim_interval;
    uint64_t table_size;
    double *table;
    double rez;
    double *pva_limit;

    void construct_pva_table(int dim1_size, int dim2_size, int dim3_size, int dim4_size, double resolution) {
        this->dim_num = 4;
        this->dim_size = (int*)malloc(sizeof(int)*this->dim_num);
        this->dim_interval = (uint64_t*)malloc(sizeof(uint64_t)*this->dim_num);

        this->dim_size[0] = dim1_size;
        this->dim_size[1] = dim2_size;
        this->dim_size[2] = dim3_size;
        this->dim_size[3] = dim4_size;

        this->dim_interval[3] = 1;
        this->dim_interval[2] = dim_interval[3] * dim4_size;
        this->dim_interval[1] = dim_interval[2] * dim3_size;
        this->dim_interval[0] = dim_interval[1] * dim2_size;

        this->table_size = this->dim_interval[0] * dim1_size;
        this->table = (double*)malloc(sizeof(double)*this->table_size);

        this->rez = resolution;

        this->pva_limit = (double*)malloc(sizeof(double)*3);
        this->pva_limit[0] = this->rez*double(dim1_size/2);
        this->pva_limit[1] = this->rez*double(dim2_size/2);
        this->pva_limit[2] = this->rez*double(dim4_size/2);
    }

    void compute_idx_from_pva(double dlt_p, double v0, double vf, double a0,
                              int &idx1, int &idx2, int &idx3, int &idx4) {
        idx1 = round(dlt_p/this->rez) + this->dim_size[0]/2;
        idx2 = round(v0/this->rez) + this->dim_size[1]/2;
        idx3 = round(vf/this->rez) + this->dim_size[2]/2;
        idx4 = round(a0/this->rez) + this->dim_size[3]/2;
    }

    double query_pva_table(double dlt_p, double v0, double vf, double a0)
    {
        if (fabs(dlt_p) > this->pva_limit[0])
        {
            dlt_p=pva_limit[0]*fabs(dlt_p)/dlt_p ;
            //ROS_INFO("Invalid input!,dlt_p so large");
        }
        if (fabs(v0) > this->pva_limit[1])
        {
            v0=pva_limit[1]*fabs(v0)/v0 ;
            //ROS_INFO("Invalid input! v0 so large");
        }
        if (fabs(vf) > this->pva_limit[1])
        {
            vf=pva_limit[1]*fabs(vf)/vf ;
            //ROS_INFO("Invalid input! vf so large");
        }
        if (fabs(a0) > this->pva_limit[2])
        {
            a0=pva_limit[0]*fabs(a0)/a0 ;
            //ROS_INFO("Invalid input! a0 so large");
        }

        int idx1, idx2, idx3, idx4;
        this->compute_idx_from_pva(dlt_p, v0, vf, a0, idx1, idx2, idx3, idx4);

        uint64_t idx = idx1*this->dim_interval[0] + idx2*this->dim_interval[1] +
                       idx3*this->dim_interval[2] + idx4*this->dim_interval[3];

        // std::cout << "idx: " << idx << std::endl;

        return this->table[idx];
    }

    void pva_table2csv(const std::string &str) {
        std::ofstream outfile;
        outfile.open(str, std::ios::out);

        for (int i = 0; i < 4; ++i) outfile << std::to_string(this->dim_size[i]) << ',';
        outfile << std::to_string(this->rez) << std::endl;

        for (uint64_t i = 0; i < this->table_size-1; ++i) outfile << std::to_string(this->table[i]) << ',';
        outfile << std::to_string(this->table[table_size-1]);

        outfile.close();
    }

    void csv2pva_table(const std::string &str) {
        int tmp_dim_size[4];
        double tmp_rez;

        std::ifstream infile(str, std::ios::in);
        std::string tmp_str;

        for (int i = 0; i < 4; ++i) {
            getline(infile, tmp_str, ',');
            tmp_dim_size[i] = std::stoi(tmp_str);
        }

        getline(infile, tmp_str);
        tmp_rez = std::stod(tmp_str);

        this->construct_pva_table(tmp_dim_size[0], tmp_dim_size[1],
                                  tmp_dim_size[2], tmp_dim_size[3], tmp_rez);

        for (uint64_t i = 0; i < this->table_size; ++i) {
            getline(infile, tmp_str, ',');
            this->table[i] = std::stod(tmp_str);
        }
    }

    void free_pva_table() {
        free(this->pva_limit);
        free(this->table);
        free(this->dim_interval);
        free(this->dim_size);
    }
}pva_table;


typedef struct euler_angle
{
    float yaw;
    float pitch;
    float roll;
}euler;

//欧拉角转四元数
void euler2qua(geometry_msgs::PoseStamped *p,euler ang)
{
    float croll2=cos(ang.roll/2);
    float cyaw2=cos(ang.yaw/2);
    float cpitch2=cos(ang.pitch/2);

    float sroll2=sin(ang.roll/2);
    float syaw2=sin(ang.yaw/2);
    float spitch2=sin(ang.pitch/2);

    p->pose.orientation.w=croll2*cpitch2*cyaw2+sroll2*spitch2*syaw2;

    p->pose.orientation.x=sroll2*cpitch2*cyaw2-croll2*spitch2*syaw2;

    p->pose.orientation.y=croll2*spitch2*cyaw2+sroll2*cpitch2*syaw2;

    p->pose.orientation.z=croll2*cpitch2*syaw2+sroll2*spitch2*cyaw2;

}



#endif //PVA_TRACKER_PUB_ZUAN_QUAN_H




#endif //AN_QIAOV1_AN_QIAO_ACTUATOR_H
