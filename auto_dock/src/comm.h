

#ifndef COMM_H
#define COMM_H

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <fstream>
using namespace std;
#define PI                      3.1415926
#define DTR                     PI/180.0        //Degree to Rad
#define RTD                     180.0/PI        //Rad to Degree

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>


class GetTFPose
{
  public:
    GetTFPose();
    GetTFPose(double start_delay_time, double freq);
    ~GetTFPose();
    void GetPose(std::string source_frame, std::string target_frame, double *out_x, double *out_y, double *out_z, double *out_yaw, bool *out_flag);
    void init(double start_delay_time, double freq);

    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
    double now_time;
    bool rev_flag;

  private:
    tf::TransformListener tf_;
    bool first_delay_flag;
    int rev_delay;
    int rev_delay_cnt;
    bool debug_flag;
};

GetTFPose::GetTFPose()
{
    init(5.0, 10.0);
}

GetTFPose::GetTFPose(double start_delay_time, double freq)
{
    init(start_delay_time, freq);
}

GetTFPose::~GetTFPose()
{
}

void GetTFPose::init(double start_delay_time, double freq)
{
    rev_delay=(int)(start_delay_time*freq);
    rev_delay_cnt=0;
    first_delay_flag=0;
}

void GetTFPose::GetPose(std::string source_frame, std::string target_frame, double *out_x, double *out_y, double *out_z, double *out_yaw, bool *out_flag)
{
    //Get Pose from TF listener
    //printf("first_delay_flag=%d, rev_delay_cnt=%d rev_delay=%d\n", first_delay_flag, rev_delay_cnt, rev_delay);
    if (first_delay_flag)
    {
        tf::StampedTransform rev_tf;
        rev_flag=0;
        try{
            tf_.lookupTransform(source_frame, target_frame, ros::Time(0), rev_tf);
            x=rev_tf.getOrigin().x();
            y=rev_tf.getOrigin().y();
            z=rev_tf.getOrigin().z();
            yaw=tf::getYaw(rev_tf.getRotation());
            now_time=ros::Time::now().toSec();
            rev_flag=1;
            *out_x=x;
            *out_y=y;
            *out_z=z;
            *out_yaw=yaw;
            *out_flag=rev_flag;
        }
        catch (tf::TransformException ex){
            if (debug_flag)
                ROS_ERROR("%s",ex.what());
        }
    }
    else
    {
        rev_delay_cnt++;
        if (rev_delay_cnt>=rev_delay)
        {
            rev_delay_cnt=rev_delay;
            first_delay_flag=1;
        }
    }
}



//Euler angle to Quaternion
geometry_msgs::Quaternion Euler_to_Quat(double roll, double pitch, double yaw)
{
    tf::Quaternion tf_quat;
    geometry_msgs::Quaternion msg_quat;
    tf_quat=tf::createQuaternionFromRPY(roll, pitch, yaw);
    tf::quaternionTFToMsg(tf_quat, msg_quat);
    return msg_quat;
}

//Quaternion to one Euler angle
double Quat_to_Euler(geometry_msgs::Quaternion msg_quat, int sel)
{
    tf::Quaternion tf_quat;
    double roll, pitch, yaw;
    tf::quaternionMsgToTF(msg_quat, tf_quat);
    tf::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);

    if (sel==0)
        return roll;
    else if (sel==1)
        return pitch;
    else if (sel==2)
        return yaw;
}



//Quaternion to Yaw
double Quat_to_Yaw(geometry_msgs::Quaternion msg_quat)
{
    double yaw;
    //TF Function Converte to Yaw Angle (Euler Angle)
    geometry_msgs::Pose msg_pose;
    msg_pose.orientation=msg_quat;
    tf::Pose tf_pose;
    tf::poseMsgToTF(msg_pose, tf_pose);
    yaw = tf::getYaw(tf_pose.getRotation());
    return yaw;
}



//Calculate Distance in XY plane
double DIS_XY(double pa_x, double pa_y, double pb_x, double pb_y)
{
    double distance;
    distance=pow(pow(pa_x-pb_x,2)+pow(pa_y-pb_y,2),0.5);
    return distance;
}

#endif // COMM_H
