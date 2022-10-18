#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <cmath>
#include <chrono>

#define _USE_MATH_DEFINES

// ROS
ros::Publisher vel_pub;

//parameter setting
std::string dock_base_frame_id;
double dock_min_v, dock_min_w, dock_max_v, dock_max_w;
double dist_to_dock, dist_to_center;
double dock_threshold_v, dock_threshold_w;

int step = 0;

bool check_angle = false;

static double clamp(double v, double v_min, double v_max)
{
    return std::min(std::max(v_min,v),v_max);
}


// Setting linear and angular velocity
void setVel(float x, float y, float yaw){
    static geometry_msgs::Twist vel_msg;

    if (step == 0){
        vel_msg.linear.x = 0;
        vel_msg.linear.y = 0;
        ROS_INFO("adjusting angle......");
        if(fabs(yaw)<0.03){
                vel_msg.angular.z = 0;
                vel_pub.publish(vel_msg);
                step = 1;
                check_angle = true;
        }
    else if((yaw<0)&&(yaw>=-dock_threshold_w)){
        vel_msg.angular.z = -dock_min_w; //turn right
    }
    else if((yaw>0)&&(yaw<=dock_threshold_w)){
        vel_msg.angular.z = dock_min_w; //turn left
    }
    else if(yaw>dock_threshold_w){
        vel_msg.angular.z = dock_max_w;   
    }
    else{
        vel_msg.angular.z = -dock_max_w;
    }
    }
    
    else if (step == 1){
        ROS_INFO("ajusting pose to center......");
          vel_msg.linear.x = 0;
          vel_msg.angular.z = 0;
        if (fabs(y) <= dist_to_center){
            vel_msg.linear.y=0;
            vel_pub.publish(vel_msg);
            step = 2;
        }
        else if ((y>-dist_to_center) && (y<0)){
            vel_msg.linear.y = -dock_min_v; //move right
        }
        else if ((y<dist_to_center)&&(y>0)){
            vel_msg.linear.y = dock_min_v; //move left
        }
        else if (y>dist_to_center){
           vel_msg.linear.y = dock_max_v; //move left
        }
        else {
            vel_msg.linear.y = -dock_max_v;
        }
    }
    else if (step == 2){
        ROS_INFO("docking in......");
          vel_msg.linear.y = 0;
          vel_msg.angular.z = 0;
        if (fabs(x)<=0.03){
            vel_msg.linear.x =0;
            vel_pub.publish(vel_msg);
            step = 3;
        }
        else if (fabs(x)<=(dist_to_dock+dock_threshold_v)){
            vel_msg.linear.x = dock_min_v;
        }
        else {
            vel_msg.linear.x = dock_max_v;
        }
       
    }
    else if (step == 3){
        vel_msg.linear.x = 0;
        vel_msg.linear.y = 0;
        vel_msg.angular.z = 0;
        vel_pub.publish(vel_msg);
        ROS_INFO("Finish Docking!");
    }
    vel_pub.publish(vel_msg);
}


int main(int argc, char** argv){
    ros::init(argc, argv, "controller_node");
    ros::NodeHandle n;

    // Load Parameters
    n.getParam("/controller/dock_base_frame_id",dock_base_frame_id);
    n.getParam("/controller/dock_min_v",dock_min_v);
    n.getParam("/controller/dock_min_w",dock_min_w);
    n.getParam("/controller/dock_max_v",dock_max_v);
    n.getParam("/controller/dock_max_w",dock_max_w);
    n.getParam("/controller/dist_to_dock",dist_to_dock);
    n.getParam("/controller/dist_to_center",dist_to_center);
    n.getParam("/controller/dock_threshold_v",dock_threshold_v);
    n.getParam("/controller/dock_threshold_w",dock_threshold_w);


    //ros::Subscriber lr_sub_ = n.subscribe("/l_or_r", 10, lrCallback);
    vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 20);
    tf::TransformListener listener_dock;
    ros::Rate rate(20.0);
    while(ros::ok()){
        tf::StampedTransform tf_dock;
        try{
            listener_dock.waitForTransform("base_link","docking_frame",ros::Time(0),ros::Duration(3.0));
            listener_dock.lookupTransform("base_link","docking_frame",ros::Time(0),tf_dock);
        }
        catch (tf::TransformException &ex){
            ROS_ERROR("%s",ex.what());
            ROS_ERROR("Did not find the pattern!");
            ros::Duration(1.0).sleep();
            continue;
        }

        // Dock_frame's origin and yaw
        float dock_x = tf_dock.getOrigin().x();
        float dock_y = tf_dock.getOrigin().y();
        float dock_yaw = tf::getYaw(tf_dock.getRotation());
        ros::spinOnce();
        setVel(dock_x, dock_y, dock_yaw);
        rate.sleep();
    }
    return 0;
}