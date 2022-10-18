#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2/utils.h>

#define BACKDIST 4

float Kp_x = 0.5;
float Kp_y = 0.5;
float Kp_yaw = 1.0;

double wait_ctime = 0;
double current_time, last_time;

double straight_time;
double straight_speed = 0.15;

int main(int argc, char** argv){
  ros::init(argc, argv, "PID_controller");

  ros::NodeHandle node;

  ros::Publisher fourMacenum_Vel = node.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  bool isBotSet = false;

  ros::Rate rate(10.0);
  while (node.ok()){

    current_time = ros::Time::now().toSec();
    double dt=current_time-last_time;
    last_time = ros::Time::now().toSec();

    geometry_msgs::TransformStamped transformStamped;
    try{
      transformStamped = tfBuffer.lookupTransform("base_footprint", "goal", ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    float yaw = tf2::getYaw(transformStamped.transform.rotation);
    float x = transformStamped.transform.translation.x;
    float y = transformStamped.transform.translation.y;
    float locDist;
    float glbDist = sqrt(pow(x, 2) + pow(y, 2));
    geometry_msgs::Twist velMsg;


   if (glbDist<0.2){
     straight_time=0.2/straight_speed;
     wait_ctime+=dt;
     if(wait_ctime<straight_time){
      velMsg.linear.x = straight_speed;
      velMsg.linear.y = 0;
      velMsg.angular.z = 0;
    std::cout<<"moving forward  "<<"linear.x: "<<velMsg.linear.x<<"linear.y: "<<velMsg.linear.y<<"linear.yaw: "<<velMsg.angular.z<<std::endl;
      }else{
         velMsg.linear.x = 0;
         velMsg.linear.y = 0;
         velMsg.angular.z = 0;
     ROS_INFO("Robot reached at goal");
      }

   }
    else{
      velMsg.linear.x = Kp_x * x;
      velMsg.linear.y = Kp_y * y;
      velMsg.angular.z = Kp_yaw * yaw;  
    std::cout<<"PID control    "<<"linear.x: "<<velMsg.linear.x<<"linear.y: "<<velMsg.linear.y<<"linear.yaw: "<<velMsg.angular.z<<std::endl;
    }

    fourMacenum_Vel.publish(velMsg);
    rate.sleep();
  }
  return 0;
}
