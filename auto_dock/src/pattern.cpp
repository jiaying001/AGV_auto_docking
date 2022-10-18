#include <ros/ros.h>
#include <laser_line_extraction/LineSegmentList.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Bool.h>
#include "comm.h"

#include <vector>
#include <cmath>
#define INFI 1000
#define _USE_MATH_DEFINES

// ROS
visualization_msgs::Marker points_msg;
//std_msgs::Bool lr_msg;

// Parameters
double laser_base_offset_x,laser_base_offset_y;
double dock_detection_range;
double pattern_angle, pattern_width, pattern_height,base2end_dist;
double detect_angle_tolerance,detect_line_tolerance, group_dist_tolerance,ac_parallel_tolerance,pose_error_tolerance;
std::string laser_frame_id;


int v_a, v_b, v_c;
int find_times=0;
int error_times=0;
bool find_once = false;
double a_detect_error=100;
double b_detect_error2=100;
double c_detect_error=100;

double x_center, y_center, z_center;

double x,y,yaw;
double last_x,last_y,last_yaw;
double last_x_temp,last_y_temp,last_yaw_temp;
bool first_find = true;
bool first_compare = true;


void populateMarkerMsg(double x, double y){
    points_msg.id = 1;
    points_msg.ns = "points";
    points_msg.type = visualization_msgs::Marker::POINTS;
    points_msg.action = visualization_msgs::Marker::ADD;
    points_msg.scale.x = 0.03;
    points_msg.scale.y = 0.03;
    points_msg.color.r = 0.0;
    points_msg.color.g = 0.0;
    points_msg.color.b = 1.0;
    points_msg.color.a = 1.0;
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = 0;
    points_msg.points.push_back(p);
    points_msg.header.frame_id = laser_frame_id;
    points_msg.header.stamp = ros::Time::now();
}


void populateTF(double x, double y, double theta){
    // publish dock_frame
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x,y,0));
    tf::Quaternion q;
    q.setRPY(0,0,theta);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"base_link","docking_frame"));
  
}


  void sendTransform(double x, double y, double yaw){
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "odom";
    transformStamped.child_frame_id = "goal";
    transformStamped.transform.translation.x = x;
    transformStamped.transform.translation.y = y;
    transformStamped.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    br.sendTransform(transformStamped);
  }


void patternCallback(const laser_line_extraction::LineSegmentList::ConstPtr& msg){
    std::vector<laser_line_extraction::LineSegment_<std::allocator<void>>> vectors = msg->line_segments;

    // Number of the line
    int lineNum = vectors.size();
    
    bool check_vec_size = true;

    //fprintf(stderr, "Number of line = %d\n", lineNum);

    // Check whether topic line_segments is publishing
    if (lineNum <= 3){
        ROS_ERROR("There isn't enough line in the laser field!");
        check_vec_size = false;
    }


    if (check_vec_size){
     //   ROS_INFO("Searching Pattern......");
   //search line a and c = pattern_height
        for(int i=0; i<lineNum; i++){
             double dist_a = sqrt(pow(fabs(vectors[i].start[0] - vectors[i].end[0]),2)+pow(fabs(vectors[i].start[1] - vectors[i].end[1]),2));
             a_detect_error = fabs(dist_a-pattern_height);
              if((a_detect_error < detect_line_tolerance)&& (vectors[i].radius<dock_detection_range)){
                 
                 std::cout<<"dist_a: "<<dist_a<<"a.radius"<<vectors[i].radius<<std::endl;
                   for (int j=i+1; j<lineNum; j++){
                        double dist_c = sqrt(pow(fabs(vectors[j].start[0] - vectors[j].end[0]),2)+pow(fabs(vectors[j].start[1] - vectors[j].end[1]),2));
                         c_detect_error = fabs(dist_c-pattern_height);
                        if(c_detect_error < detect_line_tolerance ){
                           
                           std::cout<<"dist_c: "<<dist_c<<std::endl;
                          //check the a and c parallel ?
                              double angle_ac = fabs (vectors[i].angle - vectors[j].angle);
                              double angle_ac2 = fabs(angle_ac - M_PI );

                              if( (angle_ac < ac_parallel_tolerance) || (angle_ac2 < ac_parallel_tolerance)){
                                //std::cout<<"find line a and c: v_a: "<<i<<" v_c: "<<j<<std::endl;

                                    // find line b index 
                                     for (int k=i+1; k < j; k++){
                                        double dist_b2 = sqrt(pow(fabs(vectors[k].start[0] - vectors[k].end[0]),2)+pow(fabs(vectors[k].start[1] - vectors[k].end[1]),2));
                                         b_detect_error2 = fabs(dist_b2-pattern_width);
                                        std::cout<<"dist_b: "<<dist_b2<<std::endl;
                                       if(b_detect_error2 < detect_line_tolerance ){
                                           //std::cout<<"find line b"<<std::endl;
                                             std::cout<<"dist_a: "<<dist_a<<"  dist_b: "<<dist_b2<<"  dist_c: "<<dist_c<<std::endl;
                                             v_a=i;
                                             v_c=j;
                                             v_b=k;
                                             double b_mx = (vectors[v_b].start[0]+vectors[v_b].end[0])/2;
                                             double b_my = (vectors[v_b].start[1]+vectors[v_b].end[1])/2;
                                             yaw = vectors[v_b].angle;
                                             std::cout<<"x: "<<x<<" y: "<<y<<" yaw: "<<yaw<<std::endl;
                                             x=b_mx-base2end_dist*cos(yaw);
                                             y=b_my-base2end_dist*sin(yaw);
                                         /*
                                            if(first_find){
                                               last_x=x;
                                               last_y=y;
                                               last_yaw=yaw;
                                               first_find=false;
                                             }

                                            if((fabs(x-last_x)+fabs(y-last_y)+fabs(yaw-last_yaw))>pose_error_tolerance){
                                              if(first_compare){
                                               last_x_temp=x;
                                               last_y_temp=y;
                                               last_yaw_temp=yaw;
                                               first_compare=false;
                                               }
                                               
                                               if((fabs(x-last_x_temp)+fabs(y-last_y_temp)+fabs(yaw-last_yaw_temp))<pose_error_tolerance){
                                                  error_times++;
                                                }else{
                                                  error_times=0;
                                                 // first_compare=true;
                                                   last_x_temp=x;
                                                   last_y_temp=y;
                                                   last_yaw_temp=yaw;
                                                }

                                                 if(error_times>7){
                                                       error_times=0;
                                                       last_x=x;
		                                                   last_y=y;
		                                                   last_yaw=yaw;
                                                       ROS_INFO("update pose......");
                                                      }
                                             }
                                         */
                                              x_center=x;//last_x;
                                              y_center=y;//last_y;
                                              z_center=yaw;//last_yaw;
                                             populateTF(x,y,yaw);
                                             find_once = true;
                                             ROS_INFO("Find the Pattern!");
					}else{
						find_once = false;
                                         }
                                     }// for (int k=i+1; k < j; k++)
                      
                              } //end if( (angle_ac < ac_parallel_tolerance) || (angle_ac2 < ac_parallel_tolerance)){
                              
                       }// end if(c_detect_error < detect_line_tolerance )
                   }// end for (int j=i+1; j<lineNum; j++)
                }//end if(a_detect_error < detect_line_tolerance )

        }// end  for(int i=0; i<lineNum; i++)

    } //end if (check_vec_size)

    
}



typedef struct
{
   //center point
    double x_c;
    double y_c;
    double z_c;
    double a_c;
    bool tf_c_rev_flag;
} rbtctrl;

rbtctrl rc;


int main(int argc, char** argv){
    ros::init(argc, argv, "pattern_node");
    ros::NodeHandle n;

    // Load Parameters
    n.getParam("/pattern_node/laser_base_offset_x",laser_base_offset_x);
    n.getParam("/pattern_node/laser_base_offset_y",laser_base_offset_y);
     n.getParam("/pattern_node/dock_detection_range",dock_detection_range);
    n.getParam("/pattern_node/pattern_angle",pattern_angle);
    n.getParam("/pattern_node/pattern_width",pattern_width);
    n.getParam("/pattern_node/pattern_height",pattern_height);
    n.getParam("/pattern_node/base2end_dist",base2end_dist);
    n.getParam("/pattern_node/pattern_height",ac_parallel_tolerance);
    n.getParam("/pattern_node/detect_angle_tolerance",detect_angle_tolerance);
    n.getParam("/pattern_node/detect_line_tolerance",detect_line_tolerance);
    n.getParam("/pattern_node/group_dist_tolerance",group_dist_tolerance);
    n.getParam("/pattern_node/pose_error_tolerance",pose_error_tolerance);
    n.getParam("/pattern_node/laser_frame_id",laser_frame_id);


    
    ros::Subscriber line_sub_ = n.subscribe("line_segments", 10, patternCallback);
    ros::Publisher marker_pub_ =n.advertise<visualization_msgs::Marker>("origin_markers",1);
    ros::Publisher goal_pub_= n.advertise<geometry_msgs::PoseStamped>("/detected_docking_goal", 1);

   // ros::Publisher lr_pub_ =nh_.advertise<std_msgs::Bool>("l_or_r",100);
   
    GetTFPose tf(0.1, 20);
      
    ros::Rate rate(20.0);

    while(ros::ok()){
        tf.GetPose("/odom", "/base_link", &rc.x_c, &rc.y_c, &rc.z_c, &rc.a_c, &rc.tf_c_rev_flag);
        //std::cout<<"pose_x: "<<rc.x_c<<"pose_y: "<<rc.y_c<<"pose_a: "<<rc.a_c<<std::endl;
     if(find_once){
        //from lidar2baselink
        double x_center1=x_center+laser_base_offset_x;
        double y_center1=y_center+laser_base_offset_y;
        double z_center1;
        //from baselink2odom or from baselink2map
        double x_center1_temp=x_center1*cos(rc.a_c)-y_center*sin(rc.a_c);
        double y_center1_temp=x_center1*sin(rc.a_c)+y_center*cos(rc.a_c);  
        x_center1=x_center1_temp;
        y_center1=y_center1_temp; 
        z_center1= z_center + rc.a_c;
        //z_center1= 
       //Offset Poitition (from base_link to map) 
        x_center1+=rc.x_c;
        y_center1+=rc.y_c;
       
        sendTransform(x_center1,y_center1,z_center1);   
       
       geometry_msgs::PoseStamped goal_msg;
	     goal_msg.header.frame_id = "odom";
	     goal_msg.header.stamp = ros::Time::now();
	     goal_msg.pose.position.x = x_center1;
	     goal_msg.pose.position.y = y_center1;
	     goal_msg.pose.orientation = tf::createQuaternionMsgFromYaw(z_center1);
	     goal_pub_.publish(goal_msg);
        
       }
        marker_pub_.publish(points_msg);
      //  lr_pub_.publish(lr_msg);
       // vis_pub_.publish(marker(x_center,y_center,z_center));
     


        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}
