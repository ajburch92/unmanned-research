#include <ros/ros.h>
#include "nav_msgs/Odometry.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
//#include "vehicle_pose.h"
//#include "wp_pose.h"
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>

// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <stdio.h>
#include <iostream>
#include <math.h>
#include <vector>
#include <numeric>

#define LOS_RADIUS 100 //currently in pixels

using namespace std;
using namespace cv;


ros::Publisher target_angle_pub;
ros::Publisher target_speed_pub;
ros::Publisher target_wp_pub;
ros::Subscriber sub_vehicle;
ros::Subscriber sub_wp;
ros::Subscriber sub_conv_fac;
ros::Subscriber sub_arm_bool;

double radius;
vector <double> p_i;
vector <double> p_e;
vector <double> p_v;
vector <double> p_ie;
vector <double> p_iv;
vector <double> p_ev;
double e_CTE;
double Mag_num;
double Mag_den;
vector <double> p_ia;
vector <double> p_b;
vector <double> p_eb;
vector <double> p_ae;

bool pi_bool;

double xi; 
double yi;

double distance_to_target, target_speed;
double angle_error, target_angle;

double x_vehicle, y_vehicle, x_wp, y_wp;

vector<Point> vector_wp;

double conv_fac;
double ID_num;

int arm_bool;


void update_target_speed()
{
 if (arm_bool > 0) 
 {

      distance_to_target = sqrt(pow(x_vehicle - x_wp, 2) + pow(y_vehicle - y_wp, 2));
      std_msgs::Float64 target_speed_msg;
      
      if (distance_to_target < 100) { // meter?? NEED TO CONVERT FROM PIXELS TO METERS
          target_speed = 0.8*target_speed*(1 - (1/distance_to_target));
      } else if (distance_to_target < 40) {
          target_speed = 0;
      } else {
        target_speed = 50.0*conv_fac; // pixels to meter/second    
      }

      ROS_INFO("distance_to_target: %f , target_speed: %f",  distance_to_target, target_speed);

      target_speed_msg.data = target_speed;
      target_speed_pub.publish(target_speed_msg);

    } else {

      std_msgs::Float64 target_speed_msg;
      target_speed = 0.0;
      target_speed_msg.data = target_speed;
      ROS_INFO("target_speed: %f", target_speed_msg.data);
      target_speed_pub.publish(target_speed_msg);

    }
}

void update_target_angle()
{
    radius = LOS_RADIUS;
    if (pi_bool == 0) { // starting pos
        p_i[0] = x_vehicle;
        p_i[1] = y_vehicle;
        pi_bool =1;
        ROS_INFO("p_i = %f,%f",p_i[0],p_i[1]);
    }
    // init
    p_e[0] = x_vehicle;
    p_e[1] = y_vehicle;
    p_v[0] = x_wp;
    p_v[1] = y_wp;
    // p_ie
    transform(p_e.begin(), p_e.end(), p_i.begin(), p_ie.begin(), minus<double>());
    // p_iv
    transform(p_v.begin(), p_v.end(), p_i.begin(), p_iv.begin(), minus<double>());
    // p_e
    transform(p_v.begin(), p_v.end(), p_e.begin(), p_ev.begin(), minus<double>());

    // e_CTE
    Mag_num = p_ev[0] * p_iv[1] - p_ev[1] * p_iv[0]; //2D cross prod?
    Mag_den = sqrt(pow(p_iv[0], 2.0) + pow(p_iv[1], 2.0));
    e_CTE = Mag_num / Mag_den;
    // Do A projection here before if statement, store p_a
    // implement if wrap for e_CTE<r_LOS

    // p_b
    //projection, p_ia =  A || B = A•B * B/|B|^2
    p_ia[0] = inner_product(p_ie.begin(), p_ie.end(), p_iv.begin(), 0) * p_iv[0] / pow(Mag_den, 2.0);
    p_ia[1] = inner_product(p_ie.begin(), p_ie.end(), p_iv.begin(), 0) * p_iv[1] / pow(Mag_den, 2.0);
    p_b[0] = p_ia[0] + p_i[0] + p_iv[0] / Mag_den * sqrt(pow(radius, 2.0) + pow(e_CTE, 2.0));
    p_b[1] = p_ia[1] + p_i[1] + p_iv[1] / Mag_den * sqrt(pow(radius, 2.0) + pow(e_CTE, 2.0));
    
    // p_eb
    transform(p_b.begin(), p_b.end(), p_e.begin(), p_eb.begin(), minus<double>());
    
    // compute p_ae = p_ia - p_ie
    transform(p_ia.begin(), p_ia.end(), p_ie.begin(), p_ae.begin(), minus<double>());
    if (e_CTE <= radius) {
        // r_theta
        angle_error = atan2(p_eb[1], p_eb[0]) ; //in rads

    } else { // vehicle is further than LOS_radius from the path

        angle_error= (atan2(p_ae[1], p_ae[0])); //in rads
    }
    target_angle = angle_error;        
    std_msgs::Float64 target_angle_msg;  
    target_angle_msg.data = target_angle;
    ROS_INFO("target_angle: %f", target_angle_msg.data);
    target_angle_pub.publish(target_angle_msg);


}

void armCallback (const std_msgs::Float64::ConstPtr& arm_bool_msg) 
{
    arm_bool = arm_bool_msg -> data;
    ROS_INFO("arm_bool = %i" , arm_bool);

    update_target_speed();

}

void convFacCallback (const std_msgs::Float64::ConstPtr& conv_fac_msg) 
{
    conv_fac = conv_fac_msg -> data;
    ROS_INFO("conv_fac = %f" , conv_fac);
}
   
void vehicleCallback (const nav_msgs::Odometry::ConstPtr& vehicle_odom_msg) 
{
    x_vehicle  = vehicle_odom_msg->pose.pose.position.x;
    y_vehicle = vehicle_odom_msg->pose.pose.position.y;
    p_i[0] = x_vehicle;
    p_i[1] = y_vehicle;
    ROS_INFO("vehicleCallback: ( %f , %f )",x_vehicle,y_vehicle);
    int i = 0;
    double euclidean_d=0;
    int sz = vector_wp.size();

    update_target_angle();

}

void waypointCallback (const geometry_msgs::PoseArray::ConstPtr& waypoint_pose_msg) 
{

  int i = 0;
  double euclidean_d=0;
  int sz = waypoint_pose_msg->poses.size();
  if (sz > 0) 
  {
    while ((euclidean_d <= LOS_RADIUS) && (i<(sz-1)))
    {
      euclidean_d = sqrt(((x_vehicle-waypoint_pose_msg->poses[i].position.x)*(x_vehicle-waypoint_pose_msg->poses[i].position.x)) + ((y_vehicle-waypoint_pose_msg->poses[i].position.y)*(y_vehicle-waypoint_pose_msg->poses[i].position.y)));
      if (euclidean_d > LOS_RADIUS)
      {
        break; // waypoint found
      } else // distance < los radius
      {
        if (i==(sz-1)) // last waypoint, stop
        {
          break;
        } else { // more points, go to next waypoint
          i++;
        }
      }
    }

      //ROS_INFO("waypoint( %f , %f ), waypointDistance: %f",waypoint_pose_msg->poses[i].position.x,waypoint_pose_msg->poses[i].position.y,euclidean_d);
      //vector_wp[i] = Point(waypoint_pose_msg->poses[i].position.x, waypoint_pose_msg->poses[i].position.y);

    x_wp  = waypoint_pose_msg->poses[i].position.x;
    y_wp = waypoint_pose_msg->poses[i].position.y;

    ROS_INFO("waypointTarget: ( %f , %f ), waypointDistance: %f",x_wp,y_wp,euclidean_d);
    
    geometry_msgs::Pose2D target_wp_msg;
    target_wp_msg.x = x_wp;
    target_wp_msg.y = y_wp;
    target_wp_pub.publish(target_wp_msg); 
  } // else set target speed to zero? wait for clear path?

  // update vehicle planner with starting and end points between subgoals/waypoint goals.

}



int main(int argc, char **argv)
{
  arm_bool = 0;
  conv_fac = 1.0;
  target_speed = 0.1;
  pi_bool = 0;  
  x_wp = 0;
  y_wp = 0;
  x_vehicle = 0;
  y_vehicle = 0;
  p_e.resize(2);
  p_i.resize(2);
  p_v.resize(2);
  p_ie.resize(2);
  p_iv.resize(2);
  p_ev.resize(2);
  p_ia.resize(2);
  p_b.resize(2);
  p_eb.resize(2);
  p_ae.resize(2);

  ros::init(argc, argv, "vehicle_planner");
  ros::NodeHandle n;

  //n.param("ID_num",ID_num,-1);
  ID_num = 1;
  
  stringstream ss;
  ss << ID_num;
  string s;
  s = ss.str();

  string conv_fac = "/conv_fac" + s ;
  string vehicle_odom = "/vehicle_odom" + s ;
  string wp_pose = "/wp_pose" + s ;
  string arm_bool = "/arm_bool" + s ;
  string target_angle = "/target_angle" + s ;
  string target_speed = "/target_speed" + s ;
  string target_wp = "/target_wp" + s ;

  sub_vehicle = n.subscribe(vehicle_odom,20, &vehicleCallback);
  sub_wp = n.subscribe(wp_pose,20, &waypointCallback);
  sub_conv_fac = n.subscribe(conv_fac,20, &convFacCallback);
  sub_arm_bool = n.subscribe(arm_bool,20, &armCallback);

  target_angle_pub = n.advertise<std_msgs::Float64>(target_angle,2);
  target_speed_pub = n.advertise<std_msgs::Float64>(target_speed,2);
  target_wp_pub = n.advertise<geometry_msgs::Pose2D>(target_wp,2);

  
  ros::spin();

  return 0;
}