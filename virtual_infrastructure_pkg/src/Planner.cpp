#include <ros/ros.h>
#include "nav_msgs/Odometry.h"
#include <std_msgs/Float64.h>
//#include "vehicle_pose.h"
//#include "goal_pose.h"
#include <geometry_msgs/Pose2D.h>
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <vector>
#include <numeric>

#define LOS_RADIUS 1

using namespace std;

ros::NodeHandle n;


ros::Publisher target_angle_pub;
ros::Publisher target_speed_pub;
ros::Subscriber sub_vehicle;
ros::Subscriber sub_goal;
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

double x_vehicle, y_vehicle, x_goal, y_goal;



void update_target_speed(double xe, double ye, double xv, double yv, double distance_to_target)
{
    distance_to_target = sqrt(pow(xe - xv, 2) + pow(ye - yv, 2));
    std_msgs::Float64 target_speed_msg;
    
    if (distance_to_target < 0.5) { // meter??
        target_speed = 0;
    } else {
      target_speed = 0.5;
    }

    target_speed_msg.data = target_speed;

    target_speed_pub.publish(target_speed_msg);
}

void update_target_angle(double xe, double ye, double xv, double yv, double angle_error)
{
    radius = LOS_RADIUS;
    if (pi_bool == 0) { // starting pos
        p_i[0] = xe;
        p_i[1] = ye;
        pi_bool =1;
    }
    // init
    p_e[0] = xe;
    p_e[1] = ye;
    p_v[0] = xv;
    p_v[1] = yv;
    // p_ie
    transform(p_e.begin(), p_e.end(), p_i.begin(), p_ie.begin(), minus<double>());
    // p_iv
    transform(p_v.begin(), p_v.end(), p_i.begin(), p_iv.begin(), minus<double>());

    // p_e
    transform(p_v.begin(), p_v.end(), p_e.begin(), p_ev.begin(), minus<double>());

    // e_CTE
    Mag_num = p_ev[0] * p_iv[1] - p_iv[1] * p_ev[0]; //2D cross prod?
    Mag_den = sqrt(pow(p_iv[0], 2.0) + pow(p_iv[1], 2.0));
    e_CTE = Mag_num / Mag_den;
    // Do A projection here before if statement, store p_a
    // implement if wrap for e_CTE<r_LOS
    if (e_CTE <= radius) {
        // p_b
        //projection, p_ia =  A || B = A•B * B/|B|^2
        p_ia[0] = inner_product(p_ie.begin(), p_ie.end(), p_iv.begin(), 0) * p_iv[0] / pow(Mag_den, 2.0);
        p_ia[1] = inner_product(p_ie.begin(), p_ie.end(), p_iv.begin(), 0) * p_iv[1] / pow(Mag_den, 2.0);
        p_b[0] = p_ia[0] + p_i[0] + p_iv[0] / Mag_den * sqrt(pow(radius, 2.0) + pow(e_CTE, 2.0));
        p_b[1] = p_ia[1] + p_i[1] + p_iv[1] / Mag_den * sqrt(pow(radius, 2.0) + pow(e_CTE, 2.0));
        
        // p_eb
        transform(p_b.begin(), p_b.end(), p_e.begin(), p_eb.begin(), minus<double>());
        
        // r_theta
        angle_error = atan2(p_eb[1], p_eb[0]) ; //in rads
    } else { // vehicle is further than LOS_radius from the path
        // compute p_ae = p_ia - p_ie
        transform(p_ia.begin(), p_ia.end(), p_ie.begin(), p_ae.begin(), minus<double>());
        
        angle_error= (atan2(p_ae[1], p_ae[0])); //in rads
        std_msgs::Float64 target_angle_msg;  
        target_angle_msg.data = target_angle;

        target_speed_pub.publish(target_angle_msg);

    }

}

void vehicleCallback (const geometry_msgs::Pose2D::ConstPtr& vehicle_pose_msg) 
{
    x_vehicle  = vehicle_pose_msg->x;
    y_vehicle = vehicle_pose_msg->y;
}

void goalCallback (const geometry_msgs::Pose2D::ConstPtr& goal_pose_msg) 
{
    x_goal  = goal_pose_msg->x;
    y_goal = goal_pose_msg ->y;
}



int main(int argc, char **argv)
{
  pi_bool = 0;  
  ros::init(argc, argv, "vehicle_planner");

  sub_vehicle = n.subscribe("/vehicle_pose",20, &vehicleCallback);
  sub_goal = n.subscribe("/goal_pose",20, &goalCallback);

  target_angle_pub = n.advertise<std_msgs::Float64>("/target_angle",2);
  target_speed_pub = n.advertise<std_msgs::Float64>("/target_speed",2);

  ros::spin();

  return 0;
}