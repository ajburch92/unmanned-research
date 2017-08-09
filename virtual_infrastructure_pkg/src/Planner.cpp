#include <ros/ros.h>
#include "nav_msgs/Odometry.h"
#include <std_msgs/Float64.h>
#include <virtual_infrastructure_pkg/vehicle_pose.h>
#include <virtual_infrastructure_pkg/goal_pose.h>
#include <stdio.h>
#include <iostream>
#include <math.h>

using namespace std;

class Planner {
     public:
        Planner();
        ros::NodeHandle n;
        void getLOSvector();
        void vehicleCallback(const virtual_infrastructure_pkg::vehicle_pose::ConstPtr& vehicle_pose);
        void goalCallback(const virtual_infrastructure_pkg::goal_pose::ConstPtr& goal_pose);
        void targetSpeed(double xe, double ye, double xv, double yv, double& distance_to_target);
        void targetAngle(double xe, double ye, double xv, double yv, double& angle_error);

      private:



        ros::Publisher target_angle_pub;
        ros::Publisher target_speed_pub;
        ros::Subscriber sub_vehicle;
        ros::Subscriber sub_goal;
        double x_vehicle, y_vehicle, x_goal, y_goal, target_angle, target_speed;

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
        
        bool pi_bool = 0;

        double xi; 
        double yi;
        
        double distance_to_target;
        double angle_error;

};




void Planner::target_speed(double xe, double ye, double xv, double yv, double& distance_to_target)
{
    distance_to_target = sqrt(pow(xe - xv, 2) + pow(ye - yv, 2));
    std_msgs::Float64 target_speed_msg
    
    if (distance_to_target < 0.5) { // meter??
        target_speed = 0;
    } else {
      target_speed = 0.5;
    }

    target_speed_msg.target_speed = target_speed;

    target_speed_pub.publish(target_speed_msg);
}

void Planner::target_angle(double xe, double ye, double xv, double yv, double& angle_error)
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
        std_msgs::Float64 target_angle_msg  
        target_angle_msg.target_angle; = target_angle;

        target_speed_pub.publish(target_angle_msg);

    }

}

void Planner::vehicleCallback (const virtual_infrastructure_pkg::vehicle_pose::ConstPtr& vehicle_pose) 
{
    x_vehicle  = vehicle_pose->x;
    y_vehicle = vehicle_pose->y;
}

void Planner::goalCallback (const virtual_infrastructure_pkg::goal_pose::ConstPtr& goal_pose) 
{
    x_goal  = vehicle_pose->x;
    y_goal = vehicle_pose->y;
}

Planner::Planner() 
{
  sub_vehicle = n.subscribe("/vehicle_pose",20, &Planner::vehicleCallback);
  sub_goal = n.subscribe("/goal_pose",20, &Planner::goalCallback);

  target_angle_pub = n.advertise<std_msgs::Float64>("/target_angle",20);
  target_speed_pub = n.advertise<std_msgs::Float64>("/target_speed",20);



  pi_bool = 0;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vehicle_planner");

  Planner p;

  ros::spin();

  return 0;
}