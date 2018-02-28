#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose2D.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
//#include "vehicle_pose.h"
//#include "goal_pose.h"
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>

// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv2/calib3d/calib3d.hpp>
// 
#include <sstream>
#include <string>
#include <iostream>
#include <vector>
#include <deque>
#include <math.h>
#include <stdio.h>

using namespace std; 
using namespace cv;

class Odom
{
public:
	Odom()
	{
		current_time = ros::Time::now();
		last_time = ros::Time::now();

		ros::NodeHandle nh_odomp("~");
	    nh_odomp.param("ID_num",ID_num,0);
  
		stringstream ss;
		ss << ID_num;
		string s;
		s = ss.str();
		string vehicle_pose_ss = "/vehicle_pose" + s ;
		string vehicle_odom_ss = "/vehicle_odom" + s ;
		string selector_state_ss = "/selector_state" + s ;

		vehicle_pose_sub = nh_odom.subscribe(vehicle_pose_ss,1, &Odom::positionCallback, this);
		odom_pub = nh_odom.advertise<nav_msgs::Odometry>(vehicle_odom_ss, 2);
		ss_pub = nh_odom.advertise<std_msgs::Int8>(selector_state_ss, 2);
    	sub_corners1 = nh_odom.subscribe("corners1",1,&Odom::corners1Callback,this);
    	sub_corners2 = nh_odom.subscribe("corners2",1,&Odom::corners2Callback,this);
       	sub_conf1 = nh_odom.subscribe("confidence1",1,&Odom::confidence1Callback,this);
	   	sub_conf2 = nh_odom.subscribe("confidence2",1,&Odom::confidence2Callback,this);

		x = 0.0;
		y = 0.0;
		x_prev = 0.0;
		y_prev = 0.0;
		th_prev = 0.0;
		th = 0.0;
		vx = 0.0;
		vy = 0.0;
		th = 0.0;		

		confidence1 = 0.0;
		confidence2 = 0.0;
		selector_state = 0;
	}

	void getPerspectives() {
	    //corner_tic++;
	    //if (corner_tic >= 2) { //only after both messages are received. 
	      //  corner_tic = 0;

	        // calc camcam H mat
	        H_camcam = getPerspectiveTransform(corners1_pts,corners2_pts);
			H_camcam_inv = H_camcam.inv(DECOMP_SVD);	
   			ROS_INFO("camcam");

	        cout << H_camcam << endl;
	    //}
	        
	    // calc cambird H mat

	    undistorted_pts[0].x=corners1_pts[0].x;
	    undistorted_pts[1].x=corners1_pts[0].x+board_w-1;
	    undistorted_pts[2].x=corners1_pts[0].x;
	    undistorted_pts[3].x=corners1_pts[0].x+board_w-1;
	    undistorted_pts[0].y=corners1_pts[0].y;
	    undistorted_pts[1].y=corners1_pts[0].y;
	    undistorted_pts[2].y=corners1_pts[0].y+board_h-1;
	    undistorted_pts[3].y=corners1_pts[0].y+board_h-1;



  //   	undistorted_pts[0].x=0;
		// undistorted_pts[1].x=board_w-1;
		// undistorted_pts[2].x=0;
		// undistorted_pts[3].x=board_w-1;
		// undistorted_pts[0].y=0;
		// undistorted_pts[1].y=0;
		// undistorted_pts[2].y=board_h-1;
		// undistorted_pts[3].y=board_h-1;

	    //H_cambird = getPerspectiveTransform(corners1_pts, undistorted_pts);
	    //H_cambird = getPerspectiveTransform(undistorted_pts, corners1_pts);
		//H_cambird.at<double>(2,2) = 10;

		//ROS_INFO("cambird");

	    //cout << H_cambird << endl;

	}



	void confidence1Callback (const std_msgs::Float64::ConstPtr& confidence1_msg) 
	{
		confidence1 = confidence1_msg -> data;
	    ROS_INFO("confidence1: ( %f )",confidence1);
	}


		void confidence2Callback (const std_msgs::Float64::ConstPtr& confidence2_msg) 
	{
		confidence2 = confidence2_msg -> data;
	    ROS_INFO("confidence2: ( %f )",confidence2);
	}


	void corners1Callback (const geometry_msgs::PoseArray::ConstPtr& corners1_msg) 
	{
	    corners1_vec.clear();
	    int size = corners1_msg->poses.size();
	    for (int i=0;i<size;i++)
	    {
	        corners1_vec.push_back(Point((int)corners1_msg->poses[i].position.x,(int)corners1_msg->poses[i].position.y));
	        corners1_pts[i] = corners1_vec[i];    
	    }

	    getPerspectives();
	}



	void corners2Callback (const geometry_msgs::PoseArray::ConstPtr& corners2_msg) 
	{		

	    
	    corners2_vec.clear();
	    int size = corners2_msg->poses.size();
	    for (int i=0;i<size;i++)
	    {
	        corners2_vec.push_back(Point((int)corners2_msg->poses[i].position.x,(int)corners2_msg->poses[i].position.y));
	        corners2_pts[i] = corners2_vec[i];    
	    }

		ROS_INFO("corners2");
        cout << corners2_vec << endl;

	    getPerspectives();



		for (int n=0;n<=3; n++) {

	        float x = H_camcam_inv.at<double>(0,0) * corners2_vec[n].x + H_camcam_inv.at<double>(0,1) * corners2_vec[n].y + H_camcam_inv.at<double>(0,2);
	        float y = H_camcam_inv.at<double>(1,0) * corners2_vec[n].x + H_camcam_inv.at<double>(1,1) * corners2_vec[n].y + H_camcam_inv.at<double>(1,2);
	        float w = H_camcam_inv.at<double>(2,0) * corners2_vec[n].x + H_camcam_inv.at<double>(2,1) * corners2_vec[n].y + H_camcam_inv.at<double>(2,2);

	        corners2_pts_camcam[n]=Point(x/w,y/w);
		
		}

	}
	void positionCallback (const geometry_msgs::Pose2D::ConstPtr& vehicle_pose_msg) 
	{
		current_time = ros::Time::now();
		dt = (current_time - last_time).toSec();

		// read msg data
		double xtemp = vehicle_pose_msg->x + 1280;
		double ytemp = vehicle_pose_msg->y + 960;
		double th = vehicle_pose_msg->theta;

	    Point2f vehicle_pose_temp;

	    vehicle_pose_temp=Point(xtemp,ytemp);
	
	    ROS_INFO("vehicle_pose1: ( %f , %f )",xtemp,ytemp);

	    x  = double(vehicle_pose_temp.x);
	    y = double(vehicle_pose_temp.y);
	    th = th;
		// calculate velocities
		vx = (x-x_prev)/dt;
		vy = (y-y_prev)/dt;
		vth = (th-th_prev)/dt;

		last_time = current_time;
		x_prev = x;
		y_prev = y;
		th_prev = th;
	
		pose_selector () ;

	}

	void pose_selector () 
	{
		if (confidence1 || confidence2 > 0) {
			switch (selector_state) 					
			{
				case 1:// last selection was GS1
					if (confidence1 >= confidence2) {
						selector_state = 1;
					} else { //confidence2 > confidence1
						// make sure confidences arent saturated, no need to switch
						// only switch when current confidence drops below 1, and is less than confidence 2
						if (confidence1 >= 1.00) {
							selector_state = 1; // keep 1 
						} else {selector_state = 2;}
					} 					
					break;
				case 2: // last selection was GS2
					if (confidence1 > confidence2) {
						selector_state = 1;
					} else { //confidence2 >= confidence1
						if (confidence2 >= 1.00) {
							selector_state = 2; // keep 2 selected 
						} else {selector_state = 1;}
					} 
					break;
				default : //(0) give GS1 priority
					if (confidence1 >= confidence2) {
						selector_state = 1;
					} else { //confidence2 > confidence1
						selector_state = 2;
					} 
					break;
			}

			// PUBLISH ODOM

			geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

			geometry_msgs::TransformStamped odom_trans;
			odom_trans.header.stamp = current_time;
			odom_trans.header.frame_id = "odom";
			odom_trans.child_frame_id = "vehicle";

			odom_trans.transform.translation.x = x;
			odom_trans.transform.translation.y = y;
			odom_trans.transform.translation.z = 0.0;
			odom_trans.transform.rotation = odom_quat;

			//send the transform
			odom_broadcaster.sendTransform(odom_trans);

			//next, we'll publish the odometry message over ROS
			nav_msgs::Odometry odom;
			odom.header.stamp = current_time;
			odom.header.frame_id = "odom";

			//set the position
			odom.pose.pose.position.x = x;
			odom.pose.pose.position.y = y;
			odom.pose.pose.position.z = 0.0;
			odom.pose.pose.orientation = odom_quat;

			//set the velocity
			odom.child_frame_id = "vehicle";
			odom.twist.twist.linear.x = vx;
			odom.twist.twist.linear.y = vy;
			odom.twist.twist.angular.z = vth;

			//publish the message
			if (selector_state == ID_num) 
			{

				odom_pub.publish(odom);

			}

		} else { selector_state = 0 ; }
		
		std_msgs::Int8 selector_state_msg;
		selector_state_msg.data = selector_state;
		ss_pub.publish(selector_state_msg);

	}


private:
	ros::NodeHandle nh_odom;
	ros::Time current_time, last_time;
	ros::Publisher odom_pub;
	ros::Publisher ss_pub;
	ros::Subscriber vehicle_pose_sub;
	tf::TransformBroadcaster odom_broadcaster;
	ros::Subscriber sub_corners2;
	ros::Subscriber sub_corners1;
	ros::Subscriber sub_conf1;
	ros::Subscriber sub_conf2;

	vector<Point2f> corners2_vec;
	vector<Point2f> corners1_vec;
	Point2f undistorted_pts[4];
	Point2f corners1_pts[4];
	Point2f corners2_pts[4];
	Point2f corners2_pts_camcam[4];

	int corner_tic = 0;
	int board_w = 80;
	int board_h = 60;
	Mat H_camcam;
	Mat H_cambird;
	Mat H_camcam_inv;

	double x;
	double y;
	double x_prev;
	double y_prev;
	double th_prev;
	double th;
	double vx;
	double vy;
	double vth;
	double dt;
	
	double confidence1;
	double confidence2;
	
	int ID_num;
	int selector_state;
};

int main(int argc, char** argv){
	
	ros::init(argc, argv, "vehicle_odom");

	Odom o;

	ros::spin();

	return 0;

}
