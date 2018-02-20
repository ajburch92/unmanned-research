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
		current_time1 = ros::Time::now();
		last_time1 = ros::Time::now();
		current_time2 = ros::Time::now();
		last_time2 = ros::Time::now();
		current_time_out = ros::Time::now();

		ros::NodeHandle nh_odomp("~");
	    nh_odomp.param("ID_num",ID_num,0);
  
		stringstream ss;
		ss << ID_num;
		string s;
		s = ss.str();

		string vehicle_odom = "/vehicle_odom" + s ;

		vehicle_pose1_sub = nh_odom.subscribe("vehicle_pose1",2, &Odom::position1Callback, this);
		vehicle_pose2_sub = nh_odom.subscribe("vehicle_pose2",2, &Odom::position2Callback, this);
		odom_pub = nh_odom.advertise<nav_msgs::Odometry>(vehicle_odom, 2);
    	sub_corners1 = nh_odom.subscribe("corners1",1,&Odom::corners1Callback,this);
    	sub_corners2 = nh_odom.subscribe("corners2",1,&Odom::corners2Callback,this);
       	sub_conf1 = nh_odom.subscribe("confidence1",1,&Odom::confidence1Callback,this);
	   	sub_conf2 = nh_odom.subscribe("confidence2",1,&Odom::confidence2Callback,this);

		x1 = 0.0;
		y1 = 0.0;
		x_prev1 = 0.0;
		y_prev1 = 0.0;
		th_prev1 = 0.0;
		th1 = 0.0;
		vx1 = 0.0;
		vy1 = 0.0;
		th1 = 0.0;		
		x2 = 0.0;
		y2 = 0.0;
		x_prev2 = 0.0;
		y_prev2 = 0.0;
		th_prev2 = 0.0;
		th2 = 0.0;
		vx2 = 0.0;
		vy2 = 0.0;
		th2 = 0.0;
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
	void position1Callback (const geometry_msgs::Pose2D::ConstPtr& vehicle_pose1_msg) 
	{
		current_time1 = ros::Time::now();
		dt1 = (current_time1 - last_time1).toSec();

		// read msg data
		double xtemp = vehicle_pose1_msg->x;
		double ytemp = vehicle_pose1_msg->y;
		double th = vehicle_pose1_msg->theta;

	    Point2f vehicle_pose_temp;

	    int ID_num = 1;
	    // transform coordinates
	    if (ID_num > 1) { // ID = 2, HbirdHcamcamOG
	                float x = H_camcam_inv.at<double>(0,0) * xtemp + H_camcam_inv.at<double>(0,1) * ytemp + H_camcam_inv.at<double>(0,2);
	                float y = H_camcam_inv.at<double>(1,0) * xtemp + H_camcam_inv.at<double>(1,1) * ytemp + H_camcam_inv.at<double>(1,2);
	                float w = H_camcam_inv.at<double>(2,0) * xtemp + H_camcam_inv.at<double>(2,1) * ytemp + H_camcam_inv.at<double>(2,2);

	                vehicle_pose_temp=Point(x/w,y/w);


	    } else { // ID_num = 1 , HbirdOG

	                vehicle_pose_temp=Point(xtemp,ytemp);

	    }
	    ROS_INFO("vehicle_pose1: ( %f , %f )",xtemp,ytemp);


	    x1  = double(vehicle_pose_temp.x);
	    y1 = double(vehicle_pose_temp.y);
	    th1 = th;
		// calculate velocities
		vx1 = (x1-x_prev1)/dt1;
		vy1 = (y1-y_prev1)/dt1;
		vth1 = (th1-th_prev1)/dt1;

		last_time1 = current_time1;
		x_prev1 = x1;
		y_prev1 = y1;
		th_prev1 = th1;
	
		pose_selector () ;

	}

	void position2Callback (const geometry_msgs::Pose2D::ConstPtr& vehicle_pose2_msg) 
	{
		current_time2 = ros::Time::now();
		dt2 = (current_time2 - last_time2).toSec();

		// read msg data
		double x = vehicle_pose2_msg->x;
		double y = vehicle_pose2_msg->y;
		double th = vehicle_pose2_msg->theta;

	    Point2f vehicle_pose_temp;


	    int ID_num = 2;
	    // transform coordinates
	    if (ID_num > 1) { // ID = 2, HbirdHcamcamOG
	                float x = H_camcam_inv.at<double>(0,0) * x + H_camcam_inv.at<double>(0,1) * y + H_camcam_inv.at<double>(0,2);
	                float y = H_camcam_inv.at<double>(1,0) * x + H_camcam_inv.at<double>(1,1) * y + H_camcam_inv.at<double>(1,2);
	                float w = H_camcam_inv.at<double>(2,0) * x + H_camcam_inv.at<double>(2,1) * y + H_camcam_inv.at<double>(2,2);

	                vehicle_pose_temp=Point(x/w,y/w);


	    } else { // ID_num = 1 , HbirdOG

	                vehicle_pose_temp=Point(x,y);

	    }
	    ROS_INFO("vehicle_pose2: ( %f , %f )",x,y);


	    x2  = double(vehicle_pose_temp.x);
	    y2 = double(vehicle_pose_temp.y);
	    th2 = th;
		// calculate velocities
		vx2 = (x2-x_prev2)/dt2;
		vy2 = (y2-y_prev2)/dt2;
		vth2 = (th2-th_prev2)/dt2;

		last_time2 = current_time2;
		x_prev2 = x2;
		y_prev2 = y2;
		th_prev2 = th2;

		pose_selector () ;
	}

	void pose_selector () 
	{
		if (confidence1 || confidence2 > 0) {
	//		if confidence1 >
			switch (selector_state) 					
			{
				case 1:
					if (confidence1 >= confidence2) {

						x_out = x1;
						y_out = y1;
						th_out = th1;
						vx_out = vx1;
						vy_out = vy1;
						vth_out = vth1;
						current_time_out = current_time1;
						selector_state = 1;

					} else { //confidence2 > confidence1
						x_out = x2;
						y_out = y2;
						th_out = th2;
						vx_out = vx2;
						vy_out = vy2;
						vth_out = vth2;
						current_time_out = current_time2;
						selector_state = 2;

					} 					
					break;
				case 2:
					if (confidence1 > confidence2) {

						x_out = x1;
						y_out = y1;
						th_out = th1;
						vx_out = vx1;
						vy_out = vy1;
						vth_out = vth1;
						current_time_out = current_time1;
						selector_state = 1;

					} else { //confidence2 >= confidence1
						x_out = x2;
						y_out = y2;
						th_out = th2;
						vx_out = vx2;
						vy_out = vy2;
						vth_out = vth2;
						current_time_out = current_time2;
						selector_state = 2;

					} 
					break;
				default : //(0)
					if (confidence1 >= confidence2) {

						x_out = x1;
						y_out = y1;
						th_out = th1;
						vx_out = vx1;
						vy_out = vy1;
						vth_out = vth1;
						current_time_out = current_time1;	
						selector_state = 1;


					} else { //confidence2 > confidence1
						x_out = x2;
						y_out = y2;
						th_out = th2;
						vx_out = vx2;
						vy_out = vy2;
						vth_out = vth2;
						current_time_out = current_time2;
						selector_state = 2;

					} 
					break;
			}

			// PUBLISH ODOM

			geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th_out);

			geometry_msgs::TransformStamped odom_trans;
			odom_trans.header.stamp = current_time_out;
			odom_trans.header.frame_id = "odom";
			odom_trans.child_frame_id = "vehicle";

			odom_trans.transform.translation.x = x_out;
			odom_trans.transform.translation.y = y_out;
			odom_trans.transform.translation.z = 0.0;
			odom_trans.transform.rotation = odom_quat;

			//send the transform
			odom_broadcaster.sendTransform(odom_trans);

			//next, we'll publish the odometry message over ROS
			nav_msgs::Odometry odom;
			odom.header.stamp = current_time_out;
			odom.header.frame_id = "odom";

			//set the position
			odom.pose.pose.position.x = x_out;
			odom.pose.pose.position.y = y_out;
			odom.pose.pose.position.z = 0.0;
			odom.pose.pose.orientation = odom_quat;

			//set the velocity
			odom.child_frame_id = "vehicle";
			odom.twist.twist.linear.x = vx_out;
			odom.twist.twist.linear.y = vy_out;
			odom.twist.twist.angular.z = vth_out;

			//publish the message
			if (selector_state == ID_num) 
			{

				odom_pub.publish(odom);

			}

		} else { selector_state = 0 ; }
		

	}


private:
	ros::NodeHandle nh_odom;
	ros::Time current_time1, last_time1;
	ros::Time current_time2, last_time2;
	ros::Time current_time_out;
	ros::Publisher odom_pub;
	ros::Subscriber vehicle_pose1_sub;
	ros::Subscriber vehicle_pose2_sub;
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

	double x1;
	double y1;
	double x_prev1;
	double y_prev1;
	double th_prev1;
	double th1;
	double vx1;
	double vy1;
	double vth1;
	double dt1;
	
	double x2;
	double y2;
	double x_prev2;
	double y_prev2;
	double th_prev2;
	double th2;
	double vx2;
	double vy2;
	double vth2;
	double dt2;
	
	double confidence1;
	double confidence2;
	
	double x_out;
	double y_out;
	double th_out;
	double vx_out;
	double vy_out;
	double vth_out;

	int ID_num;
	int selector_state;
};

int main(int argc, char** argv){
	
	ros::init(argc, argv, "vehicle_odom");

	Odom o;

	ros::spin();

	return 0;

}
