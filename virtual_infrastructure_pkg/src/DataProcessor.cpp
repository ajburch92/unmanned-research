

//
// DataProcessor.cpp
// Austin Burch
// 
//

// ros
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>

// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/xfeatures2d.hpp>
//#include <opencv2/stitching.hpp>
// 
#include <sstream>
#include <string>
#include <iostream>
#include <vector>
#include <deque>
#include <math.h>
#include <stdio.h>
#include <algorithm>
//msg file headers

#define LOS_RADIUS 15 //currently in pixels
#define HEADING_LINE_THICKNESS 1
#define VEHICLE_POSE_HISTORY_SIZE 20

using namespace std;
using namespace cv;
const int scale_factor = 8; // this needs to be the same as in the rgb node. change to launch file parameter. 
// this may not work for all resolutions. this value needs to match the value in astar. retrive from param launch file.

const string windowName = "Visualizer";

vector<Point> goal_points;
Point target_wp;
Point target_wp2;
vector<Point> vector_wp;
vector<Point> vector_wp2;

void onMouse( int evt, int x, int y, int flags, void *param) 
{
	if (evt == CV_EVENT_LBUTTONDOWN) 
	{
		vector<Point>* ptPtr = (vector<Point>*)param;
		ptPtr->push_back(Point(x/2,y/2));
		ROS_INFO("left click registered");


	} else if (evt == CV_EVENT_RBUTTONDOWN)
	{
		goal_points.clear();
    	//vector_wp.clear();
    	//target_wp = Point(0,0);

		ROS_INFO("right click registered");

	}
}

//Data Processor Class ////////////////////////////

class DataProcessor
{
public:

	DataProcessor()
	{

	ros::NodeHandle nh;
	image_transport::ImageTransport it_nh(nh);
	pub_worldMap = it_nh.advertise("/worldMap",1);
	pub_goal_out = nh.advertise<geometry_msgs::PoseArray>("/goal_pose_out",1);
	key_cmd_pub = nh.advertise<std_msgs::Int8>("/key_cmd",1);
	pub_vis1 = it_nh.advertise("/vis1",1);
	pub_vis2 = it_nh.advertise("/vis2",1);

    sub_corners1 = nh.subscribe("corners1",1,&DataProcessor::corners1Callback,this);
    sub_corners2 = nh.subscribe("corners2",1,&DataProcessor::corners2Callback,this);
	sub_rgb2 = it_nh.subscribe("/ground_station_rgb2",1, &DataProcessor::rgbFeed2Callback, this); 
	sub_rgb1 = it_nh.subscribe("/ground_station_rgb1",1, &DataProcessor::rgbFeed1Callback, this); 
	sub_target_wp1 = nh.subscribe("/target_wp1",1, &DataProcessor::targetwp1Callback,this);
	sub_vector_wp1 = nh.subscribe("/wp_pose1",1, &DataProcessor::vectorwp1Callback,this);
	sub_target_angle1 = nh.subscribe("/target_angle1",1, &DataProcessor::targetAngle1Callback,this);
	sub_conv_fac1 = nh.subscribe("/conv_fac1",1, &DataProcessor::convFac1Callback,this);
	sub_vehicle1 = nh.subscribe("/vehicle_pose1",1, &DataProcessor::vehicle1Callback, this);
	sub_vehicle2 = nh.subscribe("/vehicle_pose2",1, &DataProcessor::vehicle2Callback, this);
	sub_target_angle2 = nh.subscribe("/target_angle2",1, &DataProcessor::targetAngle2Callback,this);
	sub_vector_wp2 = nh.subscribe("/wp_pose2",1, &DataProcessor::vectorwp2Callback,this);
	sub_target_wp2 = nh.subscribe("/target_wp2",1, &DataProcessor::targetwp2Callback,this);
	sub_conf1 = nh.subscribe("/confidence1",1,&DataProcessor::confidence1Callback,this);
	sub_conf2 = nh.subscribe("/confidence2",1,&DataProcessor::confidence2Callback,this);
	sub_selectorState1 = nh.subscribe("/selector_state1",1,&DataProcessor::selectorState1Calllback,this);
	sub_selectorState2 = nh.subscribe("/selector_state2",1,&DataProcessor::selectorState2Calllback,this);


/*
	sub_rgb2 = it_nh.subscribe("/ground_station_rgb2",5, &DataProcessor::rgbFeed2Callback, this); 
	sub_target_wp2 = nh.subscribe("/target_wp2",5, &DataProcessor::targetwp2Callback,this);
	sub_vector_wp2 = nh.subscribe("/wp_pose2",5, &DataProcessor::vectorwp2Callback,this);
	sub_conv_fac2 = nh.subscribe("/conv_fac2",5, &DataProcessor::convFac2Callback,this);
*/
    //sub_goal = nh.subscribe("/goal_pose",20, &DataProcessor::goalCallback, this);


    undistorted_pts[0].x=0;
    undistorted_pts[1].x=board_w-1;
    undistorted_pts[2].x=0;
    undistorted_pts[3].x=board_w-1;
    undistorted_pts[0].y=0;
    undistorted_pts[1].y=0;
    undistorted_pts[2].y=board_h-1;
    undistorted_pts[3].y=board_h-1;

	corners1_vec.push_back(Point2f(undistorted_pts[0].x,undistorted_pts[0].y));
	corners1_vec.push_back(Point2f(undistorted_pts[1].x,undistorted_pts[1].y));
	corners1_vec.push_back(Point2f(undistorted_pts[2].x,undistorted_pts[2].y));
	corners1_vec.push_back(Point2f(undistorted_pts[3].x,undistorted_pts[3].y));
	corners2_vec.push_back(Point2f(undistorted_pts[0].x,undistorted_pts[0].y));
	corners2_vec.push_back(Point2f(undistorted_pts[1].x,undistorted_pts[1].y));
	corners2_vec.push_back(Point2f(undistorted_pts[2].x,undistorted_pts[2].y));
	corners2_vec.push_back(Point2f(undistorted_pts[3].x,undistorted_pts[3].y));

    H_camcam = getPerspectiveTransform(undistorted_pts, undistorted_pts);
    H_cambird = getPerspectiveTransform(undistorted_pts, undistorted_pts);
    getPerspectives();
	cout << H_camcam << endl;
	cout << H_cambird << endl;
	

	
	// Set each element in history to 0
	for (int i = 0; i < VEHICLE_POSE_HISTORY_SIZE; i++) {
	    vehicle_pose_history[i] = Point(0, 0);
	}

	}

	~DataProcessor()
	{
    //cv::destroyWindow(); // all
	}

	string intToString(int number){ // for writing txt on frame
		std::stringstream ss;
		ss << number;
		return ss.str();
	}

	void downsampleFrame(Mat img) {
		Size size(img.cols / scale_factor , img.rows / scale_factor); // this should be 160 x 120 
		resize(img,img,size);
	}

	void upsampleFrame(Mat &img) {
		Size size(img.cols * 2 , img.rows * 2); // this should be 160 x 120 
		resize(img,img,size);
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


	void selectorState1Calllback (const std_msgs::Int8::ConstPtr& selector_state_msg) 
	{
		selector_state1 = selector_state_msg -> data;
	    ROS_INFO("selector_state1: ( %i )",selector_state1);
	}


	void selectorState2Calllback (const std_msgs::Int8::ConstPtr& selector_state_msg) 
	{
		selector_state2 = selector_state_msg -> data;
	    ROS_INFO("selector_state2: ( %i )",selector_state2);
	}


	void convFac1Callback (const std_msgs::Float64::ConstPtr& conv_fac_msg) 
	{
		conv_fac = conv_fac_msg -> data;
	    ROS_INFO("convFacCallback: ( %f )",conv_fac);
	}


	void targetAngle1Callback (const std_msgs::Float64::ConstPtr& target_angle_msg) 
	{
		target_angle = target_angle_msg -> data;
	    ROS_INFO("targetAngleCallback: ( %f )",target_angle);
	}

	void targetwp1Callback (const geometry_msgs::Pose2D::ConstPtr& target_wp_msg) 
	{
	    target_wp.x  = target_wp_msg->x / scale_factor;
	    target_wp.y = target_wp_msg->y / scale_factor;
	    ROS_INFO("targetwpCallback: ( %i , %i )",target_wp.x,target_wp.y);
	}

	void vectorwp1Callback (const geometry_msgs::PoseArray::ConstPtr& vector_wp_msg) 
	{
    	ROS_INFO("vectorwpDownCallback");
    	vector_wp.clear();
    	int size = vector_wp_msg->poses.size();
    	for (int i=0;i<size;i++)
    	{
    		vector_wp.push_back(Point((int)vector_wp_msg->poses[i].position.x,(int)vector_wp_msg->poses[i].position.y));
		}
	}

	void targetwp2Callback (const geometry_msgs::Pose2D::ConstPtr& target_wp_msg) 
	{
	    target_wp2.x  = target_wp_msg->x / scale_factor;
	    target_wp2.y = target_wp_msg->y / scale_factor;
	    ROS_INFO("targetwp2Callback: ( %i , %i )",target_wp2.x,target_wp2.y);
	}

	void vectorwp2Callback (const geometry_msgs::PoseArray::ConstPtr& vector_wp_msg) 
	{
    	ROS_INFO("vectorwpDownCallback");
    	vector_wp2.clear();
    	int size = vector_wp_msg->poses.size();
    	for (int i=0;i<size;i++)
    	{
    		vector_wp2.push_back(Point((int)vector_wp_msg->poses[i].position.x,(int)vector_wp_msg->poses[i].position.y));
		}
	}

	void vehicle1Callback (const geometry_msgs::Pose2D::ConstPtr& vehicle_pose_msg) 
	{
		double xtemp, ytemp;
	    xtemp = vehicle_pose_msg->x / scale_factor + 160;
	    ytemp = vehicle_pose_msg->y / scale_factor + 120;
	    heading_angle  = vehicle_pose_msg->theta;
	    Point2f vehicle_pose_temp;
	    int ID_num = 1;
	    // transform coordinates
	    if (ID_num > 1) { // ID = 2, HbirdHcamcamOG
	                float x = H_camcam_inv.at<double>(0,0) * xtemp + H_camcam_inv.at<double>(0,1) * ytemp + H_camcam_inv.at<double>(0,2);
	                float y = H_camcam_inv.at<double>(1,0) * xtemp + H_camcam_inv.at<double>(1,1) * ytemp + H_camcam_inv.at<double>(1,2);
	                float w = H_camcam_inv.at<double>(2,0) * xtemp + H_camcam_inv.at<double>(2,1) * ytemp + H_camcam_inv.at<double>(2,2);

	                vehicle_pose_temp=Point(x/w,y/w);


	    } else { // ID_num = 0 , HbirdOG

	                vehicle_pose_temp=Point(xtemp,ytemp);

	    }
	    vehicle_pose.x  = (int)vehicle_pose_temp.x;
	    vehicle_pose.y = (int)vehicle_pose_temp.y;

	    ROS_INFO("vehicleCallback: ( %i , %i )",vehicle_pose.x,vehicle_pose.y);
	}

	void vehicle2Callback (const geometry_msgs::Pose2D::ConstPtr& vehicle_pose_msg) 
	{
		double xtemp, ytemp;
	    xtemp = vehicle_pose_msg->x / scale_factor + 160;
	    ytemp = vehicle_pose_msg->y / scale_factor + 120;
	    heading_angle2  = vehicle_pose_msg->theta;
	    Point2f vehicle_pose_temp;
	    int ID_num = 2;
	    // transform coordinates
	    if (ID_num > 1) { // ID = 2, HbirdHcamcamOG
	                float x = H_camcam_inv.at<double>(0,0) * xtemp + H_camcam_inv.at<double>(0,1) * ytemp + H_camcam_inv.at<double>(0,2);
	                float y = H_camcam_inv.at<double>(1,0) * xtemp + H_camcam_inv.at<double>(1,1) * ytemp + H_camcam_inv.at<double>(1,2);
	                float w = H_camcam_inv.at<double>(2,0) * xtemp + H_camcam_inv.at<double>(2,1) * ytemp + H_camcam_inv.at<double>(2,2);

	                vehicle_pose_temp=Point(x/w,y/w);


	    } else { // ID_num = 0 , HbirdOG

	                vehicle_pose_temp=Point(xtemp,ytemp);

	    }
	    vehicle_pose2.x  = (int)vehicle_pose_temp.x;
	    vehicle_pose2.y = (int)vehicle_pose_temp.y;

	    ROS_INFO("vehicleCallback2: ( %i , %i )",vehicle_pose2.x,vehicle_pose2.y);
	}

	void targetAngle2Callback (const std_msgs::Float64::ConstPtr& target_angle_msg) 
	{
		target_angle2 = target_angle_msg -> data;
	    ROS_INFO("targetAngle2Callback: ( %f )",target_angle2);
	}

	/*	void convFac2Callback (const std_msgs::Float64::ConstPtr& conv_fac_msg) 
	{
		conv_fac = conv_fac_msg -> data;
	    ROS_INFO("convFacCallback: ( %f )",conv_fac);
	}

*/

	void updateGoal() 
	{
		int size = goal_points.size();

		geometry_msgs::PoseArray poseArray;
	    poseArray.poses.clear();
	    poseArray.header.stamp=ros::Time::now();

		geometry_msgs::Pose goal_pose_out_msg;

		for (int i=0; i<size; i++) 
		{
			goal_pose_out_msg.position.x = goal_points[i].x;
	        goal_pose_out_msg.position.y = goal_points[i].y;

	        poseArray.poses.push_back(goal_pose_out_msg);

		}
		
		ROS_INFO("subgoal set to (%f, %f), goal vector size: %i ", poseArray.poses[0].position.x, poseArray.poses[0].position.y, size);
		
		pub_goal_out.publish(poseArray); 
		
	}

	void drawData(Mat &frame)
	{

		Scalar path_color = Scalar(150,150,0);
		Scalar wp_color = Scalar(150,150,0);
		Scalar wp_color2 = Scalar(0,150,150);
		Scalar path_color2 = Scalar(0,150,150);

		Scalar goal_color = Scalar(255,0,0);

	    vehicle_heading.x = (int) round(vehicle_pose.x + LOS_RADIUS * cos(heading_angle));
	    vehicle_heading.y = (int) round(vehicle_pose.y + LOS_RADIUS * sin(heading_angle));		    
	    line(frame, vehicle_pose, vehicle_heading, Scalar(255, 50, 0), HEADING_LINE_THICKNESS, 8, 0);



	    vehicle_heading2.x = (int) round(vehicle_pose2.x + LOS_RADIUS * cos(heading_angle));
	    vehicle_heading2.y = (int) round(vehicle_pose2.y + LOS_RADIUS * sin(heading_angle));		    
	    line(frame, vehicle_pose2, vehicle_heading2, Scalar(0, 50, 255), HEADING_LINE_THICKNESS, 8, 0);
		
	    if (selector_state1 == 1 && selector_state2 == 1) {
		    circle(frame,vehicle_pose,LOS_RADIUS, Scalar(255,0,0),2);	
		    circle(frame,vehicle_pose2,LOS_RADIUS-1, Scalar(0, 0, 255),1);
	    } else if (selector_state1 == 2 && selector_state2 == 2) {
			circle(frame,vehicle_pose,LOS_RADIUS, Scalar(255,0,0),1);
		    circle(frame,vehicle_pose2,LOS_RADIUS-1, Scalar(0, 0, 255),2);
	    } else {
			circle(frame,vehicle_pose,LOS_RADIUS, Scalar(255,0,0),1);
		    circle(frame,vehicle_pose2,LOS_RADIUS-1, Scalar(0, 0, 255),1);
	    }
		
		int size = vector_wp.size();
		ROS_INFO("wp vector size = %i",size);
    	for (int i=0;i<size;i++)
    	{
    		circle(frame,vector_wp[i], 2, path_color,1);
    		//ROS_INFO("wp:%i,%i",vector_wp[i].x,vector_wp[i].y);

		}

		int size2 = vector_wp2.size();
		ROS_INFO("wp2 vector size = %i",size2);
    	for (int i=0;i<size2;i++)
    	{
    		circle(frame,vector_wp2[i], 1, path_color2,-1);
    		//ROS_INFO("wp:%i,%i",vector_wp[i].x,vector_wp[i].y);

		}


		if (goal_points.size() > 0 ) 
		{
			for (int j = 0; j < goal_points.size(); j++)
			{
				circle(frame,goal_points[j], 2, goal_color,5);
			}

		}

		circle(frame,Point((int)target_wp.x, (int)target_wp.y), 2, wp_color,1);
		circle(frame,Point((int)target_wp2.x, (int)target_wp2.y), 2, wp_color2,-1);

		// draw target angle vector
		Point target_angle_endpoint;
	    target_angle_endpoint.x = (int) round(vehicle_pose.x + LOS_RADIUS * cos(target_angle));
	    target_angle_endpoint.y = (int) round(vehicle_pose.y + LOS_RADIUS * sin(target_angle));		    
	    line(frame, vehicle_pose, target_angle_endpoint, Scalar(200, 255, 0), HEADING_LINE_THICKNESS, 8, 0);

   		Point target_angle_endpoint2;
	    target_angle_endpoint2.x = (int) round(vehicle_pose2.x + LOS_RADIUS * cos(target_angle2));
	    target_angle_endpoint2.y = (int) round(vehicle_pose2.y + LOS_RADIUS * sin(target_angle2));		    
	    line(frame, vehicle_pose2, target_angle_endpoint2, Scalar(0, 255, 200), HEADING_LINE_THICKNESS, 8, 0);

	   	circle(frame, Point(corners1_pts[0].x,corners1_pts[0].y), 1, Scalar(255,200,0),1);
	   	circle(frame, Point(corners1_pts[1].x,corners1_pts[1].y), 1, Scalar(255,0,0),1);
	   	circle(frame, Point(corners1_pts[2].x,corners1_pts[2].y), 1, Scalar(255,0,0),1);
	  	circle(frame, Point(corners1_pts[3].x,corners1_pts[3].y), 1, Scalar(255,0,0),1);
	  	circle(frame, Point(corners2_pts_camcam[0].x,corners2_pts_camcam[0].y), 4, Scalar(0,200,255),1);
	 	circle(frame, Point(corners2_pts_camcam[1].x,corners2_pts_camcam[1].y), 4, Scalar(0,0,255),1);
	   	circle(frame, Point(corners2_pts_camcam[2].x,corners2_pts_camcam[2].y), 4, Scalar(0,0,255),1);
	 	
/*
	   	circle(frame, Point(160+corners1_pts[0].x/scale_factor-corners2_pts[0].x/scale_factor,120+corners1_pts[0].y/scale_factor - corners2_pts[0].y/scale_factor), 4, Scalar(255,0,0),1);
	   	circle(frame, Point(160+corners1_pts[1].x/scale_factor-corners2_pts[1].x/scale_factor,120+corners1_pts[1].y/scale_factor - corners2_pts[1].y/scale_factor), 4, Scalar(255,0,0),1);
	   	circle(frame, Point(160+corners1_pts[2].x/scale_factor-corners2_pts[2].x/scale_factor,120+corners1_pts[2].y/scale_factor - corners2_pts[2].y/scale_factor), 4, Scalar(255,0,0),1);
	   	circle(frame, Point(160+corners1_pts[3].x/scale_factor-corners2_pts[3].x/scale_factor,120+corners1_pts[3].y/scale_factor - corners2_pts[3].y/scale_factor), 4, Scalar(255,0,0),1);
*/
		circle(frame,vehicle_pose,2, Scalar(255, 0, 0),1);
		circle(frame,vehicle_pose2,2, Scalar(0, 0, 255),1);

		putText(frame,"--- MSSP1 ---",Point(5,15),1,1,Scalar(0,0,255),1); 

		putText(frame,"--- MSSP2 ---",Point(250,15),1,1,Scalar(255,0,0),1); 

		char confidence1txt[25];
		sprintf(confidence1txt,"confidence1 : %f",confidence1);
		putText(frame,confidence1txt,Point(5,40),1,1,Scalar(0,0,255),1); 
		char confidence2txt[25];
		sprintf(confidence2txt,"confidence2 : %f",confidence2);
		putText(frame,confidence2txt,Point(250,40),1,1,Scalar(255,0,0),1); 
		char selectorState1txt[25];
		sprintf(selectorState1txt,"selector_state1 : %i",selector_state1);		
		putText(frame,selectorState1txt,Point(5,65),1,1,Scalar(0,0,255),1); 
		char selectorState2txt[25];
		sprintf(selectorState2txt,"selector_state2 : %i",selector_state2);		
		putText(frame,selectorState2txt,Point(250,65),1,1,Scalar(255,0,0),1); 
		char vehicle_pose1txt[25];
		sprintf(vehicle_pose1txt,"vehicle_pose1 : (%i , %i)",vehicle_pose.x,vehicle_pose.y);		
		putText(frame,vehicle_pose1txt,Point(5,90),1,1,Scalar(0,0,255),1); 
		char vehicle_pose2txt[25];
		sprintf(vehicle_pose2txt,"vehicle_pose2 : (%i , %i)",vehicle_pose2.x,vehicle_pose2.y);		
		putText(frame,vehicle_pose2txt,Point(250,90),1,1,Scalar(255,0,0),1);
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




	Mat translateImg(Mat &img, int offsetx, int offsety)
	{
		Mat trans_mat = (Mat_<double>(2, 3) << 1, 0, offsetx, 0, 1, offsety);

		warpAffine(img, img, trans_mat, Size(3 * img.cols, 3 * img.rows)); // 3,4 is usual
		return trans_mat;
	}

	// void warp_crops(Mat& im_1, const Mat& im_2)
	// {
	// 	cv::Ptr<Feature2D> f2d = xfeatures2d::SIFT::create();


	// 	// Step 1: Detect the keypoints:
	// 	std::vector<KeyPoint> keypoints_1, keypoints_2;
	// 	f2d->detect( im_1, keypoints_1 );
	// 	f2d->detect( im_2, keypoints_2 );

	// 	// Step 2: Calculate descriptors (feature vectors)
	// 	Mat descriptors_1, descriptors_2;
	// 	f2d->compute( im_1, keypoints_1, descriptors_1 );
	// 	f2d->compute( im_2, keypoints_2, descriptors_2 );

	// 	// Step 3: Matching descriptor vectors using BFMatcher :
	// 	BFMatcher matcher;
	// 	std::vector< DMatch > matches;
	// 	matcher.match( descriptors_1, descriptors_2, matches );

	// 	// Keep best matches only to have a nice drawing.
	// 	// We sort distance between descriptor matches
	// 	Mat index;
	// 	int nbMatch = int(matches.size());
	// 	Mat tab(nbMatch, 1, CV_32F);
	// 	for (int i = 0; i < nbMatch; i++)
	// 		tab.at<float>(i, 0) = matches[i].distance;
	// 	sortIdx(tab, index, SORT_EVERY_COLUMN + SORT_ASCENDING);
	// 	vector<DMatch> bestMatches;

	// 	for (int i = 0; i < 200; i++)
	// 		bestMatches.push_back(matches[index.at < int > (i, 0)]);


	// 	// 1st image is the destination image and the 2nd image is the src image
	// 	std::vector<Point2f> dst_pts;                   //1st
	// 	std::vector<Point2f> source_pts;                //2nd

	// 	for (vector<DMatch>::iterator it = bestMatches.begin(); it != bestMatches.end(); ++it) {
	// 		cout << it->queryIdx << "\t" <<  it->trainIdx << "\t"  <<  it->distance << "\n";
	// 		//-- Get the keypoints from the good matches
	// 		dst_pts.push_back( keypoints_1[ it->queryIdx ].pt );
	// 		source_pts.push_back( keypoints_2[ it->trainIdx ].pt );
	// 	}

	// 	// Mat img_matches;
	// 	// drawMatches( im_1, keypoints_1, im_2, keypoints_2,
	// 	//           bestMatches, img_matches, Scalar::all(-1), Scalar::all(-1),
	// 	//           vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
	// 	//
	// 	// //-- Show detected matches
	// 	// imwrite( "Good_Matches.jpg", img_matches );



	// 	Mat H = findHomography( source_pts, dst_pts, CV_RANSAC );
	// 	cout << H << endl;

	// 	Mat wim_2;
	// 	warpPerspective(im_2, wim_2, H, im_1.size());

	// 	for (int i = 0; i < im_1.cols; i++)
	// 		for (int j = 0; j < im_1.rows; j++) {
	// 			Vec3b color_im1 = im_1.at<Vec3b>(Point(i, j));
	// 			Vec3b color_im2 = wim_2.at<Vec3b>(Point(i, j));
	// 			if (norm(color_im1) == 0)
	// 				im_1.at<Vec3b>(Point(i, j)) = color_im2;

	// 		}

	// }

	double distCalc(Point2f p1, Point2f p2)
	{
		double x = p1.x - p2.x;
		double y = p1.y - p2.y;

		double dist;
		dist = pow(x, 2) + pow(y, 2);       //calculate Euclidean distance
		dist = sqrt(dist);                  

		return dist;
	}

	double getAffineScale(vector<Point2f> c1, vector<Point2f> c2) 
	{
	
		double h1L = distCalc(c1[0],c1[2]);
		double h1R = distCalc(c1[1],c1[3]);
		double h2L = distCalc(c2[0],c2[2]);
		double h2R = distCalc(c2[1],c2[3]);
		double w1T = distCalc(c1[0],c1[1]);
		double w1B = distCalc(c1[2],c1[3]);
		double w2T = distCalc(c2[0],c2[1]);
		double w2B = distCalc(c2[2],c2[3]);
		
		double h1 = (h1L + h1R)/2;
		double h2 = (h2L + h2R)/2;
		
		double w1 = (w1T + w1B)/2;
		double w2 = (w2T+ w2B)/2;
		
		double scale = ((h1/h2) + (w1/w2))/2;
	}


/*	double getAffineAngle( Point2f c1, Point2f c2) 
	{
	//	quat RotationBetweenVectors(vec3 start, vec3 dest){

		start = normalize(start);
		dest = normalize(dest);

		float cosTheta = dot(start, dest);
		vec3 rotationAxis;

		if (cosTheta < -1 + 0.001f){
			// special case when vectors in opposite directions:
			// there is no "ideal" rotation axis
			// So guess one; any will do as long as it's perpendicular to start
			rotationAxis = cross(vec3(0.0f, 0.0f, 1.0f), start);
			if (gtx::norm::length2(rotationAxis) < 0.01 ) // bad luck, they were parallel, try again!
				rotationAxis = cross(vec3(1.0f, 0.0f, 0.0f), start);

			rotationAxis = normalize(rotationAxis);
			return gtx::quaternion::angleAxis(glm::radians(180.0f), rotationAxis);
		}

		rotationAxis = cross(start, dest);

		float s = sqrt( (1+cosTheta)*2 );
		float invs = 1 / s;

		return quat(
			s * 0.5f, 
			rotationAxis.x * invs,
			rotationAxis.y * invs,
			rotationAxis.z * invs
		);

	}*/


	void affineImg(vector<Point2f> c1, vector<Point2f> c2, double scale, Mat &temp)
	{

	   
	   Point2f srcTri[3];
	   Point2f dstTri[3];

	   Mat rot_mat( 2, 3, CV_32FC1 );
	   Mat warp_mat( 2, 3, CV_32FC1 );
	   Mat src, warp_dst, warp_rotate_dst;

	   /// Load the image
	   temp.copyTo(src);

	   /// Set the dst image the same type and size as src
	   warp_dst = Mat::zeros( src.rows, src.cols, src.type() );

	   /// Set your 3 points to calculate the  Affine Transform
	   srcTri[0] = c2[0];
	   srcTri[1] = c2[1];
	   srcTri[2] = c2[2];

	   dstTri[0] = c1[0];
	   dstTri[1] = c1[1];
	   dstTri[2] = c1[2];

	   /// Get the Affine Transform
	   warp_mat = getAffineTransform( srcTri, dstTri );

	   /// Apply the Affine Transform just found to the src image
	   warpAffine( src, warp_dst, warp_mat, warp_dst.size() );
	   warp_dst.copyTo(temp);

	}



	void updateMap(Mat &temp, int ID) {
    	Mat worldMaptemp(temp.rows*3,temp.cols*3,temp.type(),Scalar(0));
    	if (counter < 1) { //zero out
    		worldMaptemp.copyTo(worldMap);
    		worldMap.copyTo(worldMap1);
    		worldMap.copyTo(worldMap2);
    	}
	   	worldMap_tic++;;
    	//else, for 2 count, add images from ID 1 and 2
	    if (ID > 1) { // ID = 2, HbirdHcamcamOG

	   		worldMap1.setTo(Scalar(0));
	   		//worldMaptemp.setTo(Scalar(0));
	   		temp.copyTo(worldMap1(Rect(temp.cols,temp.rows,temp.cols,temp.rows)));
	   		cout << "c1: " << corners1_vec << endl;
	   		cout << "c2: "<< corners2_vec << endl;
	   		double scale = getAffineScale(corners1_vec, corners2_vec);
	   		cout << "scale: "<< scale << endl;
	   		affineImg(corners1_vec, corners2_vec,scale, worldMap1);
			cout << "affine done" << endl;
/*	        
for (int xt = 1; xt <= 160; xt++)
       		{
            	for (int yt = 1; yt<=120; yt++)
            	{   
	                cv::Vec3b pix = temp.at<cv::Vec3b>(yt,xt);

		        	float x = H_camcam_inv.at<double>(0,0) * xt + H_camcam_inv.at<double>(0,1) * yt + H_camcam_inv.at<double>(0,2);
		        	float y = H_camcam_inv.at<double>(1,0) * xt + H_camcam_inv.at<double>(1,1) * yt + H_camcam_inv.at<double>(1,2);
		        	float w = H_camcam_inv.at<double>(2,0) * xt + H_camcam_inv.at<double>(2,1) * yt + H_camcam_inv.at<double>(2,2);
		        	float xf = x/w;
		        	float yf = y/w;
		        	cout << x << "," << y << "," <<  w << endl;

		        	cout << xt << "," << yt << endl;

		        	cout << xf << "," << yf << endl;

		        	//cout << "original : " << temp.at<cv::Vec3b>(yt,xt) << endl;
		        	
		        	if (xf < 480 && yf < 360){
			        	worldMap1.at<cv::Vec3b>((int)yf,(int)xf)[0] = pix[0];
		            	worldMap1.at<cv::Vec3b>((int)yf,(int)xf)[1] = pix[1];
		            	worldMap1.at<cv::Vec3b>((int)yf,(int)xf)[2] = pix[2];
       		        	//cout << "transformed : " << worldMap1.at<cv::Vec3b>(yf,xf) << endl;

		            }
	           	}
        	}
*/

			//warpPerspective(worldMaptemp , worldMaptemp, H_camcam_inv, worldMaptemp.size(), INTER_LINEAR, BORDER_CONSTANT, Scalar::all(0)); 	    
	   		
	   		//worldMaptemp.copyTo(worldMap1);

	    } else { // ID_num = 1 
	   		worldMap2.setTo(Scalar(0));
    		temp.copyTo(worldMap2(Rect(temp.cols,temp.rows,temp.cols,temp.rows)));

    	}

    	// publish and reset
    	//if (worldMap_tic >= 2) {
    		worldMap_tic = 0;
    		addWeighted(worldMap1,0.6,worldMap2,0.6,0.0,worldMap);
		    drawData(worldMap);
		    upsampleFrame(worldMap);
	    	imshow(windowName,worldMap);

			char k = (char) cv::waitKey(30); //wait for esc key

			std_msgs::Int8 key_cmd_msg;
			//if(k == 27) break;
			if(k== ' ')  // start tracking : case 1 and key_cmd int value 1
			{
				key_cmd_msg.data = 1; 
			} 
			else if (k==27) // reinitialize background : case 2 and key_cmd int value 2
			{
				key_cmd_msg.data = 2; 
				

			}
			else {key_cmd_msg.data = 0;}

			key_cmd_pub.publish(key_cmd_msg);

		   	std_msgs::Header header;
	    	header.stamp = ros::Time::now();
	    	cv_bridge::CvImage worldMap_bridge;
	    	sensor_msgs::Image worldMap_msg;
	    	worldMap_bridge = cv_bridge::CvImage(header,sensor_msgs::image_encodings::BGR8, worldMap);
	    	worldMap_bridge.toImageMsg(worldMap_msg);
	    	pub_worldMap.publish(worldMap_msg);

		//}


	}

	void corners1Callback (const geometry_msgs::PoseArray::ConstPtr& corners1_msg) 
	{

	    corners1_vec.clear();
	    int size = corners1_msg->poses.size();
	    for (int i=0;i<size;i++)
	    {
	        corners1_vec.push_back(Point2f(160+corners1_msg->poses[i].position.x/scale_factor,120+corners1_msg->poses[i].position.y/scale_factor));
	        corners1_pts[i] = corners1_vec[i];    
	    }
	    ROS_INFO("corners1");
        cout << corners1_vec << endl;

	    getPerspectives();
	}

	void corners2Callback (const geometry_msgs::PoseArray::ConstPtr& corners2_msg) 
	{		

	    
	    corners2_vec.clear();
	    int size = corners2_msg->poses.size();
	    for (int i=0;i<size;i++)
	    {
	        corners2_vec.push_back(Point2f(160+(int)corners2_msg->poses[i].position.x/scale_factor,120+(int)corners2_msg->poses[i].position.y/scale_factor));
	        corners2_pts[i] = corners2_vec[i];    
	    }

		ROS_INFO("corners2");
        cout << corners2_vec << endl;

	    getPerspectives();

		for (int n=0;n<=3; n++) {

	        float x = H_camcam_inv.at<double>(0,0) * corners2_msg->poses[n].position.x + H_camcam_inv.at<double>(0,1) * corners2_msg->poses[n].position.y + H_camcam_inv.at<double>(0,2);
	        float y = H_camcam_inv.at<double>(1,0) * corners2_msg->poses[n].position.x + H_camcam_inv.at<double>(1,1) * corners2_msg->poses[n].position.y + H_camcam_inv.at<double>(1,2);
	        float w = H_camcam_inv.at<double>(2,0) * corners2_msg->poses[n].position.x + H_camcam_inv.at<double>(2,1) * corners2_msg->poses[n].position.y + H_camcam_inv.at<double>(2,2);

	        corners2_pts_camcam[n]=Point(x/w,y/w);
		
		}

	}
	
	void rgbFeed1Callback(const sensor_msgs::ImageConstPtr& msg)
	{

		cv_bridge::CvImage img_bridge;
		sensor_msgs::Image img_msg;
		cv_bridge::CvImagePtr cv_ptr;
		
		try
		{
			cv_ptr  = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_INFO("cv_bridge exception: %s", e.what());
			return;
		}

	    std_msgs::Header header; //empty header
	    header.seq = counter; // user defined counter
	    header.stamp = ros::Time::now(); // time
	    
	    frame = cv_ptr -> image;

	    // update mouseclick events
	    if (goal_points.size() > 0 ) 
		{
	    	updateGoal();
		}

		updateMap(frame,1);

		img_bridge = cv_bridge::CvImage(header,sensor_msgs::image_encodings::BGR8,frame);
		img_bridge.toImageMsg(img_msg);
		pub_vis1.publish(img_msg);

	    counter++;
	}
	
	void rgbFeed2Callback(const sensor_msgs::ImageConstPtr& msg2)
	{

		cv_bridge::CvImage img_bridge2;
		sensor_msgs::Image img_msg2;
		cv_bridge::CvImagePtr cv_ptr2;
		
		try
		{
			cv_ptr2  = cv_bridge::toCvCopy(msg2,sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e2)
		{
			ROS_INFO("cv_bridge exception: %s", e2.what());
			return;
		}

	    std_msgs::Header header2; //empty header
	    header2.seq = counter2; // user defined counter
	    header2.stamp = ros::Time::now(); // time
	    frame2 = cv_ptr2 -> image;
		
		img_bridge2 = cv_bridge::CvImage(header2,sensor_msgs::image_encodings::BGR8,frame2);
		img_bridge2.toImageMsg(img_msg2);
		pub_vis2.publish(img_msg2);
		
		ROS_INFO("GROUNDSTATION2 FRAME COLLECTED, size : %i x %i" , frame2.cols, frame2.rows);
		updateMap(frame2,2);



	    counter2++;
	}


private:

	// INITIALIZATION ////////////////////////////////////////////////////////////////////

	int counter = 0;
	int counter2 = 0;
	Mat scaledFrame;
	Mat scaledFrame2;
	Mat frame;
	Mat frame2;
	Mat worldMap;
	Mat worldMap1;
	Mat worldMap2;

	Mat H_camcam_inv;

	//astar Params
	int astar_size = 150; // change to length of car

	image_transport::Subscriber sub_rgb1;
	image_transport::Subscriber sub_rgb2;
	image_transport::Publisher pub_worldMap;
	image_transport::Publisher pub_vis1;
	image_transport::Publisher pub_vis2;
	ros::Publisher pub_goal_out;
	ros::Publisher key_cmd_pub;

	ros::Subscriber sub_target_wp1;
	ros::Subscriber sub_vector_wp1;
	ros::Subscriber sub_target_angle1;
	ros::Subscriber sub_conv_fac1;
	ros::Subscriber sub_goal1;
	ros::Subscriber sub_vehicle1;
	ros::Subscriber sub_target_wp2;
	ros::Subscriber sub_vector_wp2;
	ros::Subscriber sub_target_angle2;
	ros::Subscriber sub_conv_fac2;
	ros::Subscriber sub_goal2;
	ros::Subscriber sub_vehicle2;
	ros::Subscriber sub_corners2;
	ros::Subscriber sub_corners1;
	ros::Subscriber sub_conf1;
	ros::Subscriber sub_conf2;
	ros::Subscriber sub_selectorState1;
	ros::Subscriber sub_selectorState2;


	double target_angle = 0;
	double target_angle2=0;
	double heading_angle2=0;
	double heading_angle = 0;

	// vehicle location history
	Point * vehicle_pose_history = new Point[VEHICLE_POSE_HISTORY_SIZE];
	
	int vehicle_pose_history_pointer;

	double conv_fac;

	int scaledFrame_height;
	int scaledFrame_width;

	Point vehicle_pose;
	Point vehicle_pose2;
	Point goal_pose;
	Point vehicle_heading;
	Point vehicle_heading2;

	vector<Point2f> corners2_vec;
	
	Point2f corners2_pts_camcam[4];

	vector<Point2f> corners1_vec;
	Point2f undistorted_pts[4];
	Point2f corners1_pts[4];
	Point2f corners2_pts[4];

	int corner_tic = 0;
	int worldMap_tic = 0;
	int board_w = 80;
	int board_h = 60;
	Mat H_camcam;
	Mat H_cambird;

	int selector_state1 = 0;
	int selector_state2 = 0;
	double confidence1 = 0;
	double confidence2 = 0;

};



//
// MAIN //////////////////////////////////////////////////////////////////////////////////////////////////
//

int main(int argc, char** argv)
{

	ros::init(argc, argv, "data_processor_node");
	ROS_INFO("launching data_processor_node");

	namedWindow(windowName);

   	cv::setMouseCallback(windowName,onMouse,(void*)(&goal_points));

	DataProcessor dp;
	
	ros::spin();

	return 0;

}



