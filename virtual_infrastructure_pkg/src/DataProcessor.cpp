

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

#define LOS_RADIUS 10 //currently in pixels
#define HEADING_LINE_THICKNESS 2
#define VEHICLE_POSE_HISTORY_SIZE 20

using namespace std;
using namespace cv;
const int scale_factor = 8; // this needs to be the same as in the rgb node. change to launch file parameter. 
// this may not work for all resolutions. this value needs to match the value in astar. retrive from param launch file.

const string windowName = "Visualizer";

vector<Point> goal_points;

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
	sub_rgb2 = it_nh.subscribe("/ground_station_rgb2",5, &DataProcessor::rgbFeed2Callback, this); 
	sub_rgb1 = it_nh.subscribe("/ground_station_rgb1",5, &DataProcessor::rgbFeed1Callback, this); 
	sub_target_wp1 = nh.subscribe("/target_wp1",5, &DataProcessor::targetwp1Callback,this);
	sub_vector_wp1 = nh.subscribe("/wp_pose1",5, &DataProcessor::vectorwp1Callback,this);
	sub_target_angle1 = nh.subscribe("/target_angle1",5, &DataProcessor::targetAngle1Callback,this);
	sub_conv_fac1 = nh.subscribe("/conv_fac1",5, &DataProcessor::convFac1Callback,this);
	sub_vehicle1 = nh.subscribe("/vehicle_pose1",5, &DataProcessor::vehicle1Callback, this);
/*
	sub_rgb2 = it_nh.subscribe("/ground_station_rgb2",5, &DataProcessor::rgbFeed2Callback, this); 
	sub_target_wp2 = nh.subscribe("/target_wp2",5, &DataProcessor::targetwp2Callback,this);
	sub_vector_wp2 = nh.subscribe("/wp_pose2",5, &DataProcessor::vectorwp2Callback,this);
	sub_target_angle2 = nh.subscribe("/target_angle2",5, &DataProcessor::targetAngle2Callback,this);
	sub_conv_fac2 = nh.subscribe("/conv_fac2",5, &DataProcessor::convFac2Callback,this);
	sub_vehicle2 = nh.subscribe("/vehicle_pose2",5, &DataProcessor::vehicle2Callback, this);*/
    //sub_goal = nh.subscribe("/goal_pose",20, &DataProcessor::goalCallback, this);

    undistorted_pts[0].x=0;
    undistorted_pts[1].x=board_w-1;
    undistorted_pts[2].x=0;
    undistorted_pts[3].x=board_w-1;
    undistorted_pts[0].y=0;
    undistorted_pts[1].y=0;
    undistorted_pts[2].y=board_h-1;
    undistorted_pts[3].y=board_h-1;

    H_camcam = getPerspectiveTransform(undistorted_pts, undistorted_pts);
    H_cambird = getPerspectiveTransform(undistorted_pts, undistorted_pts);
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

	    ROS_INFO("vehicleCallback: ( %i , %i )",vehicle_pose2.x,vehicle_pose2.y);
	}

	/*	void convFac2Callback (const std_msgs::Float64::ConstPtr& conv_fac_msg) 
	{
		conv_fac = conv_fac_msg -> data;
	    ROS_INFO("convFacCallback: ( %f )",conv_fac);
	}


	void targetAngle2Callback (const std_msgs::Float64::ConstPtr& target_angle_msg) 
	{
		target_angle = target_angle_msg -> data;
	    ROS_INFO("targetAngleCallback: ( %f )",target_angle);
	}

	void targetwp2Callback (const geometry_msgs::Pose2D::ConstPtr& target_wp_msg) 
	{
	    target_wp.x  = target_wp_msg->x;
	    target_wp.y = target_wp_msg->y;
	    ROS_INFO("targetwpCallback: ( %i , %i )",target_wp.x,target_wp.y);
	}

	void vectorwp2Callback (const geometry_msgs::PoseArray::ConstPtr& vector_wp_msg) 
	{
    	ROS_INFO("vectorwpDownCallback");
    	vector_wp.clear();
    	int size = vector_wp_msg->poses.size();
    	for (int i=0;i<size;i++)
    	{
    		vector_wp.push_back(Point((int)vector_wp_msg->poses[i].position.x,(int)vector_wp_msg->poses[i].position.y));
		}
	}

	void vehicle2Callback (const geometry_msgs::Pose2D::ConstPtr& vehicle_pose_msg) 
	{
		double xtemp, ytemp;
	    xtemp = vehicle_pose_msg->x;
	    ytemp = vehicle_pose_msg->y;
	    heading_angle  = vehicle_pose_msg->theta;
	    //translate to downsampled coordinates
	    vehicle_pose.x = (int)xtemp;
	    vehicle_pose.y = (int)ytemp;

	    ROS_INFO("vehicleCallback: ( %i , %i )",vehicle_pose.x,vehicle_pose.y);
	}*/


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

		Scalar path_color = Scalar(0,255,0);
		Scalar wp_color = Scalar(50,255,0);
		Scalar goal_color = Scalar(255,0,0);

		circle(frame,vehicle_pose,LOS_RADIUS, Scalar(102, 178, 255));

	    vehicle_heading.x = (int) round(vehicle_pose.x + LOS_RADIUS * cos(heading_angle));
	    vehicle_heading.y = (int) round(vehicle_pose.y + LOS_RADIUS * sin(heading_angle));		    
	    line(frame, vehicle_pose, vehicle_heading, Scalar(0, 128, 255), HEADING_LINE_THICKNESS, 8, 0);

	    circle(frame,vehicle_pose2,LOS_RADIUS, Scalar(102, 178, 255));

	    vehicle_heading2.x = (int) round(vehicle_pose2.x + LOS_RADIUS * cos(heading_angle));
	    vehicle_heading2.y = (int) round(vehicle_pose2.y + LOS_RADIUS * sin(heading_angle));		    
	    line(frame, vehicle_pose2, vehicle_heading2, Scalar(0, 128, 255), HEADING_LINE_THICKNESS, 8, 0);
		
		int size = vector_wp.size();
		ROS_INFO("wp vector size = %i",size);
    	for (int i=0;i<size;i++)
    	{
    		circle(frame,vector_wp[i], 1, path_color,1);
    		//ROS_INFO("wp:%i,%i",vector_wp[i].x,vector_wp[i].y);

		}

		if (goal_points.size() > 0 ) 
		{
			for (int j = 0; j < goal_points.size(); j++)
			{
				circle(frame,goal_points[j], 2, goal_color,5);
			}

		}

		circle(frame,Point((int)target_wp.x, (int)target_wp.y), 2, wp_color,2);

		// draw target angle vector
		Point target_angle_endpoint;
	    target_angle_endpoint.x = (int) round(vehicle_pose.x + LOS_RADIUS * cos(target_angle));
	    target_angle_endpoint.y = (int) round(vehicle_pose.y + LOS_RADIUS * sin(target_angle));		    
	    line(frame, vehicle_pose, target_angle_endpoint, Scalar(255, 128, 0), HEADING_LINE_THICKNESS, 8, 0);

   		Point target_angle_endpoint2;
	    target_angle_endpoint2.x = (int) round(vehicle_pose2.x + LOS_RADIUS * cos(target_angle2));
	    target_angle_endpoint2.y = (int) round(vehicle_pose2.y + LOS_RADIUS * sin(target_angle2));		    
	    line(frame, vehicle_pose2, target_angle_endpoint2, Scalar(255, 128, 0), HEADING_LINE_THICKNESS, 8, 0);

	   	circle(frame, Point(160+corners1_pts[0].x/scale_factor,120+corners1_pts[0].y/scale_factor), 2, Scalar(255,0,0),1);
	   	circle(frame, Point(160+corners1_pts[1].x/scale_factor,120+corners1_pts[1].y/scale_factor), 2, Scalar(0,255,0),1);
	   	circle(frame, Point(160+corners1_pts[2].x/scale_factor,120+corners1_pts[2].y/scale_factor), 2, Scalar(0,0,255),1);
	   	circle(frame, Point(160+corners1_pts[3].x/scale_factor,120+corners1_pts[3].y/scale_factor), 2, Scalar(0,255,255),1);

	   	circle(frame, Point(160+corners2_pts[0].x/scale_factor,120+corners2_pts[0].y/scale_factor), 2, Scalar(100,0,0),1);
	   	circle(frame, Point(160+corners2_pts[1].x/scale_factor,120+corners2_pts[1].y/scale_factor), 2, Scalar(0,100,0),1);
	   	circle(frame, Point(160+corners2_pts[2].x/scale_factor,120+corners2_pts[2].y/scale_factor), 2, Scalar(0,0,100),1);
	   	circle(frame, Point(160+corners2_pts[3].x/scale_factor,120+corners2_pts[3].y/scale_factor), 2, Scalar(0,100,100),1);

	   	circle(frame, Point(160+corners2_pts_camcam[0].x/scale_factor,120+corners2_pts_camcam[0].y/scale_factor), 4, Scalar(200,0,0),1);
	   	circle(frame, Point(160+corners2_pts_camcam[1].x/scale_factor,120+corners2_pts_camcam[1].y/scale_factor), 4, Scalar(0,200,0),1);
	   	circle(frame, Point(160+corners2_pts_camcam[2].x/scale_factor,120+corners2_pts_camcam[2].y/scale_factor), 4, Scalar(0,0,200),1);
	   	circle(frame, Point(160+corners2_pts_camcam[3].x/scale_factor,120+corners2_pts_camcam[3].y/scale_factor), 4, Scalar(0,200,200),1);
/*
/*
	   	circle(frame, Point(160+corners1_pts[0].x/scale_factor-corners2_pts[0].x/scale_factor,120+corners1_pts[0].y/scale_factor - corners2_pts[0].y/scale_factor), 4, Scalar(255,0,0),1);
	   	circle(frame, Point(160+corners1_pts[1].x/scale_factor-corners2_pts[1].x/scale_factor,120+corners1_pts[1].y/scale_factor - corners2_pts[1].y/scale_factor), 4, Scalar(255,0,0),1);
	   	circle(frame, Point(160+corners1_pts[2].x/scale_factor-corners2_pts[2].x/scale_factor,120+corners1_pts[2].y/scale_factor - corners2_pts[2].y/scale_factor), 4, Scalar(255,0,0),1);
	   	circle(frame, Point(160+corners1_pts[3].x/scale_factor-corners2_pts[3].x/scale_factor,120+corners1_pts[3].y/scale_factor - corners2_pts[3].y/scale_factor), 4, Scalar(255,0,0),1);
*/
		circle(frame,vehicle_pose,2, Scalar(102, 178, 255),2);

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
	        warpPerspective(temp , temp, H_camcam, temp.size(), WARP_INVERSE_MAP | INTER_LINEAR, BORDER_CONSTANT, Scalar::all(0)); 	    
    	    cout << "tranform performed" << endl; 
	   		worldMap1.setTo(Scalar(0));
	   		temp.copyTo(worldMap1(Rect(temp.cols,temp.rows,temp.cols,temp.rows)));

	    } else { // ID_num = 1 
	   		worldMap2.setTo(Scalar(0));
    		temp.copyTo(worldMap2(Rect(temp.cols,temp.rows,temp.cols,temp.rows)));

    	}

    	// publish and reset
    	if (worldMap_tic >= 2) {
    		worldMap_tic = 0;
    		addWeighted(worldMap1,0.5,worldMap2,0.5,0.0,worldMap);
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

		}


	}

	void corners1Callback (const geometry_msgs::PoseArray::ConstPtr& corners1_msg) 
	{

	    corners1_vec.clear();
	    int size = corners1_msg->poses.size();
	    for (int i=0;i<size;i++)
	    {
	        corners1_vec.push_back(Point2f(corners1_msg->poses[i].position.x,corners1_msg->poses[i].position.y));
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

		ROS_INFO("GROUNDSTATION2 FRAME COLLECTED, size : %i x %i" , frame2.cols, frame2.rows);
		updateMap(frame2,2);

		img_bridge2 = cv_bridge::CvImage(header2,sensor_msgs::image_encodings::BGR8,frame2);
		img_bridge2.toImageMsg(img_msg2);
		pub_vis2.publish(img_msg2);

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

	Point target_wp;
	vector<Point> vector_wp;

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



