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
// 
#include <sstream>
#include <string>
#include <iostream>
#include <vector>
#include <deque>
#include <math.h>
#include <stdio.h>
//msg file headers

#define LOS_RADIUS 100 //currently in pixels
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
		ptPtr->push_back(Point(x,y) * scale_factor);
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

	void upsampleFrame(Mat img) {
		Size size(img.cols * scale_factor , img.rows * scale_factor); // this should be 160 x 120 
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
	    target_wp.x  = target_wp_msg->x;
	    target_wp.y = target_wp_msg->y;
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
	    xtemp = vehicle_pose_msg->x;
	    ytemp = vehicle_pose_msg->y;
	    heading_angle  = vehicle_pose_msg->theta;
	    Point2f vehicle_pose_temp;
	    int ID_num = 1;
	    // transform coordinates
	    if (ID_num > 1) { // ID = 2, HbirdHcamcamOG
	                float x = H_camcam.at<double>(0,0) * xtemp + H_camcam.at<double>(0,1) * ytemp + H_camcam.at<double>(0,2);
	                float y = H_camcam.at<double>(1,0) * xtemp + H_camcam.at<double>(1,1) * ytemp + H_camcam.at<double>(1,2);
	                float w = H_camcam.at<double>(2,0) * xtemp + H_camcam.at<double>(2,1) * ytemp + H_camcam.at<double>(2,2);

	                vehicle_pose_temp=Point(x/w,y/w);

	                x = H_cambird.at<double>(0,0) * vehicle_pose_temp.x + H_cambird.at<double>(0,1) * vehicle_pose_temp.y + H_cambird.at<double>(0,2);
	                y = H_cambird.at<double>(1,0) * vehicle_pose_temp.x + H_cambird.at<double>(1,1) * vehicle_pose_temp.y + H_cambird.at<double>(1,2);
	                w = H_cambird.at<double>(2,0) * vehicle_pose_temp.x + H_cambird.at<double>(2,1) * vehicle_pose_temp.y + H_cambird.at<double>(2,2);

	                vehicle_pose_temp=Point(x/w,y/w);

	    } else { // ID_num = 0 , HbirdOG

	                float x = H_cambird.at<double>(0,0) * xtemp + H_cambird.at<double>(0,1) * ytemp + H_cambird.at<double>(0,2);
	                float y = H_cambird.at<double>(1,0) * xtemp + H_cambird.at<double>(1,1) * ytemp + H_cambird.at<double>(1,2);
	                float w = H_cambird.at<double>(2,0) * xtemp + H_cambird.at<double>(2,1) * ytemp + H_cambird.at<double>(2,2);

	                vehicle_pose_temp=Point(x/w,y/w);

	    }
	    vehicle_pose.x  = (int)vehicle_pose_temp.x;
	    vehicle_pose.y = (int)vehicle_pose_temp.y;

	    ROS_INFO("vehicleCallback: ( %i , %i )",vehicle_pose.x,vehicle_pose.y);
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
		Scalar wp_color = Scalar(255,50,0);
		Scalar goal_color = Scalar(255,0,0);

		circle(frame,vehicle_pose,LOS_RADIUS, Scalar(102, 178, 255));

	    vehicle_heading.x = (int) round(vehicle_pose.x + LOS_RADIUS * cos(heading_angle));
	    vehicle_heading.y = (int) round(vehicle_pose.y + LOS_RADIUS * sin(heading_angle));		    
	    line(frame, vehicle_pose, vehicle_heading, Scalar(0, 128, 255), HEADING_LINE_THICKNESS, 8, 0);
		
		int size = vector_wp.size();
		ROS_INFO("wp vector size = %i",size);
    	for (int i=0;i<size;i++)
    	{
    		circle(frame,vector_wp[i], 2, path_color,2);
    		//ROS_INFO("wp:%i,%i",vector_wp[i].x,vector_wp[i].y);

		}

		if (goal_points.size() > 0 ) 
		{
			for (int j = 0; j < goal_points.size(); j++)
			{
				circle(frame,goal_points[j], 2, goal_color,10);
			}

		}

		circle(frame,Point((int)target_wp.x, (int)target_wp.y), 3, wp_color,3);

		// draw target angle vector
		Point target_angle_endpoint;
	    target_angle_endpoint.x = (int) round(vehicle_pose.x + LOS_RADIUS * cos(target_angle));
	    target_angle_endpoint.y = (int) round(vehicle_pose.y + LOS_RADIUS * sin(target_angle));		    
	    line(frame, vehicle_pose, target_angle_endpoint, Scalar(255, 128, 0), HEADING_LINE_THICKNESS, 8, 0);

	}

	void getPerspectives() {
	    corner_tic++;
	    if (corner_tic >= 2) { //only after both messages are received. 
	        corner_tic = 0;

	        // calc camcam H mat
	        H_camcam = getPerspectiveTransform(corners1_pts,corners2_pts);
   			ROS_INFO("camcam");

	        cout << H_camcam << endl;
	    }
	        
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
	    H_cambird = getPerspectiveTransform(undistorted_pts, corners1_pts);
		H_cambird.at<double>(2,2) = 10;

		ROS_INFO("cambird");

	    cout << H_cambird << endl;

	}

	void updateMap(Mat temp, int ID) {
    	Mat worldMaptemp(temp.rows*3,temp.cols*3,temp.type(),Scalar(0));
    	if (worldMap_tic < 1) { //zero out
    		worldMaptemp.copyTo(worldMap);
    	}
	   	worldMap_tic++;;
    	//else, for 2 count, add images from ID 1 and 2
	    if (ID > 1) { // ID = 2, HbirdHcamcamOG
	        //warpPerspective(temp , temp, H_camcam, temp.size(), WARP_INVERSE_MAP | INTER_LINEAR, BORDER_CONSTANT, Scalar::all(0));
    	    //warpPerspective(temp , temp, H_cambird, temp.size(), WARP_INVERSE_MAP | INTER_LINEAR, BORDER_CONSTANT, Scalar::all(0));
    		temp.copyTo(worldMaptemp(Rect(160+(corners1_pts[0].x/scale_factor),120+(corners1_pts[0].y/scale_factor),160,120)));
    		addWeighted(worldMap,0.5,worldMaptemp,0.5,0.0,worldMap);
    		//worldMap = worldMaptemp + worldMap;
    		//worldMaptemp.copyTo(worldMap);

	    } else { // ID_num = 1 , HbirdOG
	    	//upsampleFrame(temp);
            warpPerspective(temp , temp, H_cambird, temp.size(), WARP_INVERSE_MAP | INTER_LINEAR, BORDER_CONSTANT, Scalar::all(255));
			//downsampleFrame(temp);

    		temp.copyTo(worldMaptemp(Rect(temp.cols,temp.rows,temp.cols,temp.rows)));
            //warpPerspective(worldMaptemp , worldMaptemp, H_cambird, worldMaptemp.size(), WARP_INVERSE_MAP | INTER_LINEAR, BORDER_CONSTANT, Scalar::all(255));

    		addWeighted(worldMap,0.5,worldMaptemp,0.5,0.0,worldMap);
//    		worldMap = worldMaptemp + worldMap;

    		//worldMaptemp.copyTo(worldMap);
    	}

    	// publish and reset
    	if (worldMap_tic >= 2) {
    		worldMap_tic = 0;
		    drawData(worldMap);
	    	imshow(windowName,worldMap);

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
	        corners1_pts[i] = corners1_vec[i];    
	    }
        cout << corners2_vec << endl;

	    getPerspectives();

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

	    //warpPerspective(frame , frame, H_cambird, frame.size(), WARP_INVERSE_MAP | INTER_LINEAR, BORDER_CONSTANT, Scalar::all(0));
	
	    // update mouseclick events
	    if (goal_points.size() > 0 ) 
		{
	    	updateGoal();
		}

		updateMap(frame,1);

		//downsampleFrame(frame);


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
			ROS_INFO("checkpoint 1");

		}
		else {key_cmd_msg.data = 0;}

		key_cmd_pub.publish(key_cmd_msg);

        //publish MSSP msg 

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
		catch (cv_bridge::Exception& e)
		{
			ROS_INFO("cv_bridge exception: %s", e.what());
			return;
		}

	    std_msgs::Header header2; //empty header
	    header2.seq = counter2; // user defined counter
	    header2.stamp = ros::Time::now(); // time
	    frame2 = cv_ptr2 -> image;

		//warpPerspective(frame2 , frame2, H_camcam, frame.size(), WARP_INVERSE_MAP | INTER_LINEAR, BORDER_CONSTANT, Scalar::all(0));
	    //warpPerspective(frame2 , frame2, H_cambird, frame2.size(), WARP_INVERSE_MAP | INTER_LINEAR, BORDER_CONSTANT, Scalar::all(0));

		ROS_INFO("GROUNDSTATION2 FRAME COLLECTED, size : %i x %i" , frame2.cols, frame2.rows);
		updateMap(frame2,2);

		//downsampleFrame2(frame2);

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
	double heading_angle = 0;

	// vehicle location history
	Point * vehicle_pose_history = new Point[VEHICLE_POSE_HISTORY_SIZE];
	
	int vehicle_pose_history_pointer;

	double conv_fac;

	int scaledFrame_height;
	int scaledFrame_width;

	Point vehicle_pose;
	Point goal_pose;
	Point vehicle_heading;

	vector<Point2f> corners2_vec;
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



