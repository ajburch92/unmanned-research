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
const int scale_factor = 4;

const string windowName = "Visualizer";

vector<Point> goal_points;
Point p;

void onMouse( int evt, int x, int y, int flags, void *param) 
{
	if (evt == CV_EVENT_LBUTTONDOWN) 
	{
		//Point*p = (Point*)ptr;
		//p->x = x;
		//p->y = y;

		vector<Point>* ptPtr = (vector<Point>*)param;
		ptPtr->push_back(Point(x,y) * scale_factor);

	} else if (evt == CV_EVENT_RBUTTONDOWN)
	{
		goal_points.clear();
	}
	ROS_INFO("left click registered");
}

//Data Processor Class ////////////////////////////

class DataProcessor
{
public:

	DataProcessor()
	{

	ros::NodeHandle nh;
	image_transport::ImageTransport it_nh(nh);
	pub_viz = it_nh.advertise("/visualization",1);
	pub_goal_out = nh.advertise<geometry_msgs::PoseArray>("/goal_pose_out",1);


	sub_rgb = it_nh.subscribe("/ground_station_rgb",1, &DataProcessor::rgbFeedCallback, this); 
	sub_target_wp = nh.subscribe("/target_wp",2, &DataProcessor::targetwpCallback,this);
	sub_vector_wp = nh.subscribe("/wp_pose",2, &DataProcessor::vectorwpCallback,this);
	sub_vector_wp = nh.subscribe("/target_angle",2, &DataProcessor::targetAngleCallback,this);
	sub_conv_fac = nh.subscribe("/conv_fac",2, &DataProcessor::convFacCallback,this);
	sub_vehicle = nh.subscribe("/vehicle_pose",20, &DataProcessor::vehicleCallback, this);
    //sub_goal = nh.subscribe("/goal_pose",20, &DataProcessor::goalCallback, this);


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

	void downsampleFrame(Mat frame) {
		scaledFrame = frame;
		downsample_factor = 4;
		Size size(322,241);
		resize(frame,scaledFrame,size);

		//get size
		scaledFrame_height = scaledFrame.rows ;
		scaledFrame_width =  scaledFrame.cols ; 

		//ROS_INFO("downsampled occupancy grid height = %i" , scaledFrame_height);
		//ROS_INFO("downsampled occupancy grid width = %i" , scaledFrame_width);

	}

	void convFacCallback (const std_msgs::Float64::ConstPtr& conv_fac_msg) 
	{
		conv_fac = conv_fac_msg -> data;
	    ROS_INFO("convFacCallback: ( %f )",conv_fac);
	}


	void targetAngleCallback (const std_msgs::Float64::ConstPtr& target_angle_msg) 
	{
		target_angle = target_angle_msg -> data;
	    ROS_INFO("targetAngleCallback: ( %f )",target_angle);
	}

	void targetwpCallback (const geometry_msgs::Pose2D::ConstPtr& target_wp_msg) 
	{
	    target_wp.x  = target_wp_msg->x;
	    target_wp.y = target_wp_msg->y;
	    ROS_INFO("targetwpCallback: ( %i , %i )",target_wp.x,target_wp.y);
	}

	void vectorwpCallback (const geometry_msgs::PoseArray::ConstPtr& vector_wp_msg) 
	{
    	ROS_INFO("vectorwpCallback");
    	vector_wp.clear();
    	int size = vector_wp_msg->poses.size();
    	for (int i=0;i<size;i++)
    	{
    		vector_wp.push_back(Point((int)vector_wp_msg->poses[i].position.x,(int)vector_wp_msg->poses[i].position.y));
		}
	}

	void vehicleCallback (const geometry_msgs::Pose2D::ConstPtr& vehicle_pose_msg) 
	{
		double xtemp, ytemp;
	    xtemp = vehicle_pose_msg->x;
	    ytemp = vehicle_pose_msg->y;
	    heading_angle  = vehicle_pose_msg->theta;
	    //translate to downsampled coordinates
	    vehicle_pose.x = (int)xtemp;
	    vehicle_pose.y = (int)ytemp;

	    ROS_INFO("vehicleCallback: ( %i , %i )",vehicle_pose.x,vehicle_pose.y);
	}



/*	void goalCallback (const geometry_msgs::Pose2D::ConstPtr& goal_pose_msg) 
	{
	    double xtemp, ytemp;
	    xtemp = goal_pose_msg->x;
	    ytemp = goal_pose_msg ->y;
	    //translate to downsampled coordinates
	    goal_pose.x  = (int)xtemp;
	    goal_pose.y = (int)ytemp;

	    ROS_INFO("goalCallback: ( %i , %i )",goal_pose.x,goal_pose.y);
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
		int size = vector_wp.size();
		ROS_INFO("wp vector size = %i",size);
		Scalar path_color = Scalar(0,255,0);
		Scalar wp_color = Scalar(255,50,0);
		Scalar goal_color = Scalar(255,0,0);

		circle(frame,vehicle_pose,LOS_RADIUS, Scalar(102, 178, 255));

	    vehicle_heading.x = (int) round(vehicle_pose.x + LOS_RADIUS * cos(heading_angle));
	    vehicle_heading.y = (int) round(vehicle_pose.y + LOS_RADIUS * sin(heading_angle));		    
	    line(frame, vehicle_pose, vehicle_heading, Scalar(0, 128, 255), HEADING_LINE_THICKNESS, 8, 0);


    	for (int i=0;i<size;i++)
    	{
    		circle(frame,vector_wp[i], 2, path_color,2);
    		//ROS_INFO("wp:%i,%i",vector_wp[i].x,vector_wp[i].y);

		}

		if (goal_points.size() > 0 ) 
		{
			for (int j = 0; j < goal_points.size(); j++)
			{
				circle(frame,goal_points[j], 2, goal_color,50);
			}

		}

		circle(frame,Point((int)target_wp.x, (int)target_wp.y), 3, wp_color,3);

		// draw target angle vector
		Point target_angle_endpoint;
	    target_angle_endpoint.x = (int) round(vehicle_pose.x + LOS_RADIUS * cos(target_angle));
	    target_angle_endpoint.y = (int) round(vehicle_pose.y + LOS_RADIUS * sin(target_angle));		    
	    line(frame, vehicle_pose, target_angle_endpoint, Scalar(255, 128, 0), HEADING_LINE_THICKNESS, 8, 0);

	}
	
	void rgbFeedCallback(const sensor_msgs::ImageConstPtr& msg)
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


   	    // draw vizualization on frame
	    drawData(frame);
	    // structure msg
		
		downsampleFrame(frame);

        imshow(windowName,scaledFrame);
        waitKey(30);

        //publish viz
	    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, scaledFrame);
	    img_bridge.toImageMsg(img_msg); // from cv _bridge to sensor_msgs::Image
	    pub_viz.publish(img_msg); 

        //publish MSSP msg 

	      counter++;
	}



private:

	// INITIALIZATION ////////////////////////////////////////////////////////////////////

	int counter = 0;
	Mat scaledFrame;
	Mat frame;

	//astar Params
	int astar_size = 150; // change to length of car

	image_transport::Subscriber sub_rgb;
	image_transport::Publisher pub_viz;
	ros::Publisher pub_goal_out;

	ros::Subscriber sub_target_wp;
	ros::Subscriber sub_vector_wp;
	ros::Subscriber sub_conv_fac;
	ros::Subscriber sub_goal;
	ros::Subscriber sub_vehicle;

	Point target_wp;
	vector<Point> vector_wp;

	double target_angle = 0;
	double heading_angle = 0;

	// vehicle location history
	Point * vehicle_pose_history = new Point[VEHICLE_POSE_HISTORY_SIZE];
	
	int vehicle_pose_history_pointer;

	double conv_fac;

	int downsample_factor = 4;
	int scaledFrame_height;
	int scaledFrame_width;

	Point vehicle_pose;
	Point goal_pose;
	Point vehicle_heading;

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



