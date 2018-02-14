//
// ground_station_rgb.cpp
// Austin Burch
// 
// This file initiates the rgb_node and launches the rgbImageProcessor.
//

// ros
#include <ros/ros.h>
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
//object class
#include "Object.h"
//msg file headers
#define LOS_RADIUS 100 //currently in pixels
#define HEADING_LINE_THICKNESS 2
#define VEHICLE_POSE_HISTORY_SIZE 150

using namespace std;
using namespace cv;

// RGB Image Processor Class ////////////////////////////

class RGBImageProcessor
{
public:

	RGBImageProcessor()
	{

		ros::NodeHandle nh_rgb;
		image_transport::ImageTransport it_rgb(nh_rgb);
		ros::NodeHandle nh_rgbp("~");


		// store launch params
		//nh_rgb.param("checkerboard_width", checkerboard_width, -1);
		//nh_rgb.param("checkerboard_height", checkerboard_height, -1);

		nh_rgbp.param("ID_num",ID_num,0);
		//ID_num = 1;
		checkerboard_height = 0.3 ; // meters; or get square size, and multiply 
		checkerboard_width = 0.55 ; // meters


		stringstream ss;
		ss << ID_num;
		string s;
		s = ss.str();

		string image_rect_color = "/camera" + s + "/image_rect_color" ;
		string ground_station_rgb = "/ground_station_rgb" + s ; 
		string conv_fac = "/conv_fac" + s ;
		string occupancyGridLow = "/occupancyGridLow" + s ;
		string occupancyGridHigh = "/occupancyGridHigh" + s ;
		string vehicle_pose = "/vehicle_pose" + s ;
		string arm_bool = "/arm_bool" + s ;
		string confidence = "/confidence" + s ;
		string corners_pose = "/corners" + s ;

		rgb_sub_ = it_rgb.subscribe(image_rect_color,1, &RGBImageProcessor::rgbFeedCallback, this); // use image_rect
		rgb_pub_ = it_rgb.advertise(ground_station_rgb,1);
		conv_fac_pub = nh_rgb.advertise<std_msgs::Float64>(conv_fac,2);
		occupancyGridLow_pub = it_rgb.advertise(occupancyGridLow , 1);
		occupancyGridHigh_pub = it_rgb.advertise(occupancyGridHigh , 1);
		rgb_vehicle_pub = nh_rgb.advertise<geometry_msgs::Pose2D>(vehicle_pose,2);
		corners_pub = nh_rgb.advertise<geometry_msgs::PoseArray>(corners_pose,2);
		//rgb_goal_pub = nh_rgb.advertise<geometry_msgs::Pose2D>("goal_pose",2);
		rgb_arm_bool_pub = nh_rgb.advertise<std_msgs::Float64>(arm_bool,2);
		rgb_confidence_pub = nh_rgb.advertise<std_msgs::Float64>(confidence,2);
		key_cmd_sub = nh_rgb.subscribe("/key_cmd" , 2 , &RGBImageProcessor::keyCallback, this);
	//	sub_target_angle = nh_rgb.subscribe("/target_angle",2, &RGBImageProcessor::targetAngleCallback,this);*/

		//create background subtractor object

		pMOG = new BackgroundSubtractorMOG();

		// Set each element in history to 0
		for (int i = 0; i < VEHICLE_POSE_HISTORY_SIZE; i++) {
		    vehicle_pose_history[i] = Point(0, 0);
		}

		Mat cam_intrinsic_mat = (Mat_<double>(3,3) << 
			1257.9354531133 , 0.0 , 711.8948570519622,
			0.0, 1258.482033400394, 459.2775748027645,
			0.0, 0.0, 1.0);
		Mat cam_dist_vec = (Mat_<double>(1,5) <<
			-0.09831822598667773, 0.1015705125684106, -0.002485105904864273, 0.01055372647729187, 0.0);

	}

	~RGBImageProcessor()
	{
    //cv::destroyWindow(); // all
	}

	string intToString(int number){ // for writing txt on frame
		std::stringstream ss;
		ss << number;
		return ss.str();
	}


	void keyCallback (const std_msgs::Int8::ConstPtr& key_cmd_msg) 
	{
		key_cmd = key_cmd_msg -> data;
	    ROS_INFO("key_cmd: ( %i )",key_cmd);
		switch (key_cmd)
		{			
			case 1 :
				if (tracking_status == FALSE)
				{
					tracking_status = TRUE;
				} else {tracking_status = FALSE;}
			    	ROS_INFO ("case 1 key_cmd : tracking status toggled");

			  break;

			case 2 :

				pMOG = new BackgroundSubtractorMOG();
				patternfound = 0;
				counter = 0;
				ROS_INFO ("case 2 key_cmd : background reinitialized");
				break;

			default : 
				break;
		}
	}


	void createHSVTrackbars(){

		namedWindow(windowName7,0);
		char TrackbarName[50];

		sprintf( TrackbarName, "H_MIN", H_MIN);
		sprintf( TrackbarName, "H_MAX", H_MAX);
		sprintf( TrackbarName, "S_MIN", S_MIN);
		sprintf( TrackbarName, "S_MAX", S_MAX);
		sprintf( TrackbarName, "V_MIN", V_MIN);
		sprintf( TrackbarName, "V_MAX", V_MAX);

		createTrackbar( "H_MIN", windowName7, &H_MIN, H_MAX);
		createTrackbar( "H_MAX", windowName7, &H_MAX, H_MAX);
		createTrackbar( "S_MIN", windowName7, &S_MIN, S_MAX);
		createTrackbar( "S_MAX", windowName7, &S_MAX, S_MAX);
		createTrackbar( "V_MIN", windowName7, &V_MIN, V_MAX);
		createTrackbar( "V_MAX", windowName7, &V_MAX, V_MAX);
	}

	void checkerboardAnalysis() {
		
		// send checkerboard coordinates, conversion factor

		//checkerboard_PXwidth = ((transImgPts[1] - transImgPts[0]) + (transImgPts[1] - transImgPts[0]))/2; 
		//int widthDif =  (transImgPts[1] - transImgPts[0]) - (transImgPts[1] - transImgPts[0]);

		//checkerboard_PXheight = ((transImgPts[0] - transImgPts[2]) + (transImgPts[1] - transImgPts[3]))/2;
		//int heightDif =  (transImgPts[1] - transImgPts[0]) - (transImgPts[1] - transImgPts[0]);

		//ROS_INFO("heightDif = %i" , heightDif);
		//ROS_INFO("widthDif = %i" , widthDif); 
			
		height_factor = checkerboard_height / double(checkerboard_PXheight) ;
		width_factor =  checkerboard_width / double(checkerboard_PXwidth) ; // convert to Meters

		std_msgs::Float64 conv_fac_msg;
	    conv_fac_msg.data = (width_factor + height_factor) / 2;
	    conv_fac_pub.publish(conv_fac_msg);

		ROS_INFO("height_factor = %f" , height_factor);
	    ROS_INFO("width_factor = %f" , width_factor); 
	}

	void downsampleFrameOut(Mat frame) {
		frameoutDown = frame;
		scale_factor = 8; // this may not work for all resolutions. this value needs to match the value in astar. retrive from param launch file.
		Size size(resizedFeed.cols / scale_factor , resizedFeed.rows / scale_factor); // this should be 160x120 
		resize(frame,frameoutDown,size);
	}

	void downsampleGrid(Mat grid) {
		gridDownHigh = grid;
		scale_factor = 8; // this may not work for all resolutions. this value needs to match the value in astar. retrive from param launch file.
		Size downsizeHigh(resizedFeed.cols / scale_factor , resizedFeed.rows / scale_factor); // this should be 160x120 
		resize(grid,gridDownHigh,downsizeHigh);
		//pyrDown( grid, gridDown, Size( grid.cols/scale_factor, grid.rows/scale_factor ) );
		
		gridDownLow = gridDownHigh;
		Size downsizeLow(gridDownHigh.cols / scale_factor , gridDownHigh.rows / scale_factor); // this should be 20x15 
		resize(gridDownHigh,gridDownLow,downsizeLow);
		//pyrDown( grid, gridDown, Size( grid.cols/scale_factor, grid.rows/scale_factor ) );

		//get size
		gridDownLow_height = gridDownLow.rows ;
		gridDownLow_width =  gridDownLow.cols ; 

		//get size
		gridDownHigh_height = gridDownHigh.rows ;
		gridDownHigh_width =  gridDownHigh.cols ; 

//		ROS_INFO("downsampled occupancy grid height = %i" , gridDownHigh_height);
//		ROS_INFO("downsampled occupancy grid width = %i" , gridDownHigh_width);
//		ROS_INFO("downsampled occupancy grid height = %i" , gridDownLow_height);
//		ROS_INFO("downsampled occupancy grid width = %i" , gridDownLow_width);
	}

	void resizeFrame(Mat frame) {
		resizedFeed = frame;
		Size size(1280,960); // original resolution : 1288x964
		resize(frame,resizedFeed,size);
		ROS_INFO("resizedFeed : %i x %i" , resizedFeed.cols, resizedFeed.rows);
	}

	void drawObject(vector<Object> theObjects, Mat &frame, vector< vector<Point> > contours, vector<Vec4i> hierarchy)
	{ // draw on frame
		int n, iter;
		if (counter < MEMORY_SIZE) {n = counter;} else {n = MEMORY_SIZE;}
		//draw objects
		Scalar white = Scalar(255,255,255);
		if (contours.size() > 0) {
			drawContours(frame,contours,-1,white,2,8,hierarchy);
		}


		for(int i =0; i<theObjects.size(); i++)
	  	{ //for each object
	    //draw current position
			try {
				drawContours(frame,contours,i,theObjects.at(i).getColor(),2,8,hierarchy);
			} catch (Exception& e) {}
	  		putText(frame,intToString(theObjects.at(i).getXPos(MEMORY_SIZE-1))+ " , " + intToString(theObjects.at(i).getYPos(MEMORY_SIZE-1)),cv::Point(theObjects.at(i).getXPos(MEMORY_SIZE-1),theObjects.at(i).getYPos(MEMORY_SIZE-1)+20),1,1,Scalar(0,255,0));
	  		putText(frame,theObjects.at(i).getType() + ": " + intToString(i+1),Point(theObjects.at(i).getXPos(MEMORY_SIZE-1),theObjects.at(i).getYPos(MEMORY_SIZE-1)-20),1,1.5,theObjects.at(i).getColor(),2);
	    	//draw past positions if tracking
	  		// if (tracking_status == TRUE) {
	  		// 	for (int j = 1; j<(n-1); j++) { 
	  		// 		//line(frame,Point(theObjects.at(i).getXPos(MEMORY_SIZE-j),theObjects.at(i).getYPos(MEMORY_SIZE - j)),Point(theObjects.at(i).getXPos(MEMORY_SIZE-(j-1)),theObjects.at(i).getYPos(MEMORY_SIZE - (j-1))),theObjects.at(i).getColor(),1);
	  		// 	}
	  		// }
	    }
	}

	int largestObject( vector<vector<Point> > contours)
	{
		int largest_area = 0;
		int largest_contour_index=0;
		for (size_t i=0; i<contours.size(); i++ ) 
		{
			double area = contourArea(contours[i]);
			if ( area < largest_area) 
			{
				largest_area = area;
				largest_contour_index = i;
			}
		}

		return largest_contour_index;

	}

	void drawAxis(Mat& img, Point p, Point q, Scalar colour, const float scale = 80) 
	{
	    double angle;
	    double hypotenuse;
	    angle = atan2((double) p.y - q.y, (double) p.x - q.x); // angle in radians
	    hypotenuse = sqrt((double) (p.y - q.y) * (p.y - q.y) + (p.x - q.x) * (p.x - q.x));
	    //    double degrees = angle * 180 / CV_PI; // convert radians to degrees (0-180 range)
	    //    cout << "Degrees: " << abs(degrees - 180) << endl; // angle in 0-360 degrees range
	    // Here we lengthen the arrow by a factor of scale
	    q.x = (int) (p.x - scale * hypotenuse * cos(angle));
	    q.y = (int) (p.y - scale * hypotenuse * sin(angle));
	    line(img, p, q, colour, 2, CV_AA);
	    // create the arrow hooks
	    p.x = (int) (q.x + 9 * cos(angle + CV_PI / 4));
	    p.y = (int) (q.y + 9 * sin(angle + CV_PI / 4));
	    line(img, p, q, colour, 1, CV_AA);
	    p.x = (int) (q.x + 9 * cos(angle - CV_PI / 4));
	    p.y = (int) (q.y + 9 * sin(angle - CV_PI / 4));
	    line(img, p, q, colour, 2, CV_AA);
	}

	void pcaOrientation(const vector<Point> &pts, Mat &objectFeed)
	{
		//Construct a buffer used by the pca analysis
	    int sz = static_cast<int> (pts.size());
	    Mat data_pts = Mat(sz, 2, CV_64FC1);
	    for (int i = 0; i < data_pts.rows; ++i) {
	        data_pts.at<double>(i, 0) = pts[i].x;
	        data_pts.at<double>(i, 1) = pts[i].y;
	    }
	    //Perform PCA analysis
	    PCA pca_analysis(data_pts, Mat(), CV_PCA_DATA_AS_ROW);
	    //Store the center of the object
	    Point cntr = Point(static_cast<int> (pca_analysis.mean.at<double>(0, 0)),
	            static_cast<int> (pca_analysis.mean.at<double>(0, 1)));
	    //Store the eigenvalues and eigenvectors
	    vector<Point2d> eigen_vecs(2);
	    vector<double> eigen_val(2);
	    for (int i = 0; i < 2; ++i) {
	        eigen_vecs[i] = Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
	                pca_analysis.eigenvectors.at<double>(i, 1));
	        eigen_val[i] = pca_analysis.eigenvalues.at<double>(0, i);
	    }
	    // Draw the principal components
	    circle(objectFeed, cntr, 3, Scalar(255, 0, 255), 2);
	    Point p1 = cntr + 0.02 * Point(static_cast<int> (eigen_vecs[0].x * eigen_val[0]), static_cast<int> (eigen_vecs[0].y * eigen_val[0]));
	    Point p2 = cntr - 0.02 * Point(static_cast<int> (eigen_vecs[1].x * eigen_val[1]), static_cast<int> (eigen_vecs[1].y * eigen_val[1]));
	    drawAxis(objectFeed, cntr, p1, Scalar(0, 255, 0), 1);
	    drawAxis(objectFeed, cntr, p2, Scalar(255, 255, 0), 5);
	    pca_vehicle_orientation_angle = atan2(eigen_vecs[0].y, eigen_vecs[0].x); // orientation in radians

	}

	void update_pose_history(){
        // Save current location to history
        vehicle_pose_history[vehicle_pose_history_pointer] = vehicle_pose;
        
        // Update circular array pointer
        vehicle_pose_history_pointer = (vehicle_pose_history_pointer + 1) % VEHICLE_POSE_HISTORY_SIZE;
	}

	void headingFilter() {
		double angle_diff = pca_vehicle_orientation_angle - poly_vehicle_orientation_angle;
		//vehicle_orientation_angle = 1.0*pca_vehicle_orientation_angle + 0.0*poly_vehicle_orientation_angle;
		double angle_change = vehicle_orientation_angle - prev_vehicle_orientation_angle;
		if (angle_change > 1.5 && abs(vehicle_orientation_angle-0.016)<0.015) 
		{
			vehicle_orientation_angle = prev_vehicle_orientation_angle;
			ROS_INFO("prev angle selected");
		} else {vehicle_orientation_angle = poly_vehicle_orientation_angle;}
		ROS_INFO("headingFilter: angle diff = %f, angle_change = %f, vehicle_orientation_angle = %f",angle_diff, angle_change,vehicle_orientation_angle);
	}

	double polyOrientation(Mat &objectFeed)
	{
    
	    // clear vector
		pose_poly.clear();
	    
	    // Sort vehicle location history chronologically
	    Point * vehicle_pose_history_sorted = new Point[VEHICLE_POSE_HISTORY_SIZE];
	    
	    // Initialize input vector (approxPolyDP takes only vectors and not arrays)
	    vector<Point> input_points;
	    
	    for (int i = 0; i < VEHICLE_POSE_HISTORY_SIZE; i++) {
	        vehicle_pose_history_sorted[i] = vehicle_pose_history[(vehicle_pose_history_pointer + i) % VEHICLE_POSE_HISTORY_SIZE];
	        input_points.push_back(vehicle_pose_history_sorted[i]);
	    }
	    
	    // Approximate location history with a polynomial curve
	    approxPolyDP(input_points, pose_poly, 3, false);

	    // Draw polynomial curve
	    for (int i = 0; i < pose_poly.size() - 1; i++) {
	    	if (pose_poly[i].x != 0 && pose_poly[i].y != 0) {
	    	    line(objectFeed, pose_poly[i], pose_poly[i + 1], Scalar(102, 178, 255), HEADING_LINE_THICKNESS, CV_AA);
	    	}
	    }
	    

	    int delta_x = pose_poly[pose_poly.size() - 1].x - pose_poly[pose_poly.size() - 2].x;
	    
	    int delta_y = pose_poly[pose_poly.size() - 1].y - pose_poly[pose_poly.size() - 2].y;
	    
	    // Angle in degrees
	    double vehicle_angle_polynomial_approximation = atan2(delta_y, delta_x) * (180 / M_PI);

	    // Compute heading point
	    Point heading_point_polynomial_approximation;
	    heading_point_polynomial_approximation.x = (int) round(pose_poly[pose_poly.size() - 1].x + LOS_RADIUS * cos(vehicle_angle_polynomial_approximation * CV_PI / 180.0));
	    heading_point_polynomial_approximation.y = (int) round(pose_poly[pose_poly.size() - 1].y + LOS_RADIUS * sin(vehicle_angle_polynomial_approximation * CV_PI / 180.0));
	    
	    // Draw line between current location and heading point
	    //line(objectFeed, vehicle_pose, heading_point_polynomial_approximation, Scalar(0, 128, 255), HEADING_LINE_THICKNESS, 8, 0);
	    
	    // Use curve polynomial tangent angle
	    poly_vehicle_orientation_angle = vehicle_angle_polynomial_approximation;

	    // look at recent distance travel
	    double recent_dist_travel=0 ;

	    for (int i = 0; i < 3; i++) {
	    	recent_dist_travel = recent_dist_travel + (sqrt(((input_points[i+1].x - input_points[i].x)*(input_points[i+1].x - input_points[i].x)) 
	    		+ ((input_points[i+1].y - input_points[i].y)*(input_points[i+1].y - input_points[i].y))));
	    }
		ROS_INFO("recent_dist_travel = %f",recent_dist_travel);

	    return recent_dist_travel;

	} 

	void morphologicalOps (Mat &thresh, int erodeS, int dilateS) 
	{
		GaussianBlur(thresh,thresh,Size(blurSize,blurSize),sigmaSize,sigmaSize);

		threshold(thresh, thresh, lowThreshold, 255, cv::THRESH_BINARY);

	    erodeElement = getStructuringElement( MORPH_RECT,Size(erodeS+1,erodeS+1)); //erode with 3x3 px rectangle
	    erode(thresh,thresh,erodeElement);

	    dilateElement = getStructuringElement( MORPH_RECT,Size(dilateS+1,dilateS+1)); //dilate with larger element so make sure object is nicely visible
	    dilate(thresh,thresh,dilateElement);

	}

	void detectObjects(Mat threshold, Mat &frame, string name) { // object detection 
	  //generate temporary vectors
		vector< vector<Point> > contours_temp; 
		vector<Vec4i> hierarchy_temp;          
		vector <Object> objects_temp; 
		double max_contour_index;
		Mat temp;

	  // cp to temporary
		threshold.copyTo(temp);

	  // detect contours
		findContours(temp,contours_temp,hierarchy_temp,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );

	  //use moments method to find filtered object
		bool objectFound = false;

	  if (hierarchy_temp.size() > 0) { // if contours were found
	  	int numObjects = hierarchy_temp.size(); 
	    //if number of objects greater than MAX_NUM_OBJECTS, then image theshold is noisy
	  	if(numObjects<MAX_NUM_OBJECTS)
	  	{
	      // find largest object
	  		int index = largestObject(contours_temp); 
	  		max_contour_index = double(index);
	      //for (int index = 0; index >= 0; index = hierarchy_temp[index][0]) //for each object
	      //{
	        Moments moment = moments((cv::Mat)contours_temp[index]); //moments method
	        double area = moment.m00;

	        if(area>MIN_OBJECT_AREA)
	        {


	          Object object(name); // generate new instance

	          object.setXPos(moment.m10/area); // find x
	          object.setYPos(moment.m01/area); // y

	          objects_temp.push_back(object); 

	          objectFound = true;

	        }
	        else { 
	        	objectFound = false; 
	        }
			if(objectFound ==true)
			{
			    //draw object on frame
			   drawObject(objects_temp,frame,contours_temp,hierarchy_temp);
			}
		}
	    else {
			putText(frame,"NOISE! ADJUST FILTER",Point(0,50),1,2,Scalar(0,0,255),2); 
	    }
	  }

	  //save temporary vectors as current        
	  if (name=="goal") {
		objects_blue = objects_temp;
	  }
	  else if (name=="green") {
		objects_green = objects_temp;
	  }
	  else if (name=="vehicle") {
		objects_yellow = objects_temp;
	  }
	  else if (name=="red") {
		objects_red = objects_temp;
	  }

	  //pcaOrientation(contours[max_contour_index],frame);
      //clear temporary vectors
	  contours_temp.clear();
	  hierarchy_temp.clear();
	  objects_temp.clear();
	}

	void overlapState(int x, int y) {
		
		int xmax, xmin, ymax, ymin;

		xmax = 10;
		xmin = 1;
		ymax = 10;
		ymin = 1;

		int dxmax = xmax - x ; 
		int dxmin = xmin - x ; 

		int dymax = ymax - y ; 
		int dymin = ymin - y ; 
		if (dxmax > 0 && dxmin < 0 && dymax > 0 && dymin < 0) {
			overlap_state = 1;
		} else { overlap_state = 0;}
	}

	void trackObjects(Mat threshold, Mat &frame,vector<Object> objects, string name) { //object tracking

		Mat temp;
		threshold.copyTo(temp);
		findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );

		bool objectFound = false;
	  	int x_obj, y_obj, x_temp, y_temp, num_objects;
		float dist_temp, xdot_obj, ydot_obj;
		int minPos = 0;
	  	float max_dist = 1000; //pixels
	  	vector<float> dist;
	  	double x,y,th;
	  	double max_contour_index;
	  	double confidence_temp = 0; 

	  	if (hierarchy.size() > 0) { // if contours are found
	  		
	  		num_objects = hierarchy.size();

	  		//find largest object
	  		int index = largestObject(contours); 
	  		max_contour_index = double(index);

	        //for (int index = 0; index < contours.size(); index++) { // for all objects
			Moments moment = moments((cv::Mat)contours[index]); //moments method
			double area = moment.m00;

			//find pose
			x_temp = (int)moment.m10/area;       
			y_temp = (int)moment.m01/area; 

			//compare distances to past tracked objects
			for (int i = 0; i<objects.size();i++) {
				x_obj = (float)objects.at(i).getXPos(MEMORY_SIZE-1);
				y_obj = (float)objects.at(i).getYPos(MEMORY_SIZE-1);
				dist_temp = sqrt(pow((x_obj - (float)x_temp),2.0) + pow((y_obj - (float)y_temp),2.0));
				dist.push_back(dist_temp);
			}

          	// find object with min dist
          	for (int j = 0 ; j<dist.size();j++)
          	{
          		if (dist[j] < dist[minPos])
          		{
          			minPos = j;
        	  	}
	        }

          	// if distance is in neighborhood, assign xy coordinates
          	if (dist[minPos] < max_dist)  // VEHICLE FOUND 
          	{

          		// Set vehicle position
               	x_obj = x_temp;
               	y_obj = y_temp;

          		objects.at(minPos).setXPos(x_obj);
          		objects.at(minPos).setYPos(y_obj);

          		// Check where vehicle is in frame / world
          		overlapState(x_obj, y_obj);

          		if (overlap_state > 0) { // is in overlapping region

          			if (waypoint_proj_perc < 0.50) { // not heading towards other GS FOV
		  				local_bool = 1;
          				// calculate confidence
			  			confidence = 0.9;		

          			} else {
          				if (trajectory_state >= 3) { // trajectory is ready to handoff			
							local_bool = 0;
          					// calc confidence
				  			confidence = 0.2;

          				} else { // remote trajectory not ready

          					local_bool = 1;
          					// calculate confidence
				  			confidence = 0.8;

          				}
          			}
          		} else { // VEHICLE FOUND : overlap_state = 0, not in an overlapping area

		  			local_bool = 1;
          			// calc confidence
		  			confidence = 1;
                               
          		}

          	} else { // VEHICLE NOT FOUND : object too far away 
          		
          		if (projection_state <= 3) { //project vehicle position (or use past position), for up to three frames, then timeout.
	          		int i=0;
	          		x_obj = objects.at(i).getXPos(MEMORY_SIZE-1); // retrieve past object position.
			  		y_obj = objects.at(i).getYPos(MEMORY_SIZE-1);

			  		objects.at(i).setXPos(x_obj);
			  		objects.at(i).setYPos(y_obj);

		  			local_bool = 1;
			  		// calc confidence
		  			confidence = 0.1;

		  		} else {

		  			local_bool = 0;
          			// calc confidence
		  			confidence = 0;
				}
          	}

	        dist.clear();
	        minPos = 0;
	 
	        drawObject(objects,frame, contours,hierarchy);

		} else { // no contours found
			num_objects = 0;
			// use guess at object travel as opposed to vision, otherwise the position index will remain zero, or last memory cycle value. 
			for (int j = 0; j<objects.size(); j++) { //for each already existing object.
				if (projection_state <= 3) { // VEHICLE FOUND : project vehicle position (or use past position), for up to three frames, then timeout.
					int i=0;
					x_obj = objects.at(i).getXPos(MEMORY_SIZE-1); // retrieve past object position.
					y_obj = objects.at(i).getYPos(MEMORY_SIZE-1);

					objects.at(i).setXPos(x_obj);
					objects.at(i).setYPos(y_obj);

					local_bool = 1;
					// calc confidence

				} else { // VEHICLE NOT FOUND : vehicle projection timed out

					local_bool = 0;
					// calc confidence
		  			confidence = 0;

				}
			}		  	  
		}
		
		x = double(x_obj);
		y = double(y_obj);

		if (local_bool > 0) { // vehicle_detected or projected, send pose.

			if (name=="vehicle") { // yellow vehicle state

				vehicle_pose = Point(x, y);
		
				geometry_msgs::Pose2D vehicle_pose_msg;

				vehicle_pose_msg.x = x;
				vehicle_pose_msg.y = y;
				
				double recent_dist_travel = polyOrientation(frame);
				if (recent_dist_travel < 10) 
				{
				} else { vehicle_orientation_angle = poly_vehicle_orientation_angle;
					pcaOrientation(contours[max_contour_index],frame);
					headingFilter();
				}
		
				vehicle_pose_msg.theta = vehicle_orientation_angle;
				prev_vehicle_orientation_angle = vehicle_orientation_angle;
				
				vehicle_pose_msg.theta = vehicle_pose_msg.theta*CV_PI/180;
				th = vehicle_pose_msg.theta;

				ROS_INFO("vehicle pose: ( %f , %f ) : th = %f ",x,y,th);
				rgb_vehicle_pub.publish(vehicle_pose_msg); 
			}
			else if (name=="goal") { // blue goal state
				ROS_INFO("goal pose: ( %i , %i ) ",x_obj,y_obj);
			} else {};
		}

	}

	
	void rgbFeedCallback(const sensor_msgs::ImageConstPtr& msg)
	{
		cv_bridge::CvImage img_bridge;
		cv_bridge::CvImage occupancyHigh_bridge;
		cv_bridge::CvImage occupancyLow_bridge;

		sensor_msgs::Image img_msg;
		sensor_msgs::Image occupancyGridLow_msg;
		sensor_msgs::Image occupancyGridHigh_msg;

		cv_bridge::CvImagePtr rgb_cv_ptr;
		int b;
		Size patternsz(8,6);
		std::vector<Point2f> imgPts_vec;
		std::vector<Point2f> objPts_vec(4);
		std::vector<Point2f> transImgPts_vec(4);

		
		try
		{
			rgb_cv_ptr  = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_INFO("cv_bridge exception: %s", e.what());
			return;
		}

	    std_msgs::Header header; //empty header
	    header.seq = counter; // user defined counter
	    header.stamp = ros::Time::now(); // time
	    cameraFeed = rgb_cv_ptr -> image;
	    
	    resizeFrame(cameraFeed);


	    if (patternfound < 1) 
	    {
			Size patternsize(8,6);
	    	patternfound = findChessboardCorners(resizedFeed , patternsize,corners);  
	    	putText(resizedFeed ,"LOOKING FOR CHECKERBOARD",Point(0,50),1,2,Scalar(0,0,255),2); 
	    	
	    	// Output resized video stream
	    	downsampleFrameOut(resizedFeed);

			img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, frameoutDown);
			img_bridge.toImageMsg(img_msg); // from cv _bridge to sensor_msgs::Image
			rgb_pub_.publish(img_msg); 
			if (patternfound < 1) {
				ROS_INFO("pattern not found");
			} else {ROS_INFO("pattern found");}
	    } 
	    else { // pattern already found
	    	if (counter < 1) 
	    	{
	    		cvtColor(resizedFeed , grayFeed, COLOR_BGR2GRAY);
	    		cornerSubPix(grayFeed, corners, Size(11,11),Size(-1,-1), TermCriteria( cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.1));

	    		imgPts[0]=corners[0];
	    		imgPts[1]=corners[board_w-1];
	    		imgPts[2]=corners[(board_h-1)*board_w];
	    		imgPts[3]=corners[(board_h-1)*board_w+board_w-1];

	    		geometry_msgs::PoseArray poseArray;
    			poseArray.poses.clear();
    			poseArray.header.stamp=ros::Time::now();
    			geometry_msgs::Pose corners_pose_msg;

	   			for (int n=0;n<=3; n++) {
		            corners_pose_msg.position.x = (imgPts[n].x);
		            corners_pose_msg.position.y = (imgPts[n].y);
		            poseArray.poses.push_back(corners_pose_msg);     
           	        cout << imgPts[n] << endl;
				
    			}

				corners_pub.publish(poseArray); 
				ROS_INFO("corners published");


	    		objPts[0].x=0;
	    		objPts[1].x=board_w-1;
	    		objPts[2].x=0;
	    		objPts[3].x=board_w-1;
	    		objPts[0].y=0;
	    		objPts[1].y=0;
	    		objPts[2].y=board_h-1;
	    		objPts[3].y=board_h-1;
	    		// scale these accordingly
/*
	    		//get gs2 transformation matrix
	    		if(ID_num == 1) {
		    		Mat H_camcam = getPerspectiveTransform(imgRemote,imgLocal)
		    		Mat H_cambird = getPerspectiveTransform(imgPts, objPts);
	    			H_camcam.at<double>(2,2) = birdseyeHeight;
	    			H_cambird.at<double>(2,2) = birdseyeHeight;

		    		//publish transform matrix
		    		//apply transform
		    		warpPerspective(resizedFeed , birdseyeFeed, H_cambird, resizedFeed.size(), WARP_INVERSE_MAP | INTER_LINEAR, BORDER_CONSTANT, Scalar::all(0));

	    		} else { // ID = 2 : collect H matricies
					while (H.isEmpty() < 1) {
						sleep(1);
					}
					//apply transforms 
		    		warpPerspective(resizedFeed , birdseyeFeed, H_camcam, resizedFeed.size(), WARP_INVERSE_MAP | INTER_LINEAR, BORDER_CONSTANT, Scalar::all(0));
		    		warpPerspective(resizedFeed , birdseyeFeed, H_cambird, resizedFeed.size(), WARP_INVERSE_MAP | INTER_LINEAR, BORDER_CONSTANT, Scalar::all(0));

	    		}
	    		*/


	    		H = getPerspectiveTransform(objPts, imgPts);
	    		cout << H.at<double>(0,0) <<  endl;
	    		H.at<double>(2,2) = birdseyeHeight;
	    		warpPerspective(resizedFeed , birdseyeFeed, H, resizedFeed.size(), WARP_INVERSE_MAP | INTER_LINEAR, BORDER_CONSTANT, Scalar::all(0));
				
				/*
				imgPts_vec.push_back(Point2f(imgPts[0].x,imgPts[0].y));
				imgPts_vec.push_back(Point2f(imgPts[1].x,imgPts[1].y));
				imgPts_vec.push_back(Point2f(imgPts[2].x,imgPts[2].y));
				imgPts_vec.push_back(Point2f(imgPts[3].x,imgPts[3].y));
	    		imgPts_vec[0].x = imgPts[0].x;
	    		imgPts_vec[0].y = imgPts[0].y;
	    		imgPts_vec[2] = imgPts[2];
	    		imgPts_vec[3] = imgPts[3];


	    		perspectiveTransform( imgPts_vec,transImgPts_vec, H2);
				// send checkerboard coordinates, conversion factor
	    		//checkerboardAnalysis();

	    		float x = H2.at<double>(0,0) * imgPts[0].x + H2.at<double>(0,1) * imgPts[0].y + H2.at<double>(0,2);
	    		float y = H2.at<double>(1,0) * imgPts[0].x + H2.at<double>(1,1) * imgPts[0].y + H2.at<double>(1,2);
	    		float w = H2.at<double>(2,0) * imgPts[0].x + H2.at<double>(2,1) * imgPts[0].y + H2.at<double>(2,2);

	    		//transImgPts[0]=Point(x/w,y/w);
				transImgPts[0]=transImgPts_vec[0];
				transImgPts[1]=transImgPts_vec[1];
	    		transImgPts[2]=transImgPts_vec[2];
	    		transImgPts[3]=transImgPts_vec[3];
				ROS_INFO("image calibrated");
				*/

	    	} else { // transform each frame
	
		   		warpPerspective(resizedFeed , birdseyeFeed, H, resizedFeed.size(), WARP_INVERSE_MAP | INTER_LINEAR, BORDER_CONSTANT, Scalar::all(0));

		    	// circle(birdseyeFeed, transImgPts[0], 10, Scalar(255,0,0),2);
		    	// circle(birdseyeFeed, transImgPts[1], 10, Scalar(0,255,0),2);
		    	// circle(birdseyeFeed, transImgPts[2], 10, Scalar(0,0,255),2);
		    	// circle(birdseyeFeed, transImgPts[3], 10, Scalar(0,255,255),2);		

		    	// circle(birdseyeFeed, imgPts[0], 5, Scalar(255,0,0),2);
		    	// circle(birdseyeFeed, imgPts[1], 5, Scalar(0,255,0),2);
		    	// circle(birdseyeFeed, imgPts[2], 5, Scalar(0,0,255),2);
		    	// circle(birdseyeFeed, imgPts[3], 5, Scalar(0,255,255),2);

		    	// ROS_INFO("image transformed");
			}	

			//birdseyeFeed = resizedFeed ;
			


			/*//adjust image and dim birdseye
			alpha = 1.2; // 1-3
			beta = 20; // 0-100
			birdseyeFeed_adjusted = Mat::zeros(birdseyeFeed.size(), birdseyeFeed.type());

			for (int y=0; y< birdseyeFeed.rows; y++) 
			{
				for (int x=0; x<birdseyeFeed.cols; x++)
				{
					for (int c = 0; c < 3 ; c++)
					{
						birdseyeFeed.at<Vec3b>(y,x)[c] = saturate_cast<uchar>(alpha*(birdseyeFeed.at<Vec3b>(y,x)[c]) - beta); // dim bg image

						birdseyeFeed_adjusted.at<Vec3b>(y,x)[c] = saturate_cast<uchar>(alpha*(birdseyeFeed.at<Vec3b>(y,x)[c]) - 60); // dim bg image

					}
				}
			}*/

	      // Background subtraction
	    	pMOG->operator()(birdseyeFeed,fgMaskMOG);

	      // Show general color detection on HSV window
	    	cvtColor(birdseyeFeed,HSV,COLOR_BGR2HSV);
//	    	createHSVTrackbars();
	    	inRange(HSV,Scalar(H_MIN,S_MIN,V_MIN),Scalar(H_MAX,S_MAX,V_MAX),HSVthreshold);


	    	morphologicalOps(fgMaskMOG, erodeSize, dilateSize);

	      // Apply object feed mask
	    	objectFeed = Scalar::all(0);
	    	birdseyeFeed.copyTo(objectFeed,fgMaskMOG);

	      // convert masked object feed to HSV color space for classification
	    	cvtColor(objectFeed,HSV,COLOR_BGR2HSV);
	    	//cvtColor(HSV,HSV,COLOR_BGR2HSV);


//	    	inRange(HSV,Scalar(H_BMIN,S_BMIN,V_BMIN),Scalar(H_BMAX,S_BMAX,V_BMAX),BLUEthreshold);
	    	//cvtColor(HSVcheck,HSVcheck,CV_HSV2BGR);
	    	//threshold(BLUEthreshold, BLUEthreshold, 0, 255, cv::THRESH_BINARY_INV);
	    	//HSV.copyTo(HSV,BLUEthreshold);
	    	//inRange(HSV,Scalar(H_GMIN,S_GMIN,V_GMIN),Scalar(H_GMAX,S_GMAX,V_GMAX),GREENthreshold);
	    	inRange(HSV,Scalar(H_YMIN,S_YMIN,V_YMIN),Scalar(H_YMAX,S_YMAX,V_YMAX),YELLOWthreshold);
	    	//inRange(HSV,Scalar(H_RMIN,S_RMIN,V_RMIN),Scalar(H_RMAX,S_RMAX,V_RMAX),REDthreshold);


	      // morphological operations on HSVobjects
//	    	morphologicalOps(BLUEthreshold, erodeSize, dilateSize);
	    	//morphologicalOps(GREENthreshold);
	    	morphologicalOps(YELLOWthreshold, erodeSize, dilateSize);
	    	//morphologicalOps(REDthreshold);
//	    	HSVobjects = YELLOWthreshold + BLUEthreshold ;
	    	HSVobjects = YELLOWthreshold;
	    	//SVobjects = BLUEthreshold + GREENthreshold + YELLOWthreshold + REDthreshold ;

	      // isolate objects from HSVobjects
	    	Mat occupancyGrid(objectFeed.rows,objectFeed.cols,CV_8UC1);
	    	Mat objectFeed_thresh(objectFeed.rows,objectFeed.cols,CV_8UC1);
	    	cvtColor(objectFeed,objectFeed_thresh,CV_BGR2GRAY);
			threshold(objectFeed_thresh, objectFeed_thresh, 1, 255, cv::THRESH_BINARY);
			threshold(HSVobjects, HSVobjects, 1, 255, cv::THRESH_BINARY);
			HSVobjects_dilated = Scalar::all(0);
			HSVobjects.copyTo(HSVobjects_dilated);
			morphologicalOps(HSVobjects_dilated, 1, astar_size);
	        subtract(objectFeed_thresh,HSVobjects_dilated,occupancyGrid);
	        morphologicalOps(occupancyGrid, 3, astar_size);
	        Mat HSVoccupancyGrid;
	        cvtColor(occupancyGrid, HSVoccupancyGrid, CV_GRAY2BGR,3);
	        cvtColor(HSVoccupancyGrid, HSVoccupancyGrid, CV_BGR2HSV,3);
	        objectFeed += HSVoccupancyGrid;

	      // either object detection or tracking mode


			if (tracking_status == FALSE)
			{
				//detectObjects(BLUEthreshold,objectFeed,"goal");
				//detectObjects(GREENthreshold,objectFeed,"green");
				detectObjects(YELLOWthreshold,objectFeed,"vehicle");
				//detectObjects(REDthreshold,objectFeed,"red");
				putText(objectFeed,"DISARMED : DETECTING VEHICLE",Point(0,50),1,1.5,Scalar(0,255,0),2); 

				// Set each element in history to 0
				for (int i = 0; i < VEHICLE_POSE_HISTORY_SIZE; i++) {
					vehicle_pose_history[i] = Point(0, 0);
				}

				arm_bool = 0;

			} 
			else // tracking mode turned on
			{
				// update pointer
				update_pose_history();

				detectObjects(occupancyGrid,objectFeed, " ");

				//trackObjects(BLUEthreshold,objectFeed,objects_blue,"goal");
				//trackObjects(GREENthreshold,objectFeed,objects_green,"green");
				trackObjects(YELLOWthreshold,objectFeed,objects_yellow,"vehicle");
				//trackObjects(REDthreshold,objectFeed,objects_red,"red");
				putText(objectFeed,"ARMED : TRACKING VEHICLE",Point(0,50),1,1.5,Scalar(0,0,255),2); 

				arm_bool = 1;

			}
			// downsample objectFeed to get occupancy grid
			downsampleGrid(occupancyGrid);

			// Create processing palate
			// dialog_box = Mat::zeros(100,400,CV_8UC3);
			//putText(dialog_box, "text test", Point(0,50),1,2,Scalar(255),2);
			//imshow(trackbar_window_name,dialog_box); 

			/*	      createTrackbar( "Min Threshold:", trackbar_window_name, &lowThreshold, max_lowThreshold);
			createTrackbar( "Erode Size:", trackbar_window_name, &erodeSize, max_erode+1);
			createTrackbar( "Dilate Size:", trackbar_window_name, &dilateSize, max_dilate+1);
			//createTrackbar( "Blur Size:", trackbar_window_name, &blurSize, max_blur+2);
			createTrackbar( "Blur Sigma Size:", trackbar_window_name, &sigmaSize, max_sigma+1);
			createTrackbar( "Distortion Angle", trackbar_window_name, &distortionAngle, max_distortion+1);
			createTrackbar( "Birds Eye Height", trackbar_window_name, &birdseyeHeight, max_birdseye+1);

			createTrackbar( "contrast", trackbar_window_name, &alpha, max_alpha+1);
			createTrackbar( "brightness", trackbar_window_name, &beta, max_beta+1);*/
			// drawCheckerboardCorners
			//drawChessboardCorners(birdseyeFeed, patternsize, Mat(corners), patternfound);      

			// add dimmed birdseyeFeed_adjusted to objectFeed display.
			//objectFeed += birdseyeFeed_adjusted;
			objectFeed += birdseyeFeed;
			downsampleFrameOut(objectFeed);
			// Show processed image
/*			imshow(windowName1,objectFeed);
			imshow(windowName2,resizedFeed);
			imshow(windowName3,HSV);
			imshow(windowName4,HSVoccupancyGrid);
			imshow(windowName5,occupancyGrid);
			imshow(windowName6,birdseyeFeed);*/
			//cv::waitKey(30); //wait for esc key

			std_msgs::Float64 arm_bool_msg;
			arm_bool_msg.data = arm_bool;
			rgb_arm_bool_pub.publish(arm_bool_msg);

			// Output modified video stream
			if (framedrop_count >= 4) 
		    {
		    	framedrop_count = 0;
				img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, frameoutDown);
				img_bridge.toImageMsg(img_msg); // from cv _bridge to sensor_msgs::Image
				rgb_pub_.publish(img_msg); 
			}
			// send high res occupancy grid
			occupancyHigh_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8,gridDownHigh);
			//send low res occupancy grid
			occupancyLow_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, gridDownLow);

			occupancyHigh_bridge.toImageMsg(occupancyGridHigh_msg);
			occupancyGridHigh_pub.publish(occupancyGridHigh_msg);
			occupancyLow_bridge.toImageMsg(occupancyGridLow_msg);
			occupancyGridLow_pub.publish(occupancyGridLow_msg);

			std_msgs::Float64 confidence_msg;
			confidence_msg.data = confidence;
			rgb_confidence_pub.publish(confidence_msg);

			counter++;
			framedrop_count++;
			ROS_INFO("counter=%i",counter);
	    }
	}
private:

	// INITIALIZATION ////////////////////////////////////////////////////////////////////


	int counter = 0;
	int framedrop_count = 0;

	// generate Mats
	Mat cameraFeed;
	Mat resizedFeed;
	Mat objectFeed;
	Mat grayFeed;
	Mat H;
	Mat birdseyeFeed;
	Mat HSV;
	Mat HSVobjects;
	Mat HSVobjects_dilated;
	Mat BLUEthreshold;
	Mat GREENthreshold;
	Mat YELLOWthreshold;
	Mat REDthreshold;
	Mat HSVthreshold;
	Mat HSVobjects_invert;
	Mat erodeElement;
	Mat dilateElement;
		//Mat occupancyGrid;
	Mat birdgrayFeed;
	Mat birdseyeFeed_adjusted;
	Mat gridDownLow;
	Mat gridDownHigh;
	Mat frameoutDown;
	//Mat occupancyGrid;
	//background subtraction global variables.
	Mat frame;
	Mat fgMaskMOG;

	Ptr<BackgroundSubtractor> pMOG;
	int history=1;
	float varThreshold = 16;
	bool bShadowDetection = true;

	int blurSize = 11;
	int sigmaSize = 3.5;
	int const max_blur = 40;
	int const max_sigma = 10;

	//max number of objects to be detected in frame
	const int MAX_NUM_OBJECTS=10;
	//minimum and maximum object area
	const int MIN_OBJECT_AREA = 10*10;

	//names of feed windows
	const string windowName1 = "ground_station_rgb_node";
	const string windowName2 = "resizedFeed";
	const string windowName3 = "HSV";
	const string windowName4 = "HSV occupancy grid";
	const string windowName5 = "occupancyGrid";
	const string windowName6 = "objectFeed";
	const string windowName7 = "HSV Trackbar";

	const string trackbar_window_name = "Parameter Palate" ;
	Mat dialog_box;

	//pallate controls
	int lowThreshold = 15;
	int dilateSize = 10;
	int erodeSize = 8;
	int const max_lowThreshold = 255;
	int const max_dilate = 100;
	int const max_erode = 100;
	int const max_beta = 100;
	int const max_alpha = 3.0;
	int alpha = 1;
	int beta = 0;

	int ratio = 3;
	int kernel_size = 3;

	//Color Segmentation Values
	double H_BMIN = 84;
	double H_BMAX = 129;
	double S_BMIN = 100;
	double S_BMAX = 198;
	double V_BMIN = 119;
	double V_BMAX = 255;

	int H_GMIN = 0;
	int H_GMAX = 75;
	int S_GMIN = 75;
	int S_GMAX = 150;
	int V_GMIN = 100;
	int V_GMAX = 180;

	int H_YMIN = 0;
	int H_YMAX = 55;
	int S_YMIN = 100;
	int S_YMAX = 220;
	int V_YMIN = 180;
	int V_YMAX = 255;

	int H_RMIN = 75;
	int H_RMAX = 255;
	int S_RMIN = 75;
	int S_RMAX = 160;
	int V_RMIN = 75;
	int V_RMAX = 160;

	int H_MIN = 0;
	int H_MAX = 255;
	int S_MIN = 0;
	int S_MAX = 255;
	int V_MIN = 0;
	int V_MAX = 255;

	//Distortion parameters
	int distortionAngle = 0;
	int const max_distortion = 180;
	int birdseyeHeight = 21;
	int const max_birdseye = 100;

	//checkerboard ... or chess, board parameters
	int board_w = 8;
	int board_h = 6;
	Size patternsize;
	Point2f objPts[4];
	Point2f imgPts[4];
	Point2f imgPtsLocal[4];	
	Point2f imgPtsRemote[4];	
	Point2f transImgPts[4];

	vector<Point2f> corners;
	vector<Point2f> transCorners;
	bool patternfound = 0;

	//astar Params
	int astar_size = 150; // change to length of car

	// tracking/detection toggle status (via space bar)
	bool tracking_status = FALSE;

	// vector init
	vector< vector<Point> > contours; 
	vector<Vec4i> hierarchy; 
	//vector <Object> objects; 

	vector <Object> objects_blue; 
	vector <Object> objects_green; 
	vector <Object> objects_yellow; 
	vector <Object> objects_red; 

	ros::Publisher rgb_vehicle_pub;
	ros::Publisher rgb_goal_pub;
	ros::Publisher conv_fac_pub;
	ros::Publisher rgb_arm_bool_pub;
	ros::Publisher rgb_confidence_pub;
	ros::Publisher corners_pub;

	Mat background;
	image_transport::Subscriber rgb_sub_;
	image_transport::Publisher rgb_pub_;
	image_transport::Publisher occupancyGridLow_pub;
	image_transport::Publisher occupancyGridHigh_pub;

	ros::Subscriber key_cmd_sub;


	double checkerboard_height = 1;
	int checkerboard_PXheight = 80;
	double checkerboard_width = 1;
	int checkerboard_PXwidth = 140;

	double height_factor, width_factor;
	int scale_factor; 

	int gridDownLow_height;
	int gridDownLow_width;
	int gridDownHigh_height;
	int gridDownHigh_width;


	double target_angle = 0;

	// vehicle location
	Point vehicle_pose ;
	Point prev_vehicle_pose ;

	// vehicle location history
	Point * vehicle_pose_history = new Point[VEHICLE_POSE_HISTORY_SIZE];
	int vehicle_pose_history_pointer;

	// vizualization waitkey cmds
	int key_cmd = 0;

	double vehicle_orientation_angle=0;
	double pca_vehicle_orientation_angle = 0;
	double poly_vehicle_orientation_angle = 0;
	double prev_vehicle_orientation_angle = 0;
	vector<Point> pose_poly;

	int arm_bool = 0;
	double confidence = 0;
	int ID_num ;
	int overlap_state = 0;

	int local_bool = 0;
	double waypoint_proj_perc = 0;

	int trajectory_state = 0;
	int projection_state = 0;
};





//
// MAIN //////////////////////////////////////////////////////////////////////////////////////////////////
//

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ground_station_rgb_node");
	ROS_INFO("ground_station_rgb_node launching");


  //launch image convertor
	RGBImageProcessor rip;
	
	//ros::MultiThreadedSpinner spinner(0);
	//spinner.spin();

	ros::spin();

	return 0;

}


