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

using namespace std;

// INITIALIZATION ////////////////////////////////////////////////////////////////////


int counter = 0;

// generate Mats
Mat cameraFeed;
Mat objectFeed;
Mat grayFeed;
Mat Homogeneous;
Mat birdseyeFeed;
Mat HSV;
Mat HSVobjects;
Mat BLUEthreshold;
Mat GREENthreshold;
Mat YELLOWthreshold;
Mat REDthreshold;
Mat HSVthreshold;
Mat HSVobjects_invert;
Mat erodeElement;
Mat dilateElement;
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
const int MIN_OBJECT_AREA = 20*20;

//names of feed windows
const string windowName = "Raw Feed";
const string windowName2 = "HSVobjects";
const string windowName3 = "Output Feed";
const string windowName4 = "Undistorted Feed";
const string windowName5 = "HSV Threshold Image";
const string windowName6 = "Threshold Image";
const string windowName7 = "HSV Trackbar";

const string trackbar_window_name = "Parameter Palate" ;
Mat dialog_box;

//pallate controls
int lowThreshold = 50;
int dilateSize = 30;
int erodeSize = 15;
int const max_lowThreshold = 255;
int const max_dilate = 100;
int const max_erode = 100;
int ratio = 3;
int kernel_size = 3;

//Color Segmentation Values
double H_BMIN = 90;
double H_BMAX = 136;
double S_BMIN = 100;
double S_BMAX = 186;
double V_BMIN = 20;
double V_BMAX = 135;

int H_GMIN = 34;
int H_GMAX = 80;
int S_GMIN = 50;
int S_GMAX = 220;
int V_GMIN = 50;
int V_GMAX = 200;

int H_YMIN = 20;
int H_YMAX = 30;
int S_YMIN = 50;
int S_YMAX = 220;
int V_YMIN = 50;
int V_YMAX = 200;

int H_RMIN = 0;
int H_RMAX = 196;
int S_RMIN = 100;
int S_RMAX = 196;
int V_RMIN = 30;
int V_RMAX = 76;

int H_MIN = 1;
int H_MAX = 255;
int S_MIN = 1;
int S_MAX = 255;
int V_MIN = 1;
int V_MAX = 255;

//Distortion parameters
int distortionAngle = 0;
int const max_distortion = 180;
int birdseyeHeight = 10;
int const max_birdseye = 100;

//checkerboard ... or chess, board parameters
int board_w = 8;
int board_h = 6;
int board_n = board_w*board_h;
Size board_sz(board_w, board_h);
Point2f objPts[4], imgPts[4];
Size patternsize(8,6);
vector<Point2f> corners;
bool patternfound = 0;

// tracking/detection toggle status (via space bar)
bool tracking_status = FALSE;

// vector init
vector< vector<Point> > contours; 
vector<Vec4i> hierarchy; 
vector< vector<Point> > contours_prev; 
vector<Vec4i> hierarchy_prev;                   
//vector <Object> objects; 

vector <Object> objects_blue; 
vector <Object> objects_green; 
vector <Object> objects_yellow; 
vector <Object> objects_red; 

ros::Publisher rgb_vehicle_pub;
ros::Publisher rgb_goal_pub;

//
// METHODS /////////////////////////////////////////////////////////////////////////////
//

string intToString(int number){ // for writing txt on frame
	std::stringstream ss;
	ss << number;
	return ss.str();
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


void drawObject(vector<Object> theObjects, Mat &frame, vector< vector<Point> > contours, vector<Vec4i> hierarchy)
{ // draw on frame
	int n, iter;
	if (counter < MEMORY_SIZE) {n = counter;} else {n = MEMORY_SIZE;}

	Scalar white = Scalar(255,255,255);
	if (contours.size() > 0) {
		drawContours(frame,contours,-1,white,3,8,hierarchy);
	}
	for(int i =0; i<theObjects.size(); i++)
  	{ //for each object
    //draw current position
		try {
			drawContours(frame,contours,i,theObjects.at(i).getColor(),3,8,hierarchy);
		} catch (Exception& e) {}
  		circle(frame,Point(theObjects.at(i).getXPos(MEMORY_SIZE-1),theObjects.at(i).getYPos(MEMORY_SIZE-1)),5,Scalar(0,0,255));
  		putText(frame,intToString(theObjects.at(i).getXPos(MEMORY_SIZE-1))+ " , " + intToString(theObjects.at(i).getYPos(MEMORY_SIZE-1)),cv::Point(theObjects.at(i).getXPos(MEMORY_SIZE-1),theObjects.at(i).getYPos(MEMORY_SIZE-1)+20),1,1,Scalar(0,255,0));
  		putText(frame,theObjects.at(i).getType() + ": " + intToString(i),Point(theObjects.at(i).getXPos(MEMORY_SIZE-1),theObjects.at(i).getYPos(MEMORY_SIZE-1)-20),1,2,theObjects.at(i).getColor());
    	//draw past positions if tracking
  		if (tracking_status == TRUE) {
  			for (int j = 2; j<(n-2); j++) { 
  				line(frame,Point(theObjects.at(i).getXPos(MEMORY_SIZE-j),theObjects.at(i).getYPos(MEMORY_SIZE - j)),Point(theObjects.at(i).getXPos(MEMORY_SIZE-(j-1)),theObjects.at(i).getYPos(MEMORY_SIZE - (j-1))),theObjects.at(i).getColor(),2);
  			}
  		}
    }
}

void morphologicalOps (Mat &thresh) 
{
	GaussianBlur(thresh,thresh,Size(blurSize,blurSize),sigmaSize,sigmaSize);

	threshold(thresh, thresh, lowThreshold, 255, cv::THRESH_BINARY);

    erodeElement = getStructuringElement( MORPH_RECT,Size(erodeSize+1,erodeSize+1)); //erode with 3x3 px rectangle
    erode(thresh,thresh,erodeElement);

    dilateElement = getStructuringElement( MORPH_RECT,Size(dilateSize+1,dilateSize+1)); //dilate with larger element so make sure object is nicely visible
    dilate(thresh,thresh,dilateElement);

  }

void detectObjects(Mat threshold, Mat &frame, string name) { // object detection 
  //generate temporary vectors
	vector< vector<Point> > contours_temp; 
	vector<Vec4i> hierarchy_temp;          
	vector <Object> objects_temp; 

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
      for (int index = 0; index >= 0; index = hierarchy_temp[index][0]) //for each object
      {
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
      else objectFound = false;
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
if (name=="blue") {
	objects_blue = objects_temp;
}
else if (name=="green") {
	objects_green = objects_temp;
}
else if (name=="yellow") {
	objects_yellow = objects_temp;
}
else if (name=="red") {
	objects_red = objects_temp;
}
  //clear temporary vectors
contours_temp.clear();
hierarchy_temp.clear();
objects_temp.clear();
}


void trackObjects(Mat threshold, Mat &frame,vector<Object> objects, string name) { //object tracking

  // similar contour detection

	Mat temp;

	threshold.copyTo(temp);

	findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );

	bool objectFound = false;
  // init
	int x_obj, y_obj, x_temp, y_temp, num_objects;
	float dist_temp, xdot_obj, ydot_obj;
	int minPos = 0;
  	float max_dist = 100; //pixels
  	vector<float> dist;
  	double x,y,th;

  if (hierarchy.size() > 0) {
  	num_objects = hierarchy.size();

      for (int index = 0; index < contours.size(); index++)
      {
          Moments moment = moments((cv::Mat)contours[index]); //moments method
          double area = moment.m00;

          x_temp = (int)moment.m10/area;       

          y_temp = (int)moment.m01/area; 

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

          	if (dist[minPos] < max_dist) 
          	{
               	ROS_INFO("x_temp%i=%i" , minPos , x_temp);

               	x_obj = x_temp;
               	y_obj = y_temp;

          		objects.at(minPos).setXPos(x_obj);
          		objects.at(minPos).setYPos(y_obj);
          	} else { // object too far away - project current position
          		int i=0;
          		x_obj = objects.at(i).getXPos(MEMORY_SIZE-1); // retrieve past object position.
		  		y_obj = objects.at(i).getYPos(MEMORY_SIZE-1);
		  		xdot_obj = objects.at(i).getXVel(MEMORY_SIZE-1); // retrieve past objects velocities
	 	  		ydot_obj = objects.at(i).getYVel(MEMORY_SIZE-1);

	 	  		x_obj = (int)(x_obj + xdot_obj * FPS);
	 	  		y_obj = (int)(y_obj + ydot_obj * FPS);

		  		objects.at(i).setXPos(x_obj);
		  		objects.at(i).setYPos(y_obj);
             	//objects.push_back(object_temp); // only track desired number of objects for right now, match to nearest object, ignore rest.
          	}
          	dist.clear();
          	minPos = 0;
 
          objectFound = true;

      }

	} else
	{
	  num_objects = 0;
	  // use guess at object travel as opposed to vision, otherwise the position index will remain zero, or last memory cycle value. 
	  for (int j = 0; j<objects.size(); j++) { //for each already existing object.
		  x_obj = objects.at(j).getXPos(MEMORY_SIZE-1); // retrieve past object position.
		  y_obj = objects.at(j).getYPos(MEMORY_SIZE-1);

		  xdot_obj = objects.at(j).getXVel(MEMORY_SIZE-1); // retrieve past objects velocities
	 	  ydot_obj = objects.at(j).getYVel(MEMORY_SIZE-1);

	 	  x_obj = (int)((float)x_obj + xdot_obj * (float)FPS);
	 	  y_obj = (int)((float)y_obj + ydot_obj * (float)FPS);

		  objects.at(j).setXPos(x_obj);
		  objects.at(j).setYPos(y_obj);
		}	  	  
	}
	
	//putText(frame,"CANT FIND OBJECTS",Point(0,50),1,2,Scalar(0,0,255),2); 
//}
  //draw object location on screen

//if vehicle, publish x,y,th .... if goal, publish x,y
x = (double)x_obj;
y = (double)y_obj;
th = atan2(y,x);

//virtual_infrastructure_pkg::vehicle_pose vehicle_pose_msg;
//virtual_infrastructure_pkg::goal_pose goal_pose_msg;
geometry_msgs::Pose2D vehicle_pose_msg;
geometry_msgs::Pose2D goal_pose_msg;

if (name=="blue") { // vehicle
	vehicle_pose_msg.x = x;
	vehicle_pose_msg.y = y;
	vehicle_pose_msg.theta = th;
	ROS_INFO("vehicle pose: ( %f , %f ) : th = %f ",x,y,th);
	rgb_vehicle_pub.publish(vehicle_pose_msg); 
}
else if (name=="red") { // goal
	goal_pose_msg.x = x;
	goal_pose_msg.y = y;
	ROS_INFO("goal pose: ( %f , %f ) ",x,y);
	rgb_goal_pub.publish(goal_pose_msg); 
}



drawObject(objects,frame, contours,hierarchy);
contours_prev = contours;
hierarchy_prev = hierarchy;

}

// RGB Image Converter Class ////////////////////////////

class RGBImageProcessor
{
	image_transport::ImageTransport it_rgb;
	image_transport::Subscriber rgb_sub_;
	image_transport::Publisher rgb_pub_;


public:

	Mat background;

	RGBImageProcessor(ros::NodeHandle nh_rgb )
	: it_rgb(nh_rgb)
	{
    // Subscribe to input video feed and publish output video feed
		rgb_sub_ = it_rgb.subscribe("/camera/image_raw",1, &RGBImageProcessor::rgbFeedCallback, this); // use image_rect
		rgb_pub_ = it_rgb.advertise("ground_station_rgb_node",1);


		rgb_vehicle_pub = nh_rgb.advertise<geometry_msgs::Pose2D>("vehicle_pose",2);
		rgb_goal_pub = nh_rgb.advertise<geometry_msgs::Pose2D>("goal_pose",2);
	}

	~RGBImageProcessor()
	{
    //cv::destroyWindow(); // all
	}

	void rgbFeedCallback(const sensor_msgs::ImageConstPtr& msg)
	{

		cv_bridge::CvImage img_bridge;
		sensor_msgs::Image img_msg;
		cv_bridge::CvImagePtr rgb_cv_ptr;
		
		try
		{
			rgb_cv_ptr  = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

	    std_msgs::Header header; //empty header
	    header.seq = counter; // user defined counter
	    header.stamp = ros::Time::now(); // time
	    cameraFeed = rgb_cv_ptr -> image;

	    dialog_box = Mat::zeros(100,400,CV_8UC3);

	    //cameraFeed.copyTo(threshold);

	    if (patternfound < 1) 
	    {
	    	patternfound = findChessboardCorners(cameraFeed, patternsize,corners);  
	    	putText(cameraFeed,"LOOKING FOR CHECKERBOARD",Point(0,50),1,2,Scalar(0,0,255),2); 
	    	imshow(windowName,cameraFeed);
	    	waitKey(30);
	    } 
	    else { // pattern status already changed 
	    	if (counter < 1) 
	    	{
	    		cvtColor(cameraFeed, grayFeed, COLOR_BGR2GRAY);
	    		cornerSubPix(grayFeed, corners, Size(11,11),Size(-1,-1), TermCriteria( cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.1));

	    		objPts[0].x=0;
	    		objPts[1].x=board_w-1;
	    		objPts[2].x=0;
	    		objPts[3].x=board_w-1;
	    		objPts[0].y=0;
	    		objPts[1].y=0;
	    		objPts[2].y=board_h-1;
	    		objPts[3].y=board_h-1;

	    		imgPts[0]=corners[0];
	    		imgPts[1]=corners[board_w-1];
	    		imgPts[2]=corners[(board_h-1)*board_w];
	    		imgPts[3]=corners[(board_h-1)*board_w+board_w-1];

	    		Homogeneous = getPerspectiveTransform(objPts, imgPts);

	    	}

	    	Homogeneous.at<double>(2,2) = birdseyeHeight;
	    	warpPerspective(cameraFeed, birdseyeFeed, Homogeneous, cameraFeed.size(), WARP_INVERSE_MAP | INTER_LINEAR, BORDER_CONSTANT, Scalar::all(0));

	    	circle(cameraFeed, imgPts[0], 9, Scalar(255,0,0),3);
	    	circle(cameraFeed, imgPts[1], 9, Scalar(0,255,0),3);
	    	circle(cameraFeed, imgPts[2], 9, Scalar(0,0,255),3);
	    	circle(cameraFeed, imgPts[3], 9, Scalar(0,255,255),3);

	      // Background subtraction
	    	pMOG->operator()(cameraFeed,fgMaskMOG);

	      // Show general color detection on HSV window
	    	cvtColor(cameraFeed,HSV,COLOR_BGR2HSV);
	    	createHSVTrackbars();
	    	inRange(HSV,Scalar(H_MIN,S_MIN,V_MIN),Scalar(H_MAX,S_MAX,V_MAX),HSVthreshold);


	    	morphologicalOps(fgMaskMOG);

	      // Apply object feed mask
	    	objectFeed = Scalar::all(0);
	    	cameraFeed.copyTo(objectFeed,fgMaskMOG);

	      // convert masked object feed to HSV color space for classification
	    	cvtColor(objectFeed,HSV,COLOR_BGR2HSV);
	    	cvtColor(HSV,HSV,COLOR_BGR2HSV);
	    	inRange(HSV,Scalar(H_BMIN,S_BMIN,V_BMIN),Scalar(H_BMAX,S_BMAX,V_BMAX),BLUEthreshold);
	    	//threshold(BLUEthreshold, BLUEthreshold, 0, 255, cv::THRESH_BINARY_INV);
	    	//HSV.copyTo(HSV,BLUEthreshold);
	    	inRange(HSV,Scalar(H_GMIN,S_GMIN,V_GMIN),Scalar(H_GMAX,S_GMAX,V_GMAX),GREENthreshold);
	    	inRange(HSV,Scalar(H_YMIN,S_YMIN,V_YMIN),Scalar(H_YMAX,S_YMAX,V_YMAX),YELLOWthreshold);
	    	inRange(HSV,Scalar(H_RMIN,S_RMIN,V_RMIN),Scalar(H_RMAX,S_RMAX,V_RMAX),REDthreshold);


	      // morphological operations on HSVobjects
	    	morphologicalOps(BLUEthreshold);
	    	morphologicalOps(GREENthreshold);
	    	morphologicalOps(YELLOWthreshold);
	    	morphologicalOps(REDthreshold);

	    	HSVobjects = BLUEthreshold + GREENthreshold + YELLOWthreshold + REDthreshold ;

	      // isolate objects from HSVobjects
	      //cameraFeed.copyTo(HSVobjects_invert);
	/*      cameraFeed = objectFeed - HSVobjects;
	      bitwise_not(HSVobjects,HSVobjects_invert);
	      bitwise_and(objectFeed,HSVobjects_invert,cameraFeed); */

	      // either object detection or tracking mode
	    	if (tracking_status == FALSE)
	    	{
	    		detectObjects(BLUEthreshold,objectFeed,"blue");
	    		detectObjects(GREENthreshold,objectFeed,"green");
	    		detectObjects(YELLOWthreshold,objectFeed,"yellow");
	    		detectObjects(REDthreshold,objectFeed,"red");
	    		putText(cameraFeed,"DETECTING OBJECTS",Point(0,50),1,2,Scalar(0,0,255),2); 

	    	} 
	      else // tracking mode turned on
	      {
	      	trackObjects(BLUEthreshold,objectFeed,objects_blue,"blue");
	      	trackObjects(GREENthreshold,objectFeed,objects_green,"green");
	      	trackObjects(YELLOWthreshold,objectFeed,objects_yellow,"yellow");
	      	trackObjects(REDthreshold,objectFeed,objects_red,"red");
	      	putText(cameraFeed,"TRACKING OBJECTS",Point(0,50),1,2,Scalar(0,0,255),2); 
	      }

	      // Create processing palate
	      //putText(dialog_box, "text test", Point(0,50),1,2,Scalar(255),2);
	      imshow(trackbar_window_name,dialog_box); 

	      createTrackbar( "Min Threshold:", trackbar_window_name, &lowThreshold, max_lowThreshold);
	      createTrackbar( "Erode Size:", trackbar_window_name, &erodeSize, max_erode+1);
	      createTrackbar( "Dilate Size:", trackbar_window_name, &dilateSize, max_dilate+1);
	      //createTrackbar( "Blur Size:", trackbar_window_name, &blurSize, max_blur+2);
	      createTrackbar( "Blur Sigma Size:", trackbar_window_name, &sigmaSize, max_sigma+1);
	      createTrackbar( "Distortion Angle", trackbar_window_name, &distortionAngle, max_distortion+1);
	      createTrackbar( "Birds Eye Height", trackbar_window_name, &birdseyeHeight, max_birdseye+1);

	      // drawCheckerboardCorners
	      drawChessboardCorners(cameraFeed, patternsize, Mat(corners), patternfound);      

	      // Show processed image
	      imshow(windowName2, HSVobjects);
	      imshow(windowName3,objectFeed);
	      imshow(windowName,cameraFeed);
	      imshow(windowName4,birdseyeFeed);
	      imshow(windowName5,HSVthreshold);
	      imshow(windowName6,fgMaskMOG);

	      char k = (char) cv::waitKey(30); //wait for esc key
	      //if(k == 27) break;
	      if(k== ' ') 
	      {
	      	if (tracking_status == TRUE)
	      	{
	      		tracking_status = FALSE;
	      		counter = 0;
	      	}
	      	else
	      	{
	      		tracking_status = TRUE;
	      		//objects.clear();
	      		//objects.insert(objects.end(), objects_blue.begin(), objects_blue.end());
	      		//objects.insert(objects.end(), objects_green.begin(), objects_green.end());
	      		//objects.insert(objects.end(), objects_yellow.begin(), objects_yellow.end());
	      		//objects.insert(objects.end(), objects_red.begin(), objects_red.end());
	      		counter = 0;
	      	}
	      } 
	      else if (k==27)
	      {
	      	pMOG = new BackgroundSubtractorMOG();
	      }

	      // Output modified video stream
	      img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, objectFeed);
	      img_bridge.toImageMsg(img_msg); // from cv _bridge to sensor_msgs::Image
	      rgb_pub_.publish(img_msg); 
	      //rgb_pub_.publish(cameraFeed->toImageMsg());

	      counter++;
	  }
	}
};

//
// MAIN //////////////////////////////////////////////////////////////////////////////////////////////////
//

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ground_station_rgb_node");

	ros::NodeHandle nh_rgb;

	ROS_INFO("ground_station_rgb_node launching");

  //create background subtractor object
	pMOG = new BackgroundSubtractorMOG();

  //launch image convertor
	RGBImageProcessor rip(	ros::NodeHandle nh_rgb);

	ros::spin();

	return 0;

}


