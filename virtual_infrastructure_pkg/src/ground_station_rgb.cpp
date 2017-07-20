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

using namespace std;

// INITIALIZATION ////////////////////////////////////////////////////////////////////
int counter = 0;

// generate Mats
Mat cameraFeed;
Mat outputFeed;
Mat grayFeed;
Mat Homogeneous;
Mat birdseyeFeed;

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
const string windowName2 = "Threshold Feed";
const string windowName3 = "Output Feed";
const string windowName4 = "Undistorted Feed";
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
vector <Object> objects; 

vector< vector<Point> > contours_curr; 
vector<Vec4i> hierarchy_curr;          
vector <Object> objects_curr; 

//
// METHODS /////////////////////////////////////////////////////////////////////////////
//

string intToString(int number){ // for writing txt on frame
  std::stringstream ss;
  ss << number;
  return ss.str();
}

void drawObject(vector<Object> theObjects,Mat &frame){ // draw on frame
  int n;
  if (counter < MEMORY_SIZE) {n = counter;} else {n = MEMORY_SIZE;}
  for(int i =0; i<theObjects.size(); i++){ //for each object
    //draw current position
    circle(frame,Point(theObjects.at(i).getXPos(MEMORY_SIZE-1),theObjects.at(i).getYPos(MEMORY_SIZE-1)),10,Scalar(0,0,255));
    putText(frame,intToString(theObjects.at(i).getXPos(MEMORY_SIZE-1))+ " , " + intToString(theObjects.at(i).getYPos(MEMORY_SIZE-1)),cv::Point(theObjects.at(i).getXPos(MEMORY_SIZE-1),theObjects.at(i).getYPos(MEMORY_SIZE-1)+20),1,1,Scalar(0,255,0));

    //draw past positions if tracking
    if (tracking_status == TRUE) {
      for (int j = 2; j<(n-3); j++) { 
        line(frame,Point(theObjects.at(i).getXPos(MEMORY_SIZE-j),theObjects.at(i).getYPos(MEMORY_SIZE - j)),Point(theObjects.at(i).getXPos(MEMORY_SIZE-(j-1)),theObjects.at(i).getYPos(MEMORY_SIZE - (j-1))),Scalar(0,255,255));
      }
      }
  }
}

void detectObjects(Mat threshold, Mat &cameraFeed) { // object detection 
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
          Object object; // generate new instance

          object.setXPos(moment.m10/area); // find x
          object.setYPos(moment.m01/area); // y

          objects_temp.push_back(object); 

          objectFound = true;

        }
        else objectFound = false;
      }
      if(objectFound ==true)
      {
        //draw object on cameraFeed
        drawObject(objects_temp,cameraFeed);
      }
    }
    else {
      putText(cameraFeed,"NOISE! ADJUST FILTER",Point(0,50),1,2,Scalar(0,0,255),2); 
    }
  }

  //save temporary vectors as current
  contours_curr = contours_temp; 
  hierarchy_curr = hierarchy_temp;          
  objects_curr = objects_temp;

  //clear temporary vectors
  contours_temp.clear();
  hierarchy_temp.clear();
  objects_temp.clear();
}


void trackObjects(Mat threshold, Mat &cameraFeed) { //object tracking

  // similar contour detection

  Mat temp;

  threshold.copyTo(temp);
  
  findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );
  
  bool objectFound = false;
  // init
  int x_temp, y_temp;
  float x_obj, y_obj, dist_temp;
  int minPos = 0;
  int max_dist = 200; //pixels
  vector<float> dist;

  if (hierarchy.size() > 0) {
    int numObjects = hierarchy.size();
    //if number of objects greater than MAX_NUM_OBJECTS, then image theshold is too noisy
    if(numObjects<MAX_NUM_OBJECTS)
    {
      for (int index = 0; index >= 0; index = hierarchy[index][0]) //for each object
      {
        Moments moment = moments((cv::Mat)contours[index]); //moments method
        double area = moment.m00;

        if(area>MIN_OBJECT_AREA) //classify as object
        {
          Object object_temp; // generate new temporary object

          x_temp = (int)moment.m10/area;       
          object_temp.setXPos(x_temp);
          //std::string ss = "x_temp = " + std::to_string(x_temp);
          //putText(dialog_box, ss, Point(0,50),1,2,Scalar(255),2);

          y_temp = (int)moment.m01/area; 
          object_temp.setYPos(y_temp);
          // compare distances to existing objects
          for (int i = 0; i<objects.size();i++) {
            x_obj = (float)objects.at(i).getXPos(MEMORY_SIZE-1);
            y_obj = (float)objects.at(i).getYPos(MEMORY_SIZE-1);
            dist_temp = sqrt(pow((x_obj - (float)x_temp),2.0) + pow((y_obj - (float)y_temp),2.0));
            dist.push_back(dist_temp);
          }

          // find object with min dist
          for (int j = 0 ; j<dist.size();j++)
          {
              if (dist[j] < minPos)
            {
                minPos = j;
            }
          }
          // if distance is in neighborhood, assign xy coordinates
          
          if (dist[minPos] < max_dist) 
          {
            objects.at(minPos).setXPos(x_temp);
            objects.at(minPos).setYPos(y_temp);
          }

          else { // object too far away - new object
             //objects.push_back(object_temp); // only track desired number of objects for right now, match to nearest object, ignore rest.
          }
          
          objectFound = true;

        }
        else objectFound = false;
      }
    }
  } else
  {
    putText(cameraFeed,"CANT FIND OBJECTS",Point(0,50),1,2,Scalar(0,0,255),2); 
  }
  //draw object location on screen
  drawObject(objects,cameraFeed);
  //clear objects_curr
}

// RGB Image Converter Class ////////////////////////////

class RGBImageProcessor
{

  ros::NodeHandle nh_rgb;
  image_transport::ImageTransport it_rgb;
  image_transport::Subscriber rgb_sub_;
  image_transport::Publisher rgb_pub_;

public:

  Mat background;

  RGBImageProcessor()
  : it_rgb(nh_rgb)
  {
    // Subscribe to input video feed and publish output video feed
    rgb_sub_ = it_rgb.subscribe("/camera/image_raw",1, &RGBImageProcessor::rgbFeedCallback, this);
    rgb_pub_ = it_rgb.advertise("ground_station_rgb_node",1);
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
    else { // pattern
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

      // drawCheckerboardCorners
      drawChessboardCorners(cameraFeed, patternsize, Mat(corners), patternfound);      

      // Background subtraction
      pMOG->operator()(cameraFeed,fgMaskMOG);

      //Gaussian Blur
      GaussianBlur(fgMaskMOG,fgMaskMOG,Size(blurSize,blurSize),sigmaSize,sigmaSize);
      
      //Binary Threshold
      threshold(fgMaskMOG, fgMaskMOG, lowThreshold, 255, cv::THRESH_BINARY);

      // erode frame
      Mat erodeElement = getStructuringElement( MORPH_RECT,Size(erodeSize+1,erodeSize+1)); //erode with 3x3 px rectangle
      erode(fgMaskMOG,fgMaskMOG,erodeElement);

      // dilate frame
      Mat dilateElement = getStructuringElement( MORPH_RECT,Size(dilateSize+1,dilateSize+1)); //dilate with larger element so make sure object is nicely visible
      dilate(fgMaskMOG,fgMaskMOG,dilateElement);

      // create black foreground
      outputFeed = Scalar::all(0);
      cameraFeed.copyTo(outputFeed,fgMaskMOG);

      // either object detection or tracking mode
      if (tracking_status == FALSE)
      {
        detectObjects(fgMaskMOG,outputFeed);
        putText(cameraFeed,"DETECTING OBJECTS",Point(0,50),1,2,Scalar(0,0,255),2); 

      } 
      else // tracking mode turned on
      {
        trackObjects(fgMaskMOG,outputFeed);
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

      // Show processed image
      imshow(windowName2, fgMaskMOG);
      imshow(windowName3,outputFeed);
      imshow(windowName,cameraFeed);
      imshow(windowName4,birdseyeFeed);

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
          contours = contours_curr;
          hierarchy = hierarchy_curr;
          objects = objects_curr ;
        }
      } 
      else if (k==27)
      {
  		pMOG = new BackgroundSubtractorMOG();
      }

      // Output modified video stream
      img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, outputFeed);
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

  // init ROS //
  ROS_INFO("ground_station_rgb_node launching");
  ros::init(argc, argv, "ground_station_rgb_node");

  //create background subtractor object
  pMOG = new BackgroundSubtractorMOG();

  //launch image convertor
  RGBImageProcessor rip;

  ros::spin();

  return 0;

}


