//
// ground_station_therm.cpp
// Austin Burch
// 
// This file initiates the therm_node and launches the thermal image processor
//

//ros
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
//opencv
#include <cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
//
#include <iostream>

//////////////////////////////////////////////////////////////////////
// For the THERM-APP camera:

// must # modprobe l4v2loopback
// for documentation, see https://github.com/umlaeute/v4l2loopback/

// must run thermapp, use launch file to start executable  

using namespace cv;
using namespace std;

static const string THERM_WINDOW = "Therm camera feed";
///////////////////////////////////////////////////////////////////////
class ThermImageConverter
{

	ros::NodeHandle nh_therm;

public:
	ThermImageConverter()
	{

		ros::Publisher therm_pub_ = nh_therm.advertise<sensor_msgs::Image>("ground_station_therm",1);

		cv_bridge::CvImage img_bridge;
		sensor_msgs::Image img_msg;
		VideoCapture vc;

    vc.open(0); // open the video camera, 0
    if (!vc.isOpened()) // if n0ot success, exit program
    {
    	cout << "Cannot open the video cam" << endl;
    }

    double  dWidth = vc.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
    double  dHeight = vc.get(CV_CAP_PROP_FRAME_HEIGHT); // get the height of frames of the video
    cout << "Frame size : " << dWidth << "x" << dHeight << endl;
    //namedWindow(THERM_WINDOW, CV_WINDOW_AUTOSIZE);
    //cv::namedWindow(BG_SEG_WINDOW, CV_WINDOW_AUTOSIZE);

    int counter = 0;
    while(1) {
    	counter++;
    	Mat therm_frame;
      std_msgs::Header header; //empty header
      header.seq = counter; // user defined counter
      header.stamp = ros::Time::now(); // time

      bool bSuccess = vc.read(therm_frame); // read a new frame from video
      
      if (!bSuccess) // if not success break loop
      {
      	cout << "Cannot read a frame from thermal video stream" << endl;
      	break;
      }

      //rotate therm_frame
      Point2f src_center(therm_frame.cols/2.0F, therm_frame.rows/2.0F);

      Mat rot_matrix = getRotationMatrix2D(src_center, 90, 1.0);

      Mat rotated_therm_frame(Size(therm_frame.size().height, therm_frame.size().width), therm_frame.type());

      warpAffine(therm_frame, rotated_therm_frame, rot_matrix, therm_frame.size());


      // Update GUI Window
      //imshow(THERM_WINDOW, rotated_therm_frame);
      //cv::imshow(BG_SEG_WINDOW, 1);

      img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, rotated_therm_frame);
      img_bridge.toImageMsg(img_msg); // from cv _bridge to sensor_msgs::Image
      therm_pub_.publish(img_msg); 

      if (waitKey(30) == 27) // wait for 'esc' key press for 30ms. If 'esc key is pressed, break loop
      {
      	cout << "esc key pressed" << endl;
      	break;
      }
  }
}

~ThermImageConverter()
{
	//destroyWindow(THERM_WINDOW);
}

};

/////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
	ROS_INFO("ground_station_therm_node launching");
	ros::init(argc, argv, "ground_station_therm_node");
	ThermImageConverter tic;
	return 0;
}