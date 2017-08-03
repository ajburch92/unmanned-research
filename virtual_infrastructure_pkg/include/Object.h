//
// Object.h
// Austin Burch
// 
// This file defines the Object class used in Object Detection and Tracking. 
//

#ifndef OBJECT_H
#define OBJECT_H

#include <cv.h>
#include <highgui.h>
#include <deque>
#include <ros/ros.h>

#define FPS 20
#define MEMORY_SIZE 1024

using namespace std;
using namespace cv;

class Object
{
public:
	Object(string name);
	~Object(void);

	int getXPos(int i);
	void setXPos(int x);

	int getYPos(int i);
	void setYPos(int y);

	float getXVel(int i);
	float getYVel(int i);

	Scalar getHSVmin();
	Scalar getHSVmax();

	void setHSVmin(Scalar min);
	void setHSVmax(Scalar max);
	
	string getType(){return type;}
	void setType(string t){type = t;}

	Scalar getColor(){
		return Color;
	}
	void setColor(Scalar c){

		Color = c;
	}	

private:

	deque<int> xPos_vec;
	deque<int> yPos_vec;
	deque<float> xVel_vec;
	deque<float> yVel_vec;

	int xPos_curr, yPos_curr;
	int xPos_prev, yPos_prev;
	float xVel_curr, yVel_curr;
	float xVel_prev, yVel_prev;
	float error;

	string type;
	Scalar HSVmin, HSVmax;
	Scalar Color;

	float velXFilter();
	float velYFilter();
	
	void rollXVectors();
	void rollYVectors();
};
#endif