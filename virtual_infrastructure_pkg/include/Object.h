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

#define FPS 30
#define MEMORY_SIZE 1024

using namespace std;
using namespace cv;

class Object
{
public:
	Object();
	~Object(void);

	int getXPos(int i);
	void setXPos(int x);

	int getYPos(int i);
	void setYPos(int y);

	float getXVel();
	float getYVel();

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

	float velXFilter();
	float velYFilter();
	
	void rollXVectors();
	void rollYVectors();
};
#endif