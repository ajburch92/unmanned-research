//
// Object.cpp
// Austin Burch
// 
// This file defines the Object class's methods. 
//

#include "Object.h"

Object::Object(string name)
{
	//allocate memory
	xPos_vec.resize(MEMORY_SIZE,0);
	yPos_vec.resize(MEMORY_SIZE,0);
	xVel_vec.resize(MEMORY_SIZE,0);
	yVel_vec.resize(MEMORY_SIZE,0);


	setType(name);
	
	if(name=="goal"){

		//TODO: use "calibration mode" to find HSV min
		//and HSV max values

		setHSVmin(Scalar(92,0,0));
		setHSVmax(Scalar(124,256,256));

		//BGR value for Blue:
		setColor(Scalar(255,0,0));

	}
	if(name=="green"){

		//TODO: use "calibration mode" to find HSV min
		//and HSV max values

		setHSVmin(Scalar(34,50,50));
		setHSVmax(Scalar(80,220,200));

		//BGR value for Green:
		setColor(Scalar(0,255,0));

	}
	if(name=="vehicle"){

		//TODO: use "calibration mode" to find HSV min
		//and HSV max values

		setHSVmin(Scalar(20,124,123));
		setHSVmax(Scalar(30,256,256));

		//BGR value for Yellow:
		setColor(Scalar(0,255,255));

	}
	if(name=="red"){

		//TODO: use "calibration mode" to find HSV min
		//and HSV max values

		setHSVmin(Scalar(0,200,0));
		setHSVmax(Scalar(19,255,255));

		//BGR value for Red:
		setColor(Scalar(0,0,255));

	}
}

Object::~Object(void)
{
	//delete mem
}

int Object::getXPos(int i){ // i = MEMORY_SIZE is equal to the current reading
	return xPos_vec[i];
}

void Object::setXPos(int x){
	xPos_curr = x;
	rollXVectors();
	xVel_curr = velXFilter(); // 
	// store prev
	xPos_prev = xPos_curr;
	xVel_prev = xVel_curr;
	// roll deques

}

int Object::getYPos(int i){
	return yPos_vec[i];

}

void Object::setYPos(int y){

	yPos_curr = y;
	rollYVectors();
	yVel_curr = velYFilter();
	yPos_prev = yPos_curr;
	yVel_prev = yVel_curr;

}

float Object::getYVel(int i){
	return yVel_vec[i];
}

float Object::velXFilter() {
	float vel_curr = (((xPos_curr - xPos_prev) * FPS) + xVel_prev) / 2 ; //moving avg of tail = 1, w = 0.5
	ROS_INFO("vel = %f",vel_curr);
	return vel_curr;
}

float Object::velYFilter() { 
	float vel_curr = (float)(((yPos_curr - yPos_prev) * FPS) + yVel_prev) / 2 ; //
	return vel_curr;
}

float Object::getXVel(int i){

	return xVel_vec[i];

}

void Object::rollXVectors() {
	xPos_vec[0] = xPos_curr;
	rotate(xPos_vec.begin(), xPos_vec.begin() + 1, xPos_vec.end());
	//ROS_INFO("xfirst= %i", yPos_vec[0]);
	//ROS_INFO("xlast= %i", yPos_vec[MEMORY_SIZE-1]);
	//ROS_INFO("xsecondtolast= %i", yPos_vec[MEMORY_SIZE-2]);
	//ROS_INFO("xthirdtolast= %i", yPos_vec[MEMORY_SIZE-3]);
	///ROS_INFO("x4thtolast= %i", yPos_vec[MEMORY_SIZE-4]);
	//ROS_INFO("x5thtolast= %i", yPos_vec[MEMORY_SIZE-5]);
	//ROS_INFO("x6thtolast= %i", yPos_vec[MEMORY_SIZE-6]);


	xVel_vec[0] = xVel_curr;
	rotate(xVel_vec.begin(), xVel_vec.begin() + 1, xVel_vec.end());
}

void Object::rollYVectors() {
	yPos_vec[0] = yPos_curr;
	rotate(yPos_vec.begin(), yPos_vec.begin() + 1, yPos_vec.end());
	yVel_vec[0] = yVel_curr;
	rotate(yVel_vec.begin(), yVel_vec.begin() + 1, yVel_vec.end());
} 

Scalar Object::getHSVmin(){

	return Object::HSVmin;

}
Scalar Object::getHSVmax(){

	return Object::HSVmax;
}

void Object::setHSVmin(Scalar min){

	Object::HSVmin = min;
}


void Object::setHSVmax(Scalar max){

	Object::HSVmax = max;
}
