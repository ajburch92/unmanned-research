//
// Object.cpp
// Austin Burch
// 
// This file defines the Object class's methods. 
//

#include "Object.h"

Object::Object()
{
	//allocate memory
	xPos_vec.resize(MEMORY_SIZE,0);
	yPos_vec.resize(MEMORY_SIZE,0);
	xVel_vec.resize(MEMORY_SIZE,0);
	yVel_vec.resize(MEMORY_SIZE,0);
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
	xVel_curr = velXFilter(); // 
	// store prev
	xPos_prev = xPos_curr;
	xVel_prev = xVel_curr;
	// roll deques
	rollXVectors();
}

int Object::getYPos(int i){
	return yPos_vec[i];

}

void Object::setYPos(int y){

	yPos_curr = y;
	yVel_curr = velYFilter();
	yPos_prev = xPos_curr;
	yVel_prev = yVel_curr;
	rollYVectors();
}

float Object::getYVel(){
	return yVel_curr;
}

float Object::velXFilter() {
	float vel_curr = (float)(((xPos_curr - xPos_prev) * FPS) + xVel_prev) / 2 ; //moving avg of tail = 1, w = 0.5
	return vel_curr;
}

float Object::velYFilter() {
	float vel_curr = (float)(((yPos_curr - yPos_prev) * FPS) + yVel_prev) / 2 ; //
	return vel_curr;
}

float Object::getXVel(){

	return xVel_curr;

}

void Object::rollXVectors() {
	rotate(xPos_vec.begin(), xPos_vec.begin() + 1, xPos_vec.end());
	xPos_vec[MEMORY_SIZE - 1] = xPos_curr;
	rotate(xVel_vec.begin(), xVel_vec.begin() + 1, xVel_vec.end());
	xVel_vec[MEMORY_SIZE - 1] = xVel_curr;
}

void Object::rollYVectors() {
	rotate(yPos_vec.begin(), yPos_vec.begin() + 1, yPos_vec.end());
	yPos_vec[MEMORY_SIZE - 1] = yPos_curr;
	rotate(yVel_vec.begin(), yVel_vec.begin() + 1, yVel_vec.end());
	yVel_vec[MEMORY_SIZE - 1] = yVel_curr;
}