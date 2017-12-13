#include "kalman_gps_acc/Vector.h"
#include <stdio.h>

Vector2D::Vector2D(){
	this->v[0]=0;
	this->v[1] = 1;
}

void Vector2D::setEntry(int row, double val){
	this->v[row] = val;
}

void Vector2D::printVector(){
	
		printf("[%f, %f]'\n", this->v[0], this->v[1]);
}

double Vector2D::getX(){
	return this->v[0];
}

double Vector2D::getY(){
	return this->v[1];
}
