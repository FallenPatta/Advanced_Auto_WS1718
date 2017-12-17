#include "kalman_gps_acc/Vector.h"
#include <stdio.h>

Vector2D::Vector2D(){
	this->v[0]=0;
	this->v[1] = 1;
}

Vector2D::Vector2D(double x, double y){
	this->v[0] = x;
	this->v[1] = y;
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

void Vector2D::setX(double x){
	this->v[0] = x;
}

void Vector2D::setY(double y){
	this->v[1] = y;
}

Vector2D Vector2D::operator+(Vector2D other){
	Vector2D result;
	result.setX(other.getX() + this->getX());
	result.setY(other.getY() + this->getY());
	return result;
}

Vector2D Vector2D::operator-(Vector2D other){
	Vector2D result;
	result.setX(this->getX() - other.getX());
	result.setY(this->getY() - other.getY());
	return result;
}

Vector2D Vector2D::operator*(Vector2D other){
	Vector2D result;
	result.setX(other.getX() * this->getX());
	result.setY(other.getY() * this->getY());
	return result;
}

Vector2D Vector2D::operator/(Vector2D other){
	Vector2D result;
	result.setX(this->getX() / other.getX());
	result.setY(this->getY() / other.getY());
	return result;
}

Vector2D Vector2D::operator*(double other){
	Vector2D result;
	result.setX(other * this->getX());
	result.setY(other * this->getY());
	return result;
}

Vector2D Vector2D::operator/(double other){
	Vector2D result;
	result.setX(this->getX() / other);
	result.setY(this->getY() / other);
	return result;
}
