#include "kalman_gps_acc/State_Vector.h"
#include "kalman_gps_acc/definitions.h"

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/core/matx.hpp"
#include "opencv2/core/types.hpp"
#include "opencv2/core/bufferpool.hpp"
#include "opencv2/core/mat.inl.hpp"

using namespace std;
using namespace cv;

	StateVector2D::StateVector2D(){
		this->x = Vec2f(0,0);
		this->x_ = Vec2f(0,0);
		//~ this->x__ = Vec2f(0,0);
	}
	
	StateVector2D::StateVector2D(Vec2f x,Vec2f x_){//,Vec2f x__){
		this->x = x;
		this->x_ = x_;
		//~ this->x__ = x__;
	}
	
	Vec2f StateVector2D::getX(){
		return this->x;
	}
	
	Vec2f StateVector2D::getXDot(){
		return this->x_;
	}
	
	//~ Vec2f StateVector2D::getXDotDot(){
		//~ return this->x__;
	//~ }
	
	void StateVector2D::setX(Vec2f x){
		this->x = x;
	}
	
	void StateVector2D::setXDot(Vec2f x_){
		this->x_ = x_;
	}
	
	void StateVector2D::setX(double x1, double x2){
		this->x = Vec2f(x1, x2);
	}
	
	void StateVector2D::setXDot(double x_1, double x_2){
		this->x_ = Vec2f(x_1, x_2);
	}
	
	//~ void StateVector2D::setXDotDot(Vec2f x__){
		//~ this->x__ = x__;
	//~ }
	
	StateVector2D StateVector2D::operator+(StateVector2D other){
			StateVector2D result;
			Vec2f resultX(this->x.val[0]+other.x.val[0],this->x.val[1]+other.x.val[1]);
			Vec2f resultX_(this->x_.val[0]+other.x_.val[0],this->x_.val[1]+other.x_.val[1]);
			//~ Vec2f resultX__(this->x__.val[0]+other.x__.val[0],this->x__.val[1]+other.x__.val[1]);
			result.setX(resultX);
			result.setXDot(resultX_);
			//~ result.setXDotDot(resultX__);
			return result;
	}
	
	StateVector2D StateVector2D::operator-(StateVector2D other){
			StateVector2D result;
			Vec2f resultX(this->x.val[0]-other.x.val[0],this->x.val[1]-other.x.val[1]);
			Vec2f resultX_(this->x_.val[0]-other.x_.val[0],this->x_.val[1]-other.x_.val[1]);
			//~ Vec2f resultX__(this->x__.val[0]-other.x__.val[0],this->x__.val[1]-other.x__.val[1]);
			result.setX(resultX);
			result.setXDot(resultX_);
			//~ result.setXDotDot(resultX__);
			return result;
	}
	
	StateVector2D StateVector2D::operator*(StateVector2D other){
			StateVector2D result;
			Vec2f resultX(this->x.val[0]*other.x.val[0],this->x.val[1]*other.x.val[1]);
			Vec2f resultX_(this->x_.val[0]*other.x_.val[0],this->x_.val[1]*other.x_.val[1]);
			//~ Vec2f resultX__(this->x__.val[0]*other.x__.val[0],this->x__.val[1]*other.x__.val[1]);
			result.setX(resultX);
			result.setXDot(resultX_);
			//~ result.setXDotDot(resultX__);
			return result;
	}
