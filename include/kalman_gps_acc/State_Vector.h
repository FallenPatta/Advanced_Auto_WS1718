#ifndef STATEVECTOR2D_H_
#define STATEVECTOR2D_H_

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/core/matx.hpp"
#include "opencv2/core/types.hpp"
#include "opencv2/core/bufferpool.hpp"
#include "opencv2/core/mat.inl.hpp"

#include "kalman_gps_acc/definitions.h"

using namespace std;
using namespace cv;

class StateVector2D{
	private:
		Vec2f x;
		Vec2f x_;
		//~ Vec2f x__;
	public:
		StateVector2D();
		StateVector2D(Vec2f x, Vec2f x_);//, Vec2f x__);
		Vec2f getX();
		Vec2f getXDot();
		//~ Vec2f getXDotDot();
		void setX(Vec2f);
		void setXDot(Vec2f);
		void setX(double, double);
		void setXDot(double, double);
		//~ void setXDotDot(Vec2f);
		
		StateVector2D operator+(StateVector2D other);
		StateVector2D operator-(StateVector2D other);
		StateVector2D operator*(StateVector2D other);
};

#endif //STATEVECTOR2D_H_
