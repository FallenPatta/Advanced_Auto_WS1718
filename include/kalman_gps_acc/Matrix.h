#ifndef MATRIX2D_H_
#define MATRIX2D_H_

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include "kalman_gps_acc/State_Vector.h"

using namespace std;
using namespace cv;

class Matrix2D: public Mat  {
	private:
		
	public:
	 Matrix2D();
	 //~ Matrix2D(double,double,double,double);
	 
	//~ Vector2D operator*(Vector2D vect);
	
	//~ StateVector2D operator*(StateVector2D vect);
	
	//~ Matrix2D operator~();
	
};

#endif // MATRIX2D_H_
