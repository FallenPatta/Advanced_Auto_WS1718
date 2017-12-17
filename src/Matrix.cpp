#include "kalman_gps_acc/Matrix.h"
#include "kalman_gps_acc/State_Vector.h"

#include <stdio.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;


Matrix2D::Matrix2D(){
	this = new Mat(2,2,CV_64F,double(0));
}

//~ Matrix2D::Matrix2D(double a,double b,double c,double d){
	//~ m[0][0] = a;
	//~ m[0][1] = b;
	//~ m[1][0] = c;
	//~ m[1][1] = d;
//~ }
	
	//~ StateVector2D Matrix2D::operator*(StateVector2D vect){
		//~ StateVector2D result;
		//~ Vector2D xVec (this->m[0][0], this->m[0][1]);
		//~ Vector2D x_Vec (this->m[1][0], this->m[1][1]);
	
		//~ result.setX(xVec * vect.getX());
		//~ result.setXDot(x_Vec * vect.getXDot());

		//~ return result;
	//~ }
	
	//~ Matrix2D Matrix2D::operator~(){
		//~ Matrix2D result(this->m[0][0], this->m[1][0],
						//~ this->m[0][1], this->m[1][1]);
		//~ return result;
	//~ }
