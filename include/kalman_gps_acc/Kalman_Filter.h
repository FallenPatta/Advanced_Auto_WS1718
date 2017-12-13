#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "kalman_gps_acc/Vector.h"
#include "kalman_gps_acc/Matrix.h"
#include <stdio.h>

class KalmanFilter{
	private:
		//~ Vector2D * X; //Current Position
		//~ Matrix2D A; //Next Position from Estimation Matrix
		
		//~ Matrix2D B; //Position Correction from Sensor Data
		//~ Vector2D U; //Sensor Input
	
	public:
	KalmanFilter();
};


#endif //KALMAN_FILTER_H_
