#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include "kalman_gps_acc/State_Vector.h"
#include "kalman_gps_acc/definitions.h"
#include <stdio.h>

using namespace std;
using namespace cv;

class KalmanFilter1{
	private:
		//~ void predictPosition(Vec2f& oldPos, Vec2f& newPos, Vec2f& acceleration, Vec2f& speed, double deltaT);
		//~ void predictVelocity(Vec2f& oldVel, Vec2f& newVel, Vec2f& acceleration, double deltaT);
		Mat getPredictionMatrix(double deltaT);
		Mat getControlMatrix(double deltaT);
		Mat getCorrelationMatrix(double);
	public:
		StateVector2D matrixStateMultiply(Mat, StateVector2D);
		void predictCovariance(Mat oldCov, Mat& newCov, double deltaT);
		void predictCovarianceWithEnv(Mat oldCov, Mat& newCov, Mat env, double deltaT);
		void predictState(StateVector2D oldState, StateVector2D& predictState, double deltaT);
		void predictStateWithControl(StateVector2D oldState, StateVector2D& predictState, StateVector2D controlState, double deltaT);
		void correctState(StateVector2D aprioriState, StateVector2D predictState, StateVector2D& correctedState
							, StateVector2D measurements, double deltaT
							, Mat aprioriCov, Mat sensorCov, Mat predictedCov, Mat& correctedCov);
		KalmanFilter1();
};


#endif //KALMAN_FILTER_H_
