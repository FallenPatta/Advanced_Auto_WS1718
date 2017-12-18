#include "kalman_gps_acc/Kalman_Filter.h"
#include "kalman_gps_acc/State_Vector.h"
#include "kalman_gps_acc/definitions.h"

#include "math.h"
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

KalmanFilter1::KalmanFilter1(){
	
}

/*
 * For: State(position, velocity) to State(position, velocity)
 * Literatur: Übergangsmatrix A
 */
Mat KalmanFilter1::getPredictionMatrix(double deltaT){
	Mat predictMatrix = Mat(2, 2, MAT_TYPE, double(0));
	predictMatrix.at<double>(0,0) = 1;
	predictMatrix.at<double>(0,1) = deltaT;
	predictMatrix.at<double>(1,0) = 0;
	predictMatrix.at<double>(1,1) = 1;
	return predictMatrix;
}

/*
 * For: State(velocity, acceleration) to State(position, velocity)
 * Literatur: Steuerungsmatrix B
 */
Mat KalmanFilter1::getControlMatrix(double deltaT){
	Mat predictMatrix = Mat(2, 2, MAT_TYPE, double(0));
	predictMatrix.at<double>(0,0) = 1;
	predictMatrix.at<double>(0,1) = deltaT;
	predictMatrix.at<double>(1,0) = 0;
	predictMatrix.at<double>(1,1) = 1;
	return predictMatrix;
}

/*
 * For: State(velocity, acceleration) to State(position, velocity)
 * Literatur: Steuerungsmatrix H
 */
Mat KalmanFilter1::getCorrelationMatrix(double deltaT){
	Mat predictMatrix = Mat(2, 2, MAT_TYPE, double(0));
	predictMatrix.at<double>(0,0) = 1;
	predictMatrix.at<double>(0,1) = deltaT;
	predictMatrix.at<double>(1,0) = 0;
	predictMatrix.at<double>(1,1) = 1;
	return predictMatrix;
}

/*
 * For: State(velocity, acceleration) to State(position, velocity)
 * Literatur: Steuerungsmatrix H
 */
Mat KalmanFilter1::getHMatrix(double deltaT){
	Mat predictMatrix = Mat(2, 2, MAT_TYPE, double(0));
	predictMatrix.at<double>(0,0) = 1;
	predictMatrix.at<double>(0,1) = 0;
	predictMatrix.at<double>(1,0) = 0;
	predictMatrix.at<double>(1,1) = 1;
	return predictMatrix;
}

/*
 * oldState : old State
 * predictState : predicted State
 * controls : Beispiel für Beschleunigungsmessung -> StateVector2D(Vector2D(0,0), Vector2D(accX, accY))
 * deltaT : timestep
 * 
 * Notiz: oldState und predictState sollten im Vector X den Wert des zu schätzenden Zustands und im Vector X_ den Wert der Ableitung enthalten (z.B. v und a)
 */
void KalmanFilter1::predictState(StateVector2D oldState, StateVector2D& predictState, double deltaT){
	Mat predictMatrix = getPredictionMatrix(deltaT);
	predictState = matrixStateMultiply(predictMatrix, oldState);
}

void KalmanFilter1::predictStateWithControl(StateVector2D oldState, StateVector2D& predictState, StateVector2D controlState, double deltaT){
	Mat predictMatrix = getPredictionMatrix(deltaT);
	Mat controlMatrix = getControlMatrix(deltaT);
	predictState = matrixStateMultiply(predictMatrix, oldState);
	StateVector2D controlValue = matrixStateMultiply(controlMatrix, controlState);
	
	predictState = predictState + controlValue;
}

void KalmanFilter1::predictCovariance(Mat oldCov, Mat& predictCov, double deltaT){
	Mat predictMatrix = getPredictionMatrix(deltaT);
	cv::Mat transposed =  cv::Mat(predictMatrix.cols, predictMatrix.rows, MAT_TYPE);
	transpose(predictMatrix, transposed);
	predictCov = predictMatrix * oldCov * transposed;
}

void KalmanFilter1::predictCovarianceWithEnv(Mat oldCov, Mat& predictCov, Mat envCov, double deltaT){
	Mat predictMatrix = getPredictionMatrix(deltaT);
	cv::Mat transposed =  cv::Mat(predictMatrix.cols, predictMatrix.rows, MAT_TYPE);
	transpose(predictMatrix, transposed);
	predictCov = predictMatrix * oldCov * transposed;
	predictCov = predictCov + envCov;
}

/*
 * cv::Mat ist row mayor so to get a value at (x, y) you use: at<double>(row(y), column(x))
 */
StateVector2D KalmanFilter1::matrixStateMultiply(Mat m, StateVector2D v){
	StateVector2D result;
	result.setX(v.getX().val[0] * m.at<double>(0,0) + v.getXDot().val[0] * m.at<double>(0,1), v.getX().val[1] * m.at<double>(0,0) + v.getXDot().val[1] * m.at<double>(0,1));
	result.setXDot(v.getX().val[0] * m.at<double>(1,0) + v.getXDot().val[0] * m.at<double>(1,1), v.getX().val[1] * m.at<double>(1,0) + v.getXDot().val[1] * m.at<double>(1,1));
	
	return result;
}

void KalmanFilter1::correctState(StateVector2D aprioriState, StateVector2D predictState, StateVector2D& correctedState
					, StateVector2D measurement, double deltaT, Mat aprioriCov, Mat sensorCov, Mat predictedCov, Mat& correctedCov){
	//Berechnung von K
	//Mat kalman_gain = predictedCov * inv;
	
	//Matrix H
	Mat hMat = getHMatrix(deltaT);
	
	// H^T
	cv::Mat transposed =  cv::Mat(hMat.cols, hMat.rows, MAT_TYPE);
	transpose(hMat, transposed);
	//Berechnung von K'
	Mat inv = (hMat * predictedCov * transposed + sensorCov).inv();
	Mat kalman_gain_ = predictedCov * transposed * inv;
		
	StateVector2D measuredState = measurement;
	
	correctedState = predictState + matrixStateMultiply(kalman_gain_, measuredState - matrixStateMultiply(hMat, predictState));
	correctedCov = predictedCov - (kalman_gain_ * hMat * predictedCov);
}
