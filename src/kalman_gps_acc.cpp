#include "ros/ros.h"
#include "tf/tf.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Quaternion.h"
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <math.h>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/core/matx.hpp"
#include "opencv2/core/types.hpp"
#include "opencv2/core/bufferpool.hpp"
#include "opencv2/core/mat.inl.hpp"

#include "kalman_gps_acc/State_Vector.h"
#include "kalman_gps_acc/Kalman_Filter.h"
#include "kalman_gps_acc/definitions.h"

using namespace std;
using namespace cv;

ros::Subscriber acc_subscriber;
ros::Subscriber gps_subscriber;
ros::Subscriber vel_subscriber;
const double PI = 3.14159f;
const double RE = 6378000.0f;
const double f = 1.0f/298.257223563f;
const double a = 6378137.0f; 
double b = a*(1.0f-f);
double e = sqrt((pow(a,2.0f)-pow(b,2.0f))/(pow(a,2.0f)));


static int first_iteration =1;
float gps_start_position[3] = {0,0,0};

double gps_data[3];
sensor_msgs::Imu acc_data;
geometry_msgs::TwistStamped vel_data;

void gps_callback(const sensor_msgs::NavSatFix::ConstPtr & msg);
void acc_callback(const sensor_msgs::Imu::ConstPtr & msg);
void vel_callback(const geometry_msgs::TwistStamped::ConstPtr & msg);
//void LLA2ECEF(float * gps);
void sensor_fusion_acc_vel(double, Mat, StateVector2D);
double deg2rad(double);

// Kalman filter 1: vel+acc
//~ Matrix2D Sigma_acc, Sigma_vel;

double covariance_acc = 0.00001;
double covariance_vel = 0.001;


KalmanFilter1 kf = KalmanFilter1();
StateVector2D imu_vel_state = StateVector2D();
Mat imu_vel_covariance = Mat(2, 2, MAT_TYPE, double(0));
StateVector2D global_state = StateVector2D();
Mat global_covariance = Mat(2, 2, MAT_TYPE, double(0));

StateVector2D imu_state = StateVector2D();
double vX = 0;
double vY = 0;
double pX = 0;
double pY = 0;

StateVector2D gps_vel_state = StateVector2D();
double vel_pX = 0;
double vel_pY = 0;

StateVector2D gps_state = StateVector2D();

ofstream outputFile_gps_pos;
ofstream outputFile_acc_integral;
ofstream outputFile_vel_integral;
ofstream outputFile_vel_imu_kalman;
ofstream outputFile_gps_imu_kalman;

int main(int argc, char **argv) {
	
	if(sizeof(double) != 8){
		cerr << "ERROR: DOUBLE WORD IS NOT OF 8 BYTE WIDTH" << endl;
		return -1;
	}
	
	outputFile_gps_pos.open("/tmp/gps_pos.csv", ofstream::out | ofstream::trunc); //Done
	outputFile_gps_pos.close();
	outputFile_acc_integral.open("/tmp/imu_pos.csv", ofstream::out | ofstream::trunc); //Done
	outputFile_acc_integral.close();
	outputFile_vel_integral.open("/tmp/vel_pos.csv", ofstream::out | ofstream::trunc); //Done
	outputFile_vel_integral.close();
	outputFile_vel_imu_kalman.open("/tmp/vel_imu_pos.csv", ofstream::out | ofstream::trunc); //Done
	outputFile_vel_imu_kalman.close();
	outputFile_gps_imu_kalman.open("/tmp/gps_imu_pos.csv", ofstream::out | ofstream::trunc); //Done
	outputFile_gps_imu_kalman.close();
	
	ros::init(argc, argv, "kalman_gps_acc");
	ros::NodeHandle n;
	acc_subscriber = n.subscribe("/imu/data", 10, acc_callback);
	gps_subscriber = n.subscribe("/fix", 10, gps_callback);
	vel_subscriber = n.subscribe("/vel", 10, vel_callback);

	ros::spin();
}

void sensor_fusion_acc_vel(double deltaT, Mat sensor_covariance, StateVector2D measurement){
	
	StateVector2D predictedState = StateVector2D();
	kf.predictState(global_state, predictedState, deltaT);
	
	Mat predictedCovariance = Mat(2, 2, MAT_TYPE, double(0));
	kf.predictCovarianceWithEnv(global_covariance, predictedCovariance, sensor_covariance, deltaT);
	
	StateVector2D corrected = StateVector2D();
	Mat correctedCovariance = Mat(2,2, MAT_TYPE, double(0));
	
	kf.correctState(global_state, predictedState, corrected, measurement, deltaT, global_covariance, sensor_covariance, predictedCovariance, correctedCovariance);
	
	global_state = corrected;
	global_covariance = correctedCovariance;
	
	vX = global_state.getXDot().val[0];
	vY = global_state.getXDot().val[1];
	pX = global_state.getX().val[0];
	pY = global_state.getX().val[1];
	
	vel_pX = global_state.getX().val[0];
	vel_pY = global_state.getX().val[1];
	
	//Output
	outputFile_gps_imu_kalman.open("/tmp/gps_imu_pos.csv", ios::out|ios::app);
	outputFile_gps_imu_kalman << global_state.getX().val[0] << "," << global_state.getX().val[1] << "," << global_state.getXDot().val[0]<<","<<global_state.getXDot().val[1]<<endl;
	outputFile_gps_imu_kalman.close();
}

void vel_callback(const geometry_msgs::TwistStamped::ConstPtr & msg){
	static ros::Time last = ros::Time::now();
	static double velpX =0;
	static double velpY =0;
	
	ros::Time current = msg->header.stamp;
	if((current-last).toSec() < 0){
		last = current;
		return;
	}
	double deltaT = (current-last).toSec();
	
	vel_pX += msg->twist.linear.x * deltaT;
	vel_pY += msg->twist.linear.y * deltaT;
	
	velpX += msg->twist.linear.x * deltaT;
	velpY += msg->twist.linear.y * deltaT;
	
	outputFile_vel_integral.open("/tmp/vel_pos.csv", ofstream::out | ofstream::app);
	outputFile_vel_integral << velpX << "," << velpY << endl;
	outputFile_vel_integral.close();
	
	gps_vel_state.setX(Vec2f(vel_pX, vel_pY));
	gps_vel_state.setXDot(Vec2f(msg->twist.linear.x, msg->twist.linear.y));
	
	Mat sensor_covariance = Mat(2, 2, MAT_TYPE, double(0));
	
	sensor_covariance.at<double>(0,0) = 0.0001f;
	sensor_covariance.at<double>(0,1) = 0.0f;
	sensor_covariance.at<double>(1,0) = 0.0f;
	sensor_covariance.at<double>(1,1) = 0.0001f;
	
	sensor_fusion_acc_vel(deltaT, sensor_covariance, gps_vel_state);
	
	last = current;
}

void gps_callback(const sensor_msgs::NavSatFix::ConstPtr & msg){
	static ros::Time last = ros::Time::now();
	
	ros::Time current = msg->header.stamp;
	if((current-last).toSec() < 0){
		last = current;
		return;
	}
	double deltaT = (current-last).toSec();
	
	double N = a/(sqrt(1.0f-pow(e,2.0f)*pow(sin(msg->latitude),2.0f)));
	
	gps_data[0] = (N+msg->altitude)*cos(deg2rad(msg->latitude))*cos(deg2rad(msg->longitude))-gps_start_position[0];
	gps_data[1] = (N+msg->altitude)*cos(deg2rad(msg->latitude))*sin(deg2rad(msg->longitude))-gps_start_position[1];
	gps_data[2] = 0;
	// Set start position to (0,0,0) in first iteration
	if(first_iteration){
		for(int i=0;i<3;i++){
			gps_start_position[i] = gps_data[i];
			gps_data[i] =0;
		}
		first_iteration=0;
	}
	
	gps_state.setX(Vec2f(gps_data[0], gps_data[1]));
	gps_state.setXDot(gps_vel_state.getXDot());
	
	Mat sensor_covariance = Mat(2, 2, MAT_TYPE, double(0));
	
	sensor_covariance.at<double>(0,0) = msg->position_covariance[0];
	sensor_covariance.at<double>(0,1) = msg->position_covariance[1];
	sensor_covariance.at<double>(1,0) = msg->position_covariance[3];
	sensor_covariance.at<double>(1,1) = msg->position_covariance[4];
	
	sensor_fusion_acc_vel(deltaT, sensor_covariance, gps_state);
	
	outputFile_gps_pos.open("/tmp/gps_pos.csv", ofstream::out | ofstream::app);
	outputFile_gps_pos << gps_data[0] << "," << gps_data[1] << endl;
	outputFile_gps_pos.close();
	last = current;
}

void acc_callback(const sensor_msgs::Imu::ConstPtr & msg){
	static double imuX = 0;
	static double imuY = 0;
	static double imuVX = 0;
	static double imuVY = 0;
	static ros::Time last = ros::Time::now();
	ros::Time current = msg->header.stamp;
	if((current-last).toSec() < 0){
		last = current;
		return;
	}
	
	
	// TODO: die imu Nachrichten müssen so gedreht werden, dass die x und y Anteile auch in der XY-Ebene liegen...
	tf::Quaternion rotated = tf::Quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
	
	tf::Matrix3x3 m(rotated);
	
	double roll, pitch, yaw;
	m.getRPY(roll,pitch,yaw);
	
	tf::Transform topTF1 = tf::Transform(m);
	
	tf::Vector3 accVec = tf::Vector3(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
	
	tf::Vector3 transformed = topTF1(accVec);
	
	double deltaT = (current-last).toSec();
	
	pX += vX*deltaT + transformed.getX() * 0.5f *deltaT*deltaT;
	pY += vY*deltaT + transformed.getY() * 0.5f *deltaT*deltaT;
	
	vX += transformed.getX()*(current-last).toSec();
	vY += transformed.getY()*(current-last).toSec();
	
	imu_state.setX(Vec2f(pX, pY));
	imu_state.setXDot(Vec2f(vX, vY));
	
	//For Output
	imuX += vX*deltaT + transformed.getX() * 0.5f *deltaT*deltaT;
	imuY += vY*deltaT + transformed.getY() * 0.5f *deltaT*deltaT;
	
	imuVX += transformed.getX()*(current-last).toSec();
	imuVY += transformed.getY()*(current-last).toSec();
	
	outputFile_acc_integral.open("/tmp/imu_pos.csv", ofstream::out | ofstream::app);
	outputFile_acc_integral << imuX << "," << imuY << endl;
	outputFile_acc_integral.close();
	
	last = current;
	
	Mat sensor_covariance = Mat(2, 2, MAT_TYPE, double(0));
	
	sensor_covariance.at<double>(0,0) = msg->linear_acceleration_covariance[0];
	sensor_covariance.at<double>(0,1) = msg->linear_acceleration_covariance[1];
	sensor_covariance.at<double>(1,0) = msg->linear_acceleration_covariance[3];
	sensor_covariance.at<double>(1,1) = msg->linear_acceleration_covariance[4];
	
	sensor_fusion_acc_vel(deltaT, sensor_covariance, imu_state);
}

double deg2rad(double ang_deg){
	return ang_deg*PI/180.0;
}
