#include "ros/ros.h"
#include "tf/tf.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/TwistStamped.h"
#include <stdio.h>
#include <iostream>
#include <math.h>

#include "kalman_gps_acc/Vector.h"
#include "kalman_gps_acc/Matrix.h"
#include "kalman_gps_acc/Kalman_Filter.h"

using namespace std;

ros::Subscriber acc_subscriber;
ros::Subscriber gps_subscriber;
ros::Subscriber vel_subscriber;
const double PI = 3.14159;
const double RE = 6378000;


static int first_iteration =1;
float gps_start_position[3] = {0,0,0};

double gps_data[3];
sensor_msgs::Imu acc_data;
geometry_msgs::TwistStamped vel_data;

void gps_callback(const sensor_msgs::NavSatFix::ConstPtr & msg);
void acc_callback(const sensor_msgs::Imu::ConstPtr & msg);
void vel_callback(const geometry_msgs::TwistStamped::ConstPtr & msg);
//void LLA2ECEF(float * gps);
void sensor_fusion_acc_vel();

// Kalman filter 1: vel+acc
Matrix2D Sigma_acc, Sigma_vel;

double covariance_acc = 0.1;
double covariance_vel = 0.01;



int main(int argc, char **argv) {
	ros::init(argc, argv, "kalman_gps_acc");
	ros::NodeHandle n;
	acc_subscriber = n.subscribe("/imu/data", 10, acc_callback);
	gps_subscriber = n.subscribe("/fix", 10, gps_callback);
	vel_subscriber = n.subscribe("/vel", 10, vel_callback);

	// Init Kalman filter gain matrices
	
	Sigma_acc.setEntry(0,0,covariance_acc);
	Sigma_acc.setEntry(1,1,covariance_acc);
	Sigma_acc.printMatrix();
	
	Sigma_vel.setEntry(0,0,covariance_vel);
	Sigma_vel.setEntry(1,1,covariance_vel);
	Sigma_vel.printMatrix();
	
	
	

	//Matrix2D m1;
	//Matrix2D m2;
	//m1.setEntry(0,0,1);
	//m1.setEntry(1,1,1);
	
	//m2.setEntry(0,0,1);
	//m2.setEntry(0,1,2);
	//m2.setEntry(1,0,3);
	//m2.setEntry(1,1,4);
	
	//Matrix2D m3 = m2.add(m1);
	//m3.printMatrix();
	
	//Vector2D v1;
	//v1.setEntry(0,1);
	//v1.setEntry(1,1);
	
	//Vector2D v2 = m1.multiply(v1);
	//v2.printVector();
	//kalman_filter();
	ros::spin();
}

void sensor_fusion_acc_vel(){
}

//void LLA2ECEF(float * gps){
	//gps[0] = (RE+gps_data.altitude)*cos(gps_data.latitude)*cos(gps_data.longitude)-gps_start_position[0];
	//gps[1] = (RE+gps_data.altitude)*cos(gps_data.latitude)*sin(gps_data.longitude)-gps_start_position[1];
	//gps[2] = (RE+gps_data.altitude)*sin(gps_data.latitude)-gps_start_position[2];
	//if(first_iteration){
		//for(int i=0;i<3;i++){
			//gps_start_position[i] = gps[i];
			//gps[i] =0;
		//}
		//first_iteration=0;
	//}
	//printf("GPS_ECEF: %f, %f, %f\n", gps[0], gps[1], gps[2]);
//}

void vel_callback(const geometry_msgs::TwistStamped::ConstPtr & msg){
	ros::Time current = msg->header.stamp;
	vel_data.twist.linear.x = msg->twist.linear.x;
	vel_data.twist.linear.y = msg->twist.linear.y;
	printf("VEL: %f, %f\n", msg->twist.linear.x, msg->twist.linear.y);
	std::cout << current << std::endl;
}

void gps_callback(const sensor_msgs::NavSatFix::ConstPtr & msg){
	ros::Time current = msg->header.stamp;
	gps_data[0] = (RE+msg->altitude)*cos(msg->latitude)*cos(msg->longitude)-gps_start_position[0];
	gps_data[1] = (RE+msg->altitude)*cos(msg->latitude)*sin(msg->longitude)-gps_start_position[1];
	gps_data[2] = (RE+msg->altitude)*sin(msg->latitude)-gps_start_position[2];
	// Set start position to (0,0,0) in first iteration
	if(first_iteration){
		for(int i=0;i<3;i++){
			gps_start_position[i] = gps_data[i];
			gps_data[i] =0;
		}
		first_iteration=0;
	}
	printf("GPS_ECEF: %f, %f, %f\n", gps_data[0], gps_data[1], gps_data[2]);
	
	//printf("GPS: %f, %f, %f\n", gps_data.latitude, gps_data.longitude, gps_data.altitude);
	std::cout << current << std::endl;
}

void acc_callback(const sensor_msgs::Imu::ConstPtr & msg){
	ros::Time current = msg->header.stamp;
	acc_data.linear_acceleration.x = msg->linear_acceleration.x;
	acc_data.linear_acceleration.y = msg->linear_acceleration.y;
	acc_data.linear_acceleration.z = msg->linear_acceleration.z;
	
	//acc_data.linear_acceleration_covariance[0] = msg->linear_acceleration_covariance[0];
	//acc_data.linear_acceleration_covariance[1] = msg->linear_acceleration_covariance[1];
	//acc_data.linear_acceleration_covariance[3] = msg->linear_acceleration_covariance[3];
	//acc_data.linear_acceleration_covariance[4] = msg->linear_acceleration_covariance[4];
	sensor_fusion_acc_vel();
	printf("ACC: %f, %f, %f\n", acc_data.linear_acceleration.x, acc_data.linear_acceleration.y,acc_data.linear_acceleration.z);
	std::cout << current << std::endl;
}






double deg2rad(double ang_deg){
	return ang_deg*PI/180.0;
}
