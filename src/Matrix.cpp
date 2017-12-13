#include "kalman_gps_acc/Matrix.h"
#include "kalman_gps_acc/Vector.h"
#include <stdio.h>


Matrix2D::Matrix2D(){
	for(int y=0;y<2;y++)
		for(int x=0;x<2;x++)
			m[y][x] = 0;
}

void Matrix2D::setEntry(int row, int col, double val){
	m[row][col] = val;
}

Matrix2D Matrix2D::multiply(Matrix2D m2){
	Matrix2D result;
	
	result.m[0][0] = this->m[0][0]*m2.m[0][0]+this->m[0][1]*m2.m[1][0];
	result.m[0][1] = this->m[0][0]*m2.m[0][1]+this->m[0][1]*m2.m[1][1];
	
	result.m[1][0] = this->m[1][0]*m2.m[0][0]+this->m[1][1]*m2.m[1][0];
	result.m[1][1] = this->m[1][0]*m2.m[0][1]+this->m[1][1]*m2.m[1][1];

	return result;
}

Vector2D Matrix2D::multiply(Vector2D v){
	Vector2D result;
	
	result.v[0] = this->m[0][0]*v.v[0]+this->m[0][1]*v.v[1];
	result.v[0] = this->m[1][0]*v.v[0]+this->m[1][1]*v.v[1];

	return result;
}

void Matrix2D::printMatrix(){
	
	printf("[");
	for(int y=0;y<2;y++){
		for(int x=0;x<2;x++){
			printf("%f ", this->m[y][x]);
		}
		if(y==0)
			printf("\n");
	}
	printf("]\n");
}

Matrix2D Matrix2D::add(Matrix2D m2){
	Matrix2D result;
	for(int y=0;y<2;y++)
		for(int x=0;x<2;x++)
			result.m[y][x] = this->m[y][x]+m2.m[y][x];
	return result;
}

Matrix2D Matrix2D::scalarMultiply(double val){
	Matrix2D result;
	for(int y=0;y<2;y++)
		for(int x=0;x<2;x++)
			result.m[y][x] = this->m[y][x]*val;
	return result;
}


Matrix2D Matrix2D::itentityMatrix(){
	Matrix2D result;
	result.setEntry(0,0,1);
	result.setEntry(1,1,1);
	return result;
}
