#ifndef MATRIX2D_H_
#define MATRIX2D_H_

#include "kalman_gps_acc/Vector.h"

class Matrix2D{
	private:
	 double m[2][2];
	 
	public:
	 Matrix2D();
	 void setEntry(int row, int col, double val);
	 Matrix2D multiply(Matrix2D m2);
	 Matrix2D add(Matrix2D m2);
	 Matrix2D itentityMatrix();
	 Vector2D multiply(Vector2D v);
	 Matrix2D scalarMultiply(double val);
	 void printMatrix();
};

#endif // MATRIX2D_H_
