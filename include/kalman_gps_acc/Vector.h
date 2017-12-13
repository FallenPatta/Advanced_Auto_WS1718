#ifndef GEOM_VECTOR_H_
#define GEOM_VECTOR_H_

#include <stdio.h>

class Vector2D{
	private:
	
	public:
	Vector2D();
	void setEntry(int row, double val);
	double v[2];
	void printVector();
	double getY();
	double getX();
	
};


#endif //GEOM_VECTOR_H_
