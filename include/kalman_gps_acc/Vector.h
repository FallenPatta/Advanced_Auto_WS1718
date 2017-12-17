#ifndef GEOM_VECTOR_H_
#define GEOM_VECTOR_H_

#include <stdio.h>

class Vector2D{
	private:
	
	public:
		Vector2D();
		Vector2D(double x, double y);
		void setEntry(int row, double val);
		double v[2];
		void printVector();
		double getY();
		double getX();
		void setY(double);
		void setX(double);
		
		Vector2D operator+(Vector2D other);
		Vector2D operator-(Vector2D other);
		Vector2D operator*(Vector2D other);
		Vector2D operator/(Vector2D other);
		
		Vector2D operator*(double other);
		Vector2D operator/(double other);
};


#endif //GEOM_VECTOR_H_
