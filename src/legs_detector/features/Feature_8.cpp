

#include <math.h>
#include <Feature_8.h>
#include <Feature_6.h>
#include <iostream>
#include <Matrix.hh>

using namespace std;

//-------------------------------------------------------------
// Circularity
//
// See:
//@InProceedings{arras2007icra,
//  title    =   {Using Boosted Features for the Detection of People in {2D} Range Data},
//  author   =   {Kai O. Arras and Oscar Martinez Mozos and Wolfram Burgard},
//  booktitle  = {Proceedings of the IEEE International Conference on Robotics and Automation},
//  year     =   {2007},
//  pages		=  {3402--3407},	
//  url      =   {http://www.informatik.uni-freiburg.de/~omartine/publications/arras2007icra.pdf},
//}	
//
//--------------------------------------------------------------
double Feature_8::calculateFeature(Segment *s)
{
	int n = s->num();

	Beam *b;
	double x_c, y_c, r;

	//check the lenght
	Feature_6 f6;

	double l = f6.calculateFeature(s);	
	if ( l==0.0 ) {
#ifdef DEBUG
	cerr << "Feature_8::calculateFeature:  length==0, res=0.0" << endl;
#endif 
		return 0.0;	
	}
	if ( n==3 ) {
// 			  A = X(2) - X(1);
// 			B = Y(2) - Y(1);
// 			C = X(3) - X(1);
// 			D = Y(3) - Y(1);
// 			E = A*(X(1) + X(2)) + B*(Y(1) + Y(2));
// 			F = C*(X(1) + X(3)) + D*(Y(1) + Y(3));
// 			G = 2*(A*(Y(3) - Y(2))-B*(X(3) - X(2)));
// 			xc = (D*E - B*F) / G;
// 			yc = (A*F - C*E) / G;
// 			r = sqrt((X(1) - xc)^2 + (Y(1) - yc)^2);
		b = s->getElement(0);
		double x1 = b->range * cos(b->angle);
		double y1 = b->range * sin(b->angle);

		b = s->getElement(1);
		double x2 = b->range * cos(b->angle);
		double y2 = b->range * sin(b->angle);
		
		b = s->getElement(2);
		double x3 = b->range * cos(b->angle);
		double y3 = b->range * sin(b->angle);

		double a = x2 - x1;
		double b = y2 - y1;
		double c = x3 - x1;
		double d = y3 - y1;
		double e = a * (x1 + x2) + b * (y1 + y2);
		double f = c * (x1 + x3) + d * (y1 + y3); 
		double g = 2 * (a * (y3 - y2) - b * (x3 - x2) );
		x_c = (d*e - b*f) / g;
		y_c = (a*f - c*e) / g;
		r = sqrt( (x1-x_c)*(x1-x_c) + (y1-y_c)*(y1-y_c) );
	} 
	else {
		// Matrix A
		Matrix A(n, 3);
		Matrix B(n, 1);
		Matrix sol(3, 1); //sol = (x_c  y_c  x_c**2 -y_c**2-r) 

		double x, y;
		for (int i=0; i<n; i++) {
			b = s->getElement(i);
			x = b->range * cos(b->angle);
			y = b->range * sin(b->angle);
		
			A[i][0] = -2*x;
			A[i][1] = -2*y;
			A[i][2] = 1;
			
			B[i][0] = -(x*x + y*y);
		}
		
		//solve
		// sol = (A'A)**-1 A' B
		sol = A.Pinv() * B;

		//Extract circle center and radius
		// xc = x(1); yc = x(2);
		// r = sqrt(-x(3)+xc*xc+yc*yc);
		
		x_c = sol[0][0];
		y_c = sol[1][0];
		double x3 = sol[2][0];
		r = sqrt( -x3 + x_c*x_c + y_c*y_c );
	}	

	// calculate residuals 
	double res=0.0;
	double diff;
	double x,y;
	for (int i=0; i<n; i++) {
		b = s->getElement(i);
		x = b->range * cos(b->angle);
		y = b->range * sin(b->angle);

		diff = r - sqrt( (x-x_c)*(x-x_c) + (y-y_c)*(y-y_c));
		res += diff*diff; 
 	}	

#ifdef DEBUG
	cerr << "Feature_8::calculateFeature:  x_c: " << x_c << " y_c: " << y_c << " r: " << r << " res:" << res << endl;
#endif 

	this->radius = r;

	return res;
}
















