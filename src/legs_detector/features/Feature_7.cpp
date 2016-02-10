

#include <math.h>
#include <Feature_7.h>
#include <iostream>

using namespace std;

//-------------------------------------------------------------
// Linearity
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
//
// fit a line using the method in :
// 15.2 Fitting Data to a Straight Line
// NUMERICAL RECIPES IN C. Second edition, 1992
// pp. 665-666
// ISBN 0-521-43108-5
//
//--------------------------------------------------------------

double SQR(double a) {
	if ( a == 0.0  ) {
		return 0.0;
	} 
	
	return a*a;
}

double Feature_7::calculateFeature(Segment *s)
{
	//-------------------------
	// fit the line y = a + bx 
	//-------------------------

	// prepare data
	int ndata = s->num();
	double *x = new double[ndata+1];
	double *y = new double[ndata+1];
	
	Beam *be;
	int i;
	for(i=1; i<=ndata; i++) {
		be = s->getElement(i-1); // indexes start at 1 in "Numerical Recipes"
		x[i] = be->range * cos(be->angle);
		y[i] = be->range * sin(be->angle); 
	}	
	
	// start method
	double b=0, a ;
	double sx=0, sy=0, sxoss, t, st2=0, ss;
	for(i=1; i<=ndata; i++) {
		sx += x[i];
		sy += y[i];
	}
	ss = ndata;
	sxoss = sx/ss;

	for(i=1; i<=ndata; i++) {
		t = x[i]-sxoss;
		st2 += t*t; 
		b += t*y[i];
	}
	
	st2 += 0.000000000000000000001; // just in case it is zero 
	b /= st2;
	a = (sy-sx*b)/ss;
	


	//-------------------------------------
	// we already have the line y = a + bx
	// now we calculate the residuals
	//
	// The difference, d, between a point p1(x1,y1) and the line Ax+By+C=0 is:
	//
	// d = abs( A*x1 + B*y1 + C) / sqrt( A*A + B*B )
	//
	// In our case the line is define as y = a + bx, then:
	// 	
	//  Ax + By + C = 0 = bx - y + a
	//  
	//  A=b, B=-1, C=a 
	//------------------------------------
	
	double A, B, C;
	A = b;
	B = -1;
	C = a;
	
	double sres=0.0;
	for(i=1; i<=ndata; i++) {
		sres += fabs( A*x[i] + B*y[i] + C ) / sqrt(A*A + B*B);	
	}
	
#ifdef DEBUG
	cerr << "Feature_7::calculateFeature:  a: " << a << " b: " << b << " sres: " << sres << endl;
#endif 
	
	delete x; 
	delete y;

	return sres;
}
















