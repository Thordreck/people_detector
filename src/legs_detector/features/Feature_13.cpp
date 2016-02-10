

#include <math.h>
#include <Feature_13.h>
#include <iostream>
#include <Matrix.hh>

using namespace std;

//-------------------------------------------------------------
// Mean angular difference
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

double setangletorange(double a, double mina) 
{
	double ar;

	while (a >= mina + 2.0*M_PI ) {
		a = a - 2.0*M_PI;
	}
	while( a < mina ) {
		a = a + 2.0*M_PI;
	}
	ar = a;

	return ar;
}

double diffangleunwrap(double a1, double a2) 
{

	// Normalize angles a1 and a2
	a1 = setangletorange(a1,0);  
	a2 = setangletorange(a2,0);
	
	// Take difference and unwrap
	double delta = a1 - a2;
	if (a1 > a2) {
		while (delta >  M_PI) {
			delta = delta - 2.0*M_PI;
		}
	}
	else
	if (a2 > a1) {
		while (delta < -M_PI) {
			delta = delta + 2.0*M_PI;
		}
	}

	return delta;	
}


double Feature_13::calculateFeature(Segment *s)
{
	
	int n = s->num();
	Beam *b1, *b2, *b3;
	double x1, x2, x3, y1, y2, y3;
	double phi1, phi2;
	double dalpha=0.0;

	for(int i=1; i<n-1; i++) {

		b1 = s->getElement(i-1);
		x1 = b1->range * cos(b1->angle);
		y1 = b1->range * sin(b1->angle);

		b2 = s->getElement(i);
		x2 = b2->range * cos(b2->angle);
		y2 = b2->range * sin(b2->angle);

		b3 = s->getElement(i+1);
		x3 = b3->range * cos(b3->angle);
		y3 = b3->range * sin(b3->angle);

		phi1 = atan2(y2-y1,x2-x1);
		phi2 = atan2(y3-y2,x3-x2);	

		dalpha += diffangleunwrap(phi1,phi2);	
	}
	
	dalpha = dalpha / (double)(n-1); 

#ifdef DEBUG
	cerr << "Feature_13::calculateFeature:  dalpha: " << dalpha << endl;
#endif 

	return dalpha;
}
















