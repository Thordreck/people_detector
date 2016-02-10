

#include <math.h>
#include <Feature_10.h>
#include <iostream>
#include <Matrix.hh>

using namespace std;

//-------------------------------------------------------------
// boundary length
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
double Feature_10::calculateFeature(Segment *s)
{
	
	int n = s->num();
	double l = 0.0;
	Beam *b;
	double x,y;	
	double dx2, dy2;
	double x_p, y_p; 

	b = s->getElement(0);
	x_p = b->range * cos(b->angle);
	y_p = b->range * sin(b->angle);

	for(int i=1; i<n; i++) {
		b = s->getElement(i);
		x = b->range * cos(b->angle);
		y = b->range * sin(b->angle);

		dx2= (x-x_p)*(x-x_p);
		dy2= (y-y_p)*(y-y_p);
		
		l += sqrt(dx2 + dy2); 	

		x_p = x;
		y_p = y;
	}

#ifdef DEBUG
	cerr << "Feature_10::calculateFeature:  l: " << l << endl;
#endif 

	return l;
}
















