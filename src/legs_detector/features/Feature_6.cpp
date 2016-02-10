

#include <math.h>
#include <Feature_6.h>
#include <iostream>

using namespace std;

//---------------------------------
// Width
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
//---------------------------------
double Feature_6::calculateFeature(Segment *s)
{
	double x_1, y_1, x_2, y_2;
	int n=s->num();

	Beam *b1 = s->getElement(0);
	Beam *b2 = s->getElement(n-1);
	
	x_1 = b1->range * cos (b1->angle);
	y_1 = b1->range * sin (b1->angle);
	
	x_2 = b2->range * cos (b2->angle);
	y_2 = b2->range * sin (b2->angle);
	 
	double dx = x_1 - x_2;
	double dy = y_1 - y_2;

	double diff = sqrt (dx*dx + dy*dy); 

	return diff;
}
















