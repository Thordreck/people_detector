

#include <math.h>
#include <Feature_2.h>

//---------------------------------
// Standard deviation
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
double Feature_2::calculateFeature(Segment *s)
{
	double value;
	
	// center of gravity
	double x_c, y_c;
	double x, y;	
	double n;
	Beam *b;


	x_c = 0.0;
	y_c = 0.0;
	for (int i=0; i < s->num(); i++) {
		b = s->getElement(i);
		x = b->range * cos(b->angle);
		y = b->range * sin(b->angle); 

		x_c = x_c + x;
	 	y_c = y_c + y;
	}	

	n = (double)( s->num() );
	x_c = x_c / n;
	y_c = y_c / n;
	

	double diff = 0.0;
	double d_x, d_y;
	for (int i=0; i < s->num(); i++) {
		b = s->getElement(i);
		x = b->range * cos(b->angle);
		y = b->range * sin(b->angle); 

		d_x = x - x_c;
		d_y = y - y_c;
		diff = diff + sqrt( d_x*d_x + d_y*d_y );
	}	

	value = sqrt( diff / (n - 1.0) ); 	

	return value; 	
}
















