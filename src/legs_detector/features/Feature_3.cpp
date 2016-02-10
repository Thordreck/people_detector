

#include <math.h>
#include <Feature_3.h>
#include <iostream>

using namespace std;

//----------------------------------------
// compar
//----------------------------------------
int compar(const void *a, const void *b) {
	double *v1 = (double *)a;
	double *v2 = (double *)b;	

	if ( *v1 < *v2 ) {
		return -1;
	} 
	else
	if ( *v1 == *v2 ) {
		return 0;
	} 
	else
	if ( *v1 > *v2 ) {
		return 1;
	} 

	return 0;
}

//---------------------------------
// Mean average deviation from median
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
double Feature_3::calculateFeature(Segment *s)
{
	double value;
	
	// center of gravity
	double x_m, y_m;
	double x, y;	
	Beam *b;
	double a_m, r_m;
	int index;

	int n = s->num();

	// arrays of polar coordinates
	double *rangeList = new double[n];
	double *angleList = new double[n];
	
	for (int i=0; i<n; i++) {
		b = s->getElement(i);
		rangeList[i] = b->range;
		angleList[i] = b->angle;
	}	

	// order the elements with respect the range and angle values
	qsort(rangeList, n, sizeof(double), compar);
	qsort(angleList, n, sizeof(double), compar);

	// calculate the medians
	x_m = 0.0;
	y_m = 0.0;
	if ( (n % 2) == 0  ) {
		// even
		index = (n / 2); 
		index --; // arrays start at cero in C		

		a_m = ( angleList[index] + angleList[index+1] ) / 2.0;
		r_m = ( rangeList[index] + rangeList[index+1] ) / 2.0;

		x_m = r_m * cos(a_m);
		y_m = r_m * sin(a_m); 		
	}
	else {
		// odd 
		index = (n +1) / 2;
		index --; // arrays start at cero in C		

		a_m = angleList[index];
		r_m = rangeList[index];

		x_m = r_m * cos(a_m);
		y_m = r_m * sin(a_m); 		
	}

#ifdef DEBUG
	cerr << "Feature_3::calculateFeature:  a_m: " << a_m << " r_m: " << r_m << endl;
	cerr << "Feature_3::calculateFeature:  x_m: " << x_m << " y_m: " << y_m << endl;
#endif 
	
	// calculate final value
	double diff = 0.0;
	double d_x, d_y;
	for (int i=0; i < n; i++) {
		b = s->getElement(i);
		x = b->range * cos(b->angle);
		y = b->range * sin(b->angle); 

		d_x = x - x_m;
		d_y = y - y_m;
		diff = diff + sqrt( d_x*d_x + d_y*d_y );
	}	

	value =  diff / (double)n; 	

	delete rangeList;
	delete angleList;	

	return value; 	
}
















