

#include <math.h>
#include <Feature_6.h>
#include <Feature_8.h>
#include <Feature_9.h>
#include <iostream>
#include <Matrix.hh>

using namespace std;

//-------------------------------------------------------------
// radius
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
double Feature_9::calculateFeature(Segment *s)
{
	Feature_8 f8;

	//check the lenght
	Feature_6 f6;

	double l = f6.calculateFeature(s);	
	if ( l==0.0 ) {
#ifdef DEBUG
	cerr << "Feature_9::calculateFeature:  length==0, res=0.0" << endl;
#endif 
		return 0.0;	
	}


	f8.calculateFeature(s);
	
#ifdef DEBUG
	cerr << "Feature_9::calculateFeature:  radius: " << f8.radius << endl;
#endif 

	return f8.radius;
}
















