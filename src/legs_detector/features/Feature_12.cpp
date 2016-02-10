

#include <math.h>
#include <Feature_6.h>
#include <Feature_12.h>
#include <iostream>
#include <Matrix.hh>

using namespace std;

//-------------------------------------------------------------
// Mean curvature
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
double Feature_12::calculateFeature(Segment *s)
{
	
	int n = s->num();
	Beam *ba, *bb, *bc;
	double da, db, dc;
	double xa, xb, xc, ya, yb, yc;
	double area;
	double curv=0.0;

	//check the lenght
	Feature_6 f6;
	
	double l = f6.calculateFeature(s);	
	if ( l==0.0 ) {
#ifdef DEBUG
	cerr << "Feature_12::calculateFeature:  length==0, res=0.0" << endl;
#endif 
		return 0.0;	
	}

	for(int i=1; i<n-1; i++) {

		ba = s->getElement(i-1);
		xa = ba->range * cos(ba->angle);
		ya = ba->range * sin(ba->angle);

		bb = s->getElement(i);
		xb = bb->range * cos(bb->angle);
		yb = bb->range * sin(bb->angle);

		bc = s->getElement(i+1);
		xc = bc->range * cos(bc->angle);
		yc = bc->range * sin(bc->angle);

		da = sqrt( (xa-xb)*(xa-xb) + (ya-yb)*(ya-yb) );
		db = sqrt( (xb-xc)*(xb-xc) + (yb-yc)*(yb-yc) );
		dc = sqrt( (xc-xa)*(xc-xa) + (yc-ya)*(yc-ya) );
	
		area = fabs((xb*ya - xa*yb) + (xc*yb - xb*yc) + (xa*yc - xc*ya)) / 2.0;

		curv += 4 *area / ( da*db*dc );
	}



#ifdef DEBUG
	cerr << "Feature_12::calculateFeature:  curv: " << curv << endl;
#endif 

	return curv;
}
















