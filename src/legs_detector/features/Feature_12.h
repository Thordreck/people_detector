#ifndef __FEATURE_12_
#define __FEATURE_12_

#include <Feature.h>

//---------------------------------
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
//---------------------------------
class Feature_12: public Feature {

	//methods 
	public:
		double calculateFeature(Segment *s);

};

#endif
