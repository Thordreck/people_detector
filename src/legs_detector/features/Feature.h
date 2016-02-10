
#ifndef __FEATURE__
#define __FEATURE__

#include <Segment.h>


//---------------------------------------------
// base class
//---------------------------------------------
class Feature {

	// functions
	public:
		Feature();
		virtual ~Feature();

		virtual double calculateFeature(Segment *S)=0;
	// data
	public:
};	

#endif

