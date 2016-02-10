//-----------------------------------------
// base class for weak hypothesis
//
// oscar martinez
//-----------------------------------------

#ifndef __WEAK_HYPOTHESIS__
#define __WEAK_HYPOTHESIS__


#include <Feature.h>
#include <stdio.h>

//-----------------------------------------
// class Hypothesis
//-----------------------------------------
class WeakHypothesis {

    // functions
    public:
        WeakHypothesis();
		~WeakHypothesis();

		// load the parameters from a file
		void load(FILE *f);

		// initializa values of the hypothesis
		// throws -1 if error	
		void init(int feature_index, double threshold, int direction, double alpha);

		//--------------------------------------
		// returns
		// classification: sign(double), > 0 positive, < 0 negative 
		//-------------------------------------- 
		double classify(Segment *s);


	public:
		Feature *feature;

		int direction;   // 1: consider positive the elements to the right of pos
						 // 2: consider positive the elements to the left of pos
		double threshold;
		double alpha;

};


#endif // __WEAK_HYPOTHESIS__

