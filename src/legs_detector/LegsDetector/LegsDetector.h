#ifndef __PEOPLEDETECTOR__
#define __PEOPLEDETECTOR__

#include <WeakHypothesis.h>
#include <dynamictable.h>
#include <Segment.h>
#include <ostream>

typedef DynamicTable<WeakHypothesis> dyntab_hypotheses;
typedef DynamicTable<Segment> dyntab_segments;


//-----------------------------------------------------
// MAIN
//-----------------------------------------------------
class LegsDetector {
	public:
		LegsDetector();
		~LegsDetector();
	
 		void load(FILE *f, int num);
	
		// +1, -1
		int classify(Segment *s);

		// Segment a scan
		void segmentScan(double threshold, int n_readings, double *angles, double *ranges, dyntab_segments *segments);

	public:

		dyntab_hypotheses *weak_classifiers;
};


#endif

