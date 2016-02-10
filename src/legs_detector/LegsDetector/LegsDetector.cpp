
#include <LegsDetector.h>
#include <ostream>
#include <iostream>
#include <stdio.h>
#include <math.h>

using namespace std;

//-----------------------------------------------------
//
//-----------------------------------------------------
LegsDetector::LegsDetector() 
{
	weak_classifiers = NULL;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
LegsDetector::~LegsDetector() 
{
	if ( weak_classifiers != NULL ) {	
		weak_classifiers->setAutoDelete(true);
		delete weak_classifiers;
		weak_classifiers = NULL;
	}
}


//-----------------------------------------------------
// Load Hypotheses from a file
//-----------------------------------------------------
void LegsDetector::load(FILE *f, int num) 
{
	WeakHypothesis *w;

	weak_classifiers = new dyntab_hypotheses(num);

	for(int i=0; i<num; i++) {
		w = new WeakHypothesis;	 
		w->load(f);
		weak_classifiers->add(w);
	}
}


//-----------------------------------------------------
//
// ----------------------------------------------------
int LegsDetector::classify(Segment *s) 
{
	WeakHypothesis *w;
	double sum = 0.0;
	int type;
	double w_type;

	int n = weak_classifiers->num();

	for(int i=0; i<n; i++) {
		w = weak_classifiers->getElement(i);
		w_type = w->classify(s);
		sum = sum + w->alpha * w_type;
	}	
	
	if ( sum < 0.0 ) {
		type = -1;
	}
	else {
		type = 1;
	}

	s->type = type;

	return type;	
}


//-----------------------------------------------------
// segment a scan
// ----------------------------------------------------
void LegsDetector::segmentScan(double threshold, int n_readings, double *angles, double *ranges, dyntab_segments *segments)
{

	//cerr << "LegsDetector::segmentScan" << endl; 

	Segment *s;
	Beam *b=NULL, *b_prev=NULL;	

	s = new Segment;
	//start with a segment
	for (int i=0; i<n_readings; i++) {			
		// read the first point
		b = new Beam;
		b->range = ranges[i];
		b->angle = angles[i];
		b->num_readings = n_readings;
		b->position = i;					
		if ( s->num() > 0 ) {
			if ( fabs(b_prev->range - b->range) < threshold ) {
				// same segment
				b_prev = b;
				b->segment_id = segments->num(); 
				s->add(b);					
			}
			else {
				segments->add(s);
				s = new Segment;
				b->segment_id = segments->num(); 
				s->add(b);
				b_prev = b;
			} 
		}	
		else {
			b_prev = b;
			s->add(b);	
		}						
	}
	segments->add(s);
} 
























