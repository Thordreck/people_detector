//-----------------------------------------
// base class for weak hypothesis
//
// oscar martinez
//-----------------------------------------

#include <WeakHypothesis.h>
#include <AllFeatures.h>
#include <iostream>

using namespace std;

//-----------------------------------------
// constructor
//-----------------------------------------
WeakHypothesis::WeakHypothesis() {
	feature = NULL;
	direction = -1;
}

//-----------------------------------------
// destructor
//-----------------------------------------
WeakHypothesis::~WeakHypothesis() {
	if ( feature != NULL ) {
		delete feature;
		feature = NULL;
	}
}


//-----------------------------------------
// 
//-----------------------------------------
void WeakHypothesis::init(int feature_index, double threshold, int direction, double alpha ) 
{
	
	this->threshold = threshold;
	this->direction = direction;
	this->alpha = alpha;

	if ( feature_index==1 ) {
		feature = new Feature_1;
	}
	else
	if ( feature_index==2 ) {
		feature = new Feature_2;
	}
	else
	if ( feature_index==3 ) {
		feature = new Feature_3;
	}
	else
	// change index as we don't implemented feature 4 and 5 (see paper)
	if ( feature_index==4 ) {
		feature = new Feature_6;
	}
	else
	if ( feature_index==5 ) {
		feature = new Feature_7;
	}
	else
	if ( feature_index==6 ) {
		feature = new Feature_8;
	}
	else
	if ( feature_index==7 ) {
		feature = new Feature_9;
	}
	else
	if ( feature_index==8 ) {
		feature = new Feature_10;
	}
	else
	if ( feature_index==9 ) {
		feature = new Feature_11;
	}
	else
	if ( feature_index==10 ) {
		feature = new Feature_12;
	}
	else
	if ( feature_index==11 ) {
		feature = new Feature_13;
	}	
	else {
		cerr << "ERROR: WeakHypothesis::init() setting feature: " << feature_index << endl;
		throw -1;
	}

}


//-----------------------------------------
// classify a segment
//-----------------------------------------
double WeakHypothesis::classify(Segment *s) 
{
	double value = feature->calculateFeature(s);
	double type=0.0;	

	if ( direction==1 ) {
		// positives are >= than threshold
		if ( value >= threshold ) {
			type = 1.0;
		}
		else {
			type = -1.0;
		}

	}
	else
	if ( direction==2 ) {
		// positives are < than threshold
		if ( value < threshold ) {
			type = 1.0; // positive
		}
		else {
			type = -1.0; // negative
		}
	}	
	else {
		cerr << "ERROR: WeakHypothesis::classify() bad direction: " << direction<< endl;
		throw -1;	
	}

	return type; 
}


//-----------------------------------------
// 
//-----------------------------------------
void WeakHypothesis::load(FILE *f) 
{
	int f_index;
	int ret;
	float threshold;
	int direction;
	float alpha;
	float missclassified;
	float sum_w;

	ret=fscanf(f, "%d %f %f %f %d %f\n", &f_index, &alpha, &threshold, &missclassified, &direction, &sum_w);
	if (ret != 6) {	
		cerr << "ERROR WeakHypothesis::load(FILE *f) " << endl;
		throw -1;
	} 
	init(f_index, threshold, direction, alpha);
}











