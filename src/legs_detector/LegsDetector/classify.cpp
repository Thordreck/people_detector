

#include <LegsDetector.h>
#include <iostream>

using namespace std;


FILE *f_training=NULL, *f_classification=NULL, *f_hypotheses=NULL;
int num_hypotheses;

LegsDetector pd;

//-----------------------------------------------------
// 
//-----------------------------------------------------
void readSegments() 
{
	int scan_index, type, n_beams;
	int ret;
	Beam *b;
	float angle, range;
	int c=0;
	int new_type;
	double bin_class;
	int false_positives=0, false_negatives=0;
	int true_positives=0, true_negatives=0; 
	int total_positives=0, total_negatives=0;
	int nf;
	float *fv;

	while ( !feof(f_training) ) {
		//start reading segments
		ret = fscanf(f_training, "%d %d %d", &scan_index, &type, &n_beams);
		if ( (ret == 3) ) {
	
			c++;
			cerr << "Reading Segment: " <<  c << " n_beams: " << n_beams << endl; 

			if  ( type == 1 ) {
				total_positives ++;
			}
			else {
				total_negatives ++;
			}

			Segment *s = new Segment;
			s->scan_id = scan_index;
			s->type = type;
			
			for (int i=0; i<n_beams; i++) {			
				// read the first point
				b = new Beam;
				fscanf(f_training, " %f %f", &angle, &range );

				b->range = range;
				b->angle = angle;
				s->add(b);				
			}
			
			// read features,
			fscanf(f_training, " %d", &nf);
			fv = new float[nf];
			for (int i=0; i<nf; i++) {			
				fscanf(f_training, " %f", &fv[i] );
			}

			cerr << "Classifying segment " << c << "...";

			bin_class = pd.classify(s);

			cerr << "DONE" << endl;
			
			if ( bin_class > 0  ) {
				new_type = 1;  // person 
			}
			else {
				new_type = 0; // no person
			}

			// to check
			if ( type==1 && new_type==1) {
				true_positives ++;
			}
			else
			if ( type==1 && new_type==0) {
				false_negatives ++;
			}
			else
			if ( type==0 && new_type==0) {
				true_negatives ++;
			}
			else
			if ( type==0 && new_type==1) {
				false_positives ++;
			}

			// save data
			fprintf(f_classification, "%d %d %d %d", scan_index, type, new_type, n_beams);
			for (int i=0; i<n_beams; i++) {			
				// read the first point
				b = s->getElement(i);
				fprintf(f_classification, " %.8f %.8f", b->angle, b->range );
			}
			
			fprintf(f_classification, " %d", nf);
			for (int i=0; i<nf; i++) {			
				// read the first point
				fprintf(f_classification, " %.8f", fv[i] );
			}
			
			fprintf(f_classification, "\n");

			delete fv;

		} // end if ( ret == 3 ) {			
	} //end while ( !feof(f_training) )

	//check
	if ( total_positives !=  (true_positives + false_negatives) ) {
		cerr << "ERROR checking: total_positives !=  (true_positives + false_negatives): " << total_positives << true_positives << "+" << false_negatives;
		throw -1; 
	}	
	if ( total_negatives !=  (true_negatives + false_positives) ) {
		cerr << "ERROR checking: total_negatives !=  (true_negatives + false_positives): " << total_negatives << true_negatives << "+" << false_positives;
		throw -1; 
	}	

	
	// confussion matrix
	printf("Confussion matrix:\n");
	printf("            Positives    Negatives    Total\n");
	printf("Positives   %d           %d           %d\n", true_positives, false_negatives, total_positives);
	printf("Negatives   %d           %d           %d\n", false_positives, true_negatives, total_negatives); 

}
