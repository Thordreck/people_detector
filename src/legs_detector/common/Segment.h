#ifndef __SEGMENT__
#define __SEGMENT__

#include<dynamictable.h>
#include<ostream>


using namespace std;

class Beam {
	public:
		double angle;
		double range;

		// total number of beams in the scan
		int num_readings;

		// position of this beam inside the scan
		int position;

		int segment_id; // segment it belongs to, in the scan
};

typedef DynamicTable<Beam> dyntab_beams;


//-----------------------------------------------------
// MAIN
//-----------------------------------------------------
class Segment {
	public:
		Segment();
		~Segment();
		int num(void);
		void add(Beam *b);
		Beam* getElement(int i);

		friend ostream& operator<<(ostream& out, Segment& s);
	
	public:
		dyntab_beams *beams;	

		int scan_id;
		int laser_id;
		int type;
};


#endif

