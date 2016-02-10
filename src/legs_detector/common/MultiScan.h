#ifndef  __MULTISCAN__ 
#define __MULTISCAN__

#include <dynamictable.h>
#include <ostream>
#include <Segment.h>


using namespace std;


typedef DynamicTable<Segment> dyntab_segments;


//-----------------------------------------------------
// MAIN
//-----------------------------------------------------
class MultiScan {
	public:
		MultiScan();
		MultiScan(int n_scans);
		~MultiScan();
	
	public:
		dyntab_segments **scans;	

		int n_scans;
		int scan_id;
		double timestamp;
		int type;
};


#endif
