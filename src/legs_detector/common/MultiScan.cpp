
#include<MultiScan.h>
#include<ostream>
#include<iostream>

using namespace std;


//-----------------------------------------------------
//
//-----------------------------------------------------
MultiScan::MultiScan() 
{
	n_scans = 0;
	scans = NULL;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
MultiScan::MultiScan(int n) 
{
	n_scans = n;
	scans = new dyntab_segments*[n_scans];
	for(int i=0; i<n_scans; i++) { 	
		scans[i]=new dyntab_segments(700);
	}
	
}

//-----------------------------------------------------
//
//-----------------------------------------------------
MultiScan::~MultiScan() 
{
	for(int i=0; i<n_scans; i++) { 	
		scans[i]->setAutoDelete(true);
		delete scans[i];
		scans[i] = NULL;
	}
	delete [] scans;
	scans=NULL;
}


























