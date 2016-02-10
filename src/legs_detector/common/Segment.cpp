
#include<Segment.h>
#include<ostream>
#include<iostream>

using namespace std;

using namespace std;

//-----------------------------------------------------
//
//-----------------------------------------------------
Segment::Segment() 
{
	beams = new dyntab_beams(700);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
Segment::~Segment() 
{
	beams->setAutoDelete(true);
	delete beams;
}


//-----------------------------------------------------
//
//-----------------------------------------------------
int Segment::num(void) 
{
	return beams->num();
}


//-----------------------------------------------------
//
//-----------------------------------------------------
void Segment::add(Beam *b) 
{
	return beams->add(b);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
Beam* Segment::getElement(int i) 
{
	return (Beam *)(beams->getElement(i));
}


//-----------------------------------------------------
// print information 
//-----------------------------------------------------
ostream& operator<<(ostream& out, Segment& s)
{
	for(int i=0; i<s.num();i++) {
		Beam *b = s.getElement(i);
		out << " (" << b->angle << "," << b->range << ")"; 
	}
	out << endl;

	return out;
}
























