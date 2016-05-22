#include "PeopleDetector.h"

using namespace ros;

int main(int argc, char **argv)
{

	init(argc,argv, "people_detector");
	PeopleDetector detector;

	spin();

	return 0;	
}
