#include "PeopleDetector.h"

using namespace ros;

int main(int argc, char **argv)
{

	init(argc,argv, "people_detector");
	PeopleDetector detector(argv[1],argv[2],argv[3]);

	spin();

	return 0;	
}
