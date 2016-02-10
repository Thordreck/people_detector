#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>

#include "CorrectData.h"

using namespace ros;


int main(int argc, char** argv)
{
	init(argc, argv, "correct_data");

	CorrectData cd;
	spin();
	
	return 0;
}
