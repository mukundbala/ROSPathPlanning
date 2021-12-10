#include <PosCorrect/PosCorrect.hpp>
#include <iostream>
#include <ros/ros.h>

using namespace std;
using namespace poscorrect;

int main(int argc, char** argv){

	ros::init(argc, argv, "pos_correct_node");
	ros::NodeHandle NodeHandle("~");

	PosCorrect PC(NodeHandle);

	PC.spin();
	ros::Duration(5).sleep();
	return 0;
}