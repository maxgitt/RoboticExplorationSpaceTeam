#include "ros/ros.h"
#include <tutorial/Test.h>
#include <iostream>
using std::cout; using std::endl;
// class Tutorial {
// public: 
// 	Tutorial();
// private:
// 	int test = 0;
// 	ros::NodeHandle nh;
// 	ros::Publisher pub;
// 	ros::Subscriber sub;
// };
void testCallback(const tutorial::Test::ConstPtr & msg){
	cout << "in callback" << endl;

}


int main(int argc, char** argv){
	ros::init(argc, argv, "teleop_REST");
	ros::NodeHandle nh;
	ros::Publisher pub;
	ros::Subscriber sub;
	pub = nh.advertise<tutorial::Test>("test", 1);
	sub = nh.subscribe<tutorial::Test>("test", 10, &testCallback);
	tutorial::Test msg;
	msg.test_integer = 10;
	ros::spin();
}

