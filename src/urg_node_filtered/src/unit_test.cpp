#include <cassert>
#include <iostream>
#include <vector>
#include <urg_node/urg_c_wrapper.h>

#define NUM_STEPS 1080

/*
catkin_make
source devel/setup.bash
rosrun urg_node_filtered unit_test
*/

/* Flat Intensities, Small Window, Small Threshold */
void test_no_edges_1() {

	vector<float> intensities(NUM_STEPS, 600.0);
	int window_size = 1;
	int threshold_delta = 1;

	vector<int> edges = determine_intensity_edges(intensities, NUM_STEPS, window_size, threshold_delta);
	assert(edges.size() == 0);
}

int main() {
	test_no_edges_1();

	cout << "Tests All Passed\n";

	return 0;
}