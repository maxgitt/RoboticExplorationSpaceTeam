#include <cassert>
#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <urg_node/urg_c_wrapper.h>
#include <ros/package.h>

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
	int threshold_delta = 100;

	vector<int> edges = determine_intensity_edges(intensities, NUM_STEPS, window_size, threshold_delta);
	assert(edges.size() == 0);
}

/* 10 Stepping Rising Intensities, Small Window, Avg Threshold */
void test_ten_stepping_rising_edges_1() {

	vector<float> intensities(NUM_STEPS, 600.0);
	int window_size = 5;
	int threshold_delta = 400;
	int plateau = 600;

	for (int i = 0; i < 1080; ++i) {
		if (i > 0 && i % 108 == 0) {
			intensities[i] = intensities[i-1] + 500;
			plateau = intensities[i];
			//cout << "New plateau: " << plateau << endl;
		}
		else {
			intensities[i] = plateau;
		}
		//cout << "Intensity at step " << i << ": " << intensities[i] << endl;
		
	}

	vector<int> edges = determine_intensity_edges(intensities, NUM_STEPS, window_size, threshold_delta);

	// for (int j = 0; j < edges.size(); ++j) {
	// 	cout << edges[j] << " ";
	// }
	// cout << endl;
	// cout << "Edges Detected: " << edges.size()  << "\n";
	assert(edges.size() == 9);
}

/* Uses real-test data from checkerboard, 10 edges, 2 nonreal */
void test_eight_edges_1() {

	int window_size = 1;
	int threshold_delta = 500;

	// Create path to test data
	string path = ros::package::getPath("urg_node_filtered");
	path += "/src/test_data.txt";

	ifstream infile(path.c_str());
	int intensity_val, idx = 0;
	vector<float> intensities(NUM_STEPS);

	while (infile >> intensity_val) {
		intensities[idx++] = intensity_val;
		//cout << "Intensity at idx " << idx - 1 << ": " << intensity_val << "\n";
	}

	vector<int> edges = determine_intensity_edges(intensities, NUM_STEPS, window_size, threshold_delta);

	// for (int j = 0; j < edges.size(); ++j) {
	// 	cout << edges[j] << " ";
	// }
	// cout << endl;
	// cout << "Edges Detected: " << edges.size()  << "\n";
	assert(edges.size() == 10);

}

int main() {
	test_no_edges_1();
	test_ten_stepping_rising_edges_1();
	test_eight_edges_1();

	cout << "Tests All Passed\n";

	return 0;
}