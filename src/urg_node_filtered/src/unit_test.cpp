#include <cassert>
#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <urg_node/urg_c_wrapper.h>
#include <ros/package.h>

#define NUM_STEPS 1080

/*
source devel/setup.bash
catkin_make
rosrun urg_node_filtered unit_test

gdb ./devel/lib/urg_node_filtered/unit_test
*/

/* Flat Intensities, Small Window, Small Threshold */
void test_no_edges() {

	vector<float> intensities(NUM_STEPS, 600.0);
	int window_size = 1;
	int threshold_delta = 100;

	vector<int> edges = determine_intensity_edges(intensities, NUM_STEPS, window_size, threshold_delta);
	assert(edges.size() == 0);
}

/* 10 Stepping Rising Intensities, Small Window, Avg Threshold */
void test_ten_stepping_rising_edges() {

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

/* Uses real-test data from checkerboard, 9 edges, 1 nonreal */
void test_determine_real_checkerboard() {

	int window_size = 3;
	int threshold_delta = 600;

	// Create path to test data
	string path = ros::package::getPath("urg_node_filtered");
	path += "/src/test_data.txt";

	ifstream infile(path.c_str());
	int intensity_val, idx = 0;
	vector<float> intensities(NUM_STEPS, 2500);

	while (infile >> intensity_val) {
		intensities[idx++] = intensity_val;
		//cout << "Intensity at idx " << idx - 1 << ": " << intensity_val << "\n";
	}

	vector<int> edges = determine_intensity_edges(intensities, NUM_STEPS, window_size, threshold_delta);

	vector<int> ends = find_flag_ends(edges, 15, 8);

	// for (int j = 0; j < edges.size(); ++j) {
	// 	cout << edges[j] << " ";
	// }
	// cout << endl;
	assert(edges.size() == 9);
	assert(ends[0] < 5 + 5 && ends[0] > 5 - 5);
	assert(ends[1] < 159 + 5 && ends[1] > 159 - 5);

}

/* Test smooth_intensities on glitched real data containing 8 flag edges
	and 1 high-intensity noise edge */
void test_smoothing_determine_real_checkerboard() {
	// Create path to test data
	string path = ros::package::getPath("urg_node_filtered");
	path += "/src/test_data.txt";

	ifstream infile(path.c_str());
	int intensity_val, idx = 0;
	vector<float> intensities(NUM_STEPS,2400);

	while (infile >> intensity_val) {
		intensities[idx++] = intensity_val;
	}

	int smooth_window_size = 5;
	int det_window_size = 5;
	int det_intensity_delta = 600;

	vector<float> smoothed_intensities = smooth_intensities(intensities, NUM_STEPS, smooth_window_size);

	vector<int> determined_edges = determine_intensity_edges(smoothed_intensities, NUM_STEPS, det_window_size, det_intensity_delta);

	// cout << "Start edges, size = " << determined_edges.size() << endl;
	// for (auto val: determined_edges) {
	// 	cout << val << endl;
	// }
	// cout << "End edges" << endl;

	assert(determined_edges.size() == 8);
}

/* No edges */
void test_gradual_slope() {
	int window_size = 1;
	int threshold_delta = 400;

	vector<float> intensities(NUM_STEPS, 1.0);

	for (int i=0; i < intensities.size(); ++i) {
		intensities[i] *= 10;
	}

	vector<int> edges = determine_intensity_edges(intensities, NUM_STEPS, window_size, threshold_delta);

	assert(edges.size() == 0);
}

/* Few edges, small window */
void test_steep_slope() {
	int window_size = 1;
	int threshold_delta = 400;
	int num_steps = 10;

	vector<float> intensities(num_steps, 1.0);

	for (int i=1; i < intensities.size(); ++i) {
		intensities[i] = intensities[i-1]*10;
	}

	vector<int> edges = determine_intensity_edges(intensities, num_steps, window_size, threshold_delta);
	assert(edges.size() == 7);
}

/* Few edges, wider window */
void test_steep_slope_wider() {
	int window_size = 3;
	int threshold_delta = 400;
	int num_steps = 10;

	vector<float> intensities(num_steps, 1.0);

	for (int i=1; i < intensities.size(); ++i) {
		intensities[i] = 10*intensities[i-1];
	}

	vector<int> edges = determine_intensity_edges(intensities, num_steps, window_size, threshold_delta);

	//cout << edges.size() << endl;
	assert(edges.size() == 2);
}

/* Test Flag Ends */
void test_normal_flag_ends() {

	vector<float> intensities(1080,1000);
	int margin_of_error = 5;
	int exp_edges = 4;

	for(int i=0; i < intensities.size(); ++i) {
		if ((216 <= i && i < 432) || (648 <= i && i < 864)) {
			intensities[i] = 1800;
		}
	}

	vector<int> edge_indices = determine_intensity_edges(intensities, NUM_STEPS, 1, 600);

	vector<int> ends = find_flag_ends(edge_indices, margin_of_error, exp_edges);

	assert(ends[0] == 216);
	assert(ends[1] == 864);
}

/* Test Real Flag with noise cutoff */
void test_real_flag_cutoff() {
	// Create path to test data
	string path = ros::package::getPath("urg_node_filtered");
	path += "/src/real_flag_cutoff_intens.txt";

	int num_steps = 261;

	ifstream infile(path.c_str());
	int intensity_val, idx = 0;
	vector<float> intensities(num_steps);

	while (infile >> intensity_val) {
		intensities[idx++] = intensity_val;
	}

	
	int smooth_window_size = 3;
	int det_window_size = 3;
	int det_intensity_delta = 700;
	int ends_gap_length_variability = 15;
	int exp_edges = 6;

	vector<float> smoothed_intensities = smooth_intensities(intensities, num_steps, smooth_window_size);

	vector<int> determined_edges = determine_intensity_edges(smoothed_intensities, num_steps, det_window_size, det_intensity_delta);

	// cout << "Start edges, size = " << determined_edges.size() << endl;
	// for (auto val: determined_edges) {
	// 	cout << val << endl;
	// }
	// cout << "End edges" << endl;

	vector<int> ends = find_flag_ends(determined_edges, ends_gap_length_variability, exp_edges);
	// cout << "Flag ends: \n";
	// for (auto idx: ends) {
	// 	cout << idx << endl;
	// }
	assert(ends[0] < 61 + 5 && ends[0] > 61 - 5);
	assert(ends[1] < 242 + 5 && ends[1] > 242 - 5);
}

/* Some noise, Real Flag */
void test_real_flag_low_noise() {
	// Create path to test data
	string path = ros::package::getPath("urg_node_filtered");
	path += "/src/real_flag_low_noise.txt";

	ifstream infile(path.c_str());
	int intensity_val, idx = 0;
	vector<float> intensities(NUM_STEPS);

	while (infile >> intensity_val) {
		intensities[idx++] = intensity_val;
	}

	
	int smooth_window_size = 1;
	int det_window_size = 1;
	int det_intensity_delta = 700;
	int ends_gap_length_variability = 15;
	int exp_edges = 6;

	vector<float> smoothed_intensities = smooth_intensities(intensities, NUM_STEPS, smooth_window_size);

	vector<int> determined_edges = determine_intensity_edges(smoothed_intensities, NUM_STEPS, det_window_size, det_intensity_delta);

	// cout << "Start edges, size = " << determined_edges.size() << endl;
	// for (auto val: determined_edges) {
	// 	cout << val << endl;
	// }
	// cout << "End edges" << endl;

	vector<int> ends = find_flag_ends(determined_edges, ends_gap_length_variability, exp_edges);
	// cout << "Flag ends: \n";
	// for (auto idx: ends) {
	// 	cout << idx << endl;
	// }
	assert(ends[0] < 465 + 5 && ends[0] > 465 - 5);
	assert(ends[1] < 645 + 5 && ends[1] > 645 - 5);
}

/* Get position, using real flag data */
void test_position_real_flag_cutoff() {
	// Create path to test data
	string path1 = ros::package::getPath("urg_node_filtered");
	string path2 = ros::package::getPath("urg_node_filtered");
	path1 += "/src/real_flag_cutoff_intens.txt";
	path2 += "/src/real_flag_cutoff_dists.txt";

	int num_steps = 261;

	ifstream intenfile(path1.c_str());
	ifstream distfile(path2.c_str());
	int intensity_val = 0;
	int idx = 0;
	int dist_val = 0;
	vector<float> intensities(num_steps);
	vector<float> distances(num_steps);

	while (intenfile >> intensity_val) {
		intensities[idx++] = intensity_val;
	}
	idx = 0;
	while (distfile >> dist_val) {
		distances[idx++] = dist_val;
	}
	
	int smooth_window_size = 3;
	int det_window_size = 3;
	int det_intensity_delta = 700;
	int ends_gap_variability = 15;
	int exp_edges = 6;

	vector<float> smoothed_intensities = smooth_intensities(intensities, num_steps, smooth_window_size);
	vector<int> determined_edges = determine_intensity_edges(smoothed_intensities, num_steps, det_window_size, det_intensity_delta);
	vector<int> ends = find_flag_ends(determined_edges, ends_gap_variability, exp_edges);

	assert(ends[0] < 61 + 5 && ends[0] > 61 - 5);
	assert(ends[1] < 242 + 5 && ends[1] > 242 - 5);

	vector<double> position;
	position = get_position(ends, distances);
	cout << "X: " << position[0] << " Y: " << position[1] << endl;
	double orientation = get_orientation(position, ends, distances);
	cout << "Theta: " << orientation << endl;
}


int main() {

	test_smoothing_determine_real_checkerboard();

	cout << "Smoothing Intensities Tests Passed\n";

	test_no_edges();
	test_ten_stepping_rising_edges();
	test_determine_real_checkerboard();
	test_gradual_slope();
	test_steep_slope();
	test_steep_slope_wider();

	cout << "Intensity Edges Tests Passed\n";


	test_normal_flag_ends();
	test_real_flag_cutoff();
	test_real_flag_low_noise();


	cout << "Flag Ends Tests Passed\n";

	test_position_real_flag_cutoff();

	cout << "Position Tests Passed\n";


	return 0;
}