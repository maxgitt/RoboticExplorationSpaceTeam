#include "rover_particle_filter/Filter.h"

#include <stdio.h>
#include <random>
#include <iostream>
#include <ctime>
#include <math.h>

using namespace std;
const pair<double, double> Filter::dimensions(3.88, 7.38);


Filter::Filter(int numParticles) {
    pose_array = new PoseArray("/filter_pose", "map");

    std::default_random_engine generator(time(0));
    std::uniform_real_distribution<double> xDistribution(-1*dimensions.first/2, dimensions.first/2);
    std::uniform_real_distribution<double> yDistribution(0, dimensions.second);
    std::uniform_real_distribution<double> thDistribution(0, 2 * M_PI);
    for (int i = 0; i < numParticles; ++i) {
        double xPos = xDistribution(generator);
        double yPos = yDistribution(generator);
        double theta = thDistribution(generator);
        points.push_back(Particle(xPos, yPos, theta));
    }
}

void Filter::resample() {
    cout << "resampling" << endl;
}

void Filter::predict() {
    cout << "predicting" << endl;
}


void Filter::update() {
    cout << "updating" << endl;
}


void Filter::determinePose() {
    cout << "Determining the new pose" << endl;
}


void Filter::process() {
    resample(); //if our particle population has depleted, resample
    predict(); //move particels based on odometry + introduce noise
    update(); //use pose info from sesnsors to weight each particle and then normalize
    determinePose(); //analyze points
    publishPoseArray();
}

void Filter::publishPoseArray() {
    pose_array->publish(points);
}