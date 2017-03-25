//
//  filter.cpp
//  ParticleFilter
//
//  Created by Kishore B. Rao on 3/25/17.
//  Copyright © 2017 Kishore B. Rao. All rights reserved.
//

#include <stdio.h>
#include "filter.h"
#include <random>
#include <iostream>
#include <math.h>

using namespace std;

const pair<double, double> MCFilter::dimensions(3.88,7.38);



MCFilter::MCFilter(int numParticles) {
    std::default_random_engine generator;
    std::uniform_real_distribution<double> xDistribution(0, dimensions.first);
    std::uniform_real_distribution<double> yDistribution(0, dimensions.second);
    std::uniform_real_distribution<double> thDistribution(0, 2 * M_PI);
    for (int i = 0; i < numParticles; ++i) {
        double xPos = xDistribution(generator);
        double yPos = yDistribution(generator);
        double theta = thDistribution(generator);
        points.push_back(Particle(xPos, yPos, theta));
    }
}


void MCFilter::resample() {
    cout << "resampling" << endl;
}

void MCFilter::predict() {
    cout << "predicting" << endl;
}


void MCFilter::update() {
    cout << "updating" << endl;
}


void MCFilter::determinePose() {
    cout << "Determining the new pose" << endl;
}


void MCFilter::process() {
    resample(); //if our particle population has depleted, resample
    predict(); //move particels based on odometry + introduce noise
    update(); //use pose info from sesnsors to weight each particle and then normalize
    determinePose(); //analyze points

}
