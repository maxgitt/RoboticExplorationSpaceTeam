#include "rover_particle_filter/Filter.h"

#include <stdio.h>
#include <random>
#include <iostream>
#include <ctime>
#include <math.h>
#include <algorithm>

using namespace std;
const pair<double, double> Filter::dimensions(3.88, 7.38);



MCFilter::MCFilter(int numParticlesIn, modelParam modelIn, SelectionAlgorithm_t algoIn) : numParticles(numParticlesIn), model(modelIn), particleAlgo(algoIn), pose(3){
     pose_array = new PoseArray("/filter_pose", "map");
    double weight = 1/numParticles;
    std::default_random_engine generator(time(0));
    std::uniform_real_distribution<double> xDistribution(-1*dimensions.first/2, dimensions.first/2);
    std::uniform_real_distribution<double> yDistribution(0, dimensions.second);
    std::uniform_real_distribution<double> thDistribution(0, 2 * M_PI);
    for (int i = 0; i < numParticles; ++i) {
        double xPos = xDistribution(generator);
        double yPos = yDistribution(generator);
        double theta = thDistribution(generator);
        points.push_back(Particle(xPos, yPos, theta, weight));
    }
}


void MCFilter::generateOutputArray(vector<double>& indices) {
    vector<double> cumSum(numParticles, 0); //cumulative sum
    double currSum = 0;
    for(int i = 0; i < numParticles; ++i) {
        currSum += points[i].getWeight();
        cumSum[i] = currSum;
    }
    vector<double> randNum(numParticles + 1, 0); //random numbers
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(0, 1);
    for (int i = 0; i < randNum.size(); ++i) {
        randNum[i] = distribution(generator);
    }
    sort(randNum.begin(), randNum.end());
    randNum.back() = 1; //set last element to be one so that you can't get past it
    
    int i = 0;
    int j = 0;
    while(i < numParticles) { //construct output array
        if (randNum[i] < cumSum[j]) { //add to output array
            indices[i] = j;
            ++i;
        }
        else { //point is removed
            ++j;
        }
    }

}

double MCFilter::calcCoeffVar() {
    double runningSum;
    for (int i = 0; i < numParticles; ++i) {
        runningSum += pow ((numParticles * points[i].getWeight() - 1) , 2);
    }
    runningSum /= numParticles;
    return runningSum;
    
}

double MCFilter::calcESS() {
    return numParticles/(1 + calcCoeffVar());
}


void MCFilter::renormalize() {
    double totalWeight = 0;
    for (int i = 0; i < numParticles; ++i) {
        totalWeight += points[i].getWeight();
    }
    for (int i = 0; i < numParticles; ++i) {
        points[i].setWeight(points[i].getWeight()/totalWeight);
    }
    
}


void MCFilter::resample() {
    double ess = calcESS();
    if (ess < numParticles * model.resamplingPercent) {
        vector<double> indices(numParticles, 0);
        generateOutputArray(indices);
        vector<Particle> lastPoints = points;
        for(int i = 0; i < indices.size(); ++i) {
            points[i] = lastPoints[indices[i]];
        }
        renormalize();
    }
}

double MCFilter::getTheta() { //Returns the change in theta from last time
    return 0.5;
}

double MCFilter::getTransData() { //Returns the distance traveled
    return 0.5;
}

void MCFilter::getPoseEstimate(vector<double>& vin) { //Returns beacon/laser data
    vin.push_back(0.5);
    vin.push_back(0.5);
    vin.push_back(0.5);
}

void MCFilter::publishPose() { //Publishes the new pose
    cout << "I am putting on ROS network!" << endl;
}

void MCFilter::modelOrientation() {
    std::default_random_engine generator;
    std::normal_distribution<double> distribution(model.meanRot, model.stdDevRot);
    for (int i = 0; i < numParticles; ++i) {
        double currTheta = points[i].getTh();
        double dTheta = getTheta();
        double randomError = distribution(generator);
        points[i].setAngle(currTheta + dTheta + randomError);
    }
}


void MCFilter::modelTranslation() {
    std::default_random_engine generator;
    double distanceTraveled = getTransData();
    double deltaRho = distanceTraveled/model.numTransSteps; //model translation as several descrete steps with error in each
    std::normal_distribution<double> driftError(model.meanDrift * deltaRho, model.stdDevDrift * deltaRho);
    std::normal_distribution<double> transError(model.meanTrans * deltaRho, model.stdDevTrans * deltaRho);
       for (int i = 0; i < numParticles; ++i) {
        for (int j = 0; j < model.numTransSteps; ++j) {
            double errorTrans = transError(generator);
            double errorDrift = driftError(generator);
            double newTh = points[i].getTh() + errorDrift;
            points[i].setAngle(newTh);
            double newXPos = points[i].getX() + ((deltaRho + errorTrans) * cos(newTh));
            double newYPos = points[i].getY() + ((deltaRho + errorTrans) * sin(newTh));
            points[i].setPos(newXPos, newYPos);
            errorDrift = driftError(generator);
            newTh = points[i].getTh() + errorDrift;
            points[i].setAngle(newTh);
            
        }
    }
}


double MCFilter::calcWeight(const double& particleVal, const double& sensorVal, const double& stdDev) {
    double coeff = 1 / (sqrt(2 * M_PI) * stdDev);
    double exponenet = exp( (-pow((sensorVal - particleVal), 2))/(2 * pow(stdDev,2)));
    return coeff * exponenet;
}



void MCFilter::predict() {
    modelOrientation(); //the robot first rotates to the desired position
    modelTranslation(); //the robot then translates
}

void MCFilter::updateParticleWeight(Particle &point, std::vector<double> &pose) {
    
    double xWeight = calcWeight(point.getX(), pose[0], model.stdDevDist);
    double yWeight = calcWeight(point.getY(), pose[1], model.stdDevDist);
    double thetaWeight = calcWeight(point.getTh(), pose[2], model.stdDevAngle);
    point.setWeight(xWeight * yWeight * thetaWeight);
    
}

void MCFilter::update() { //update the weights of each of the particles
    //weight each particle based on how far it is from our sensing information
    vector<double> poseEstimate;
    getPoseEstimate(poseEstimate);
    for (int i = 0; i < numParticles; ++i) {
        updateParticleWeight(points[i], poseEstimate);
    }
    renormalize(); //renormalize the weights
    
}


void MCFilter::bestParticle() {
    Particle bestPart = points[0];
    double bestWeight = points[0].getWeight();
    for (int i = 1; i < numParticles; ++i) {
        if (points[i].getWeight() > bestWeight) {
            bestWeight = points[i].getWeight();
            bestPart = points[i];
        }
    }
    pose[0] = bestPart.getX();
    pose[1] = bestPart.getY();
    pose[2] = bestPart.getTh();
}


void MCFilter::robustMean() {
    //first find the best weight
    double bestWeight = points[0].getWeight();
    for (int i = 1; i < numParticles; ++i) {
        if (points[i].getWeight() > bestWeight) {
            bestWeight = points[i].getWeight();
        }
    }
    //calculate weighted average while throwing out the values that are too far from the best
    double totalX = 0;
    double totalY = 0;
    double totalTh = 0;
    int numPointsUsed = 0;
    for (int i = 0; i < numParticles; ++i) {
        double currWeight = points[i].getWeight();
        if (abs(currWeight - bestWeight) < model.weightThreshold) {
            totalX += points[i].getX() * currWeight;
            totalY += points[i].getY() * currWeight;
            totalTh += points[i].getTh() * currWeight;
            ++numPointsUsed;
        }
    }
    pose[0] = totalX / numPointsUsed;
    pose[1] = totalY / numPointsUsed;
    pose[2] = totalTh / numPointsUsed;
    
}

void MCFilter::weightedMean() {
    //calculate weighted average while throwing out the values that are too far from the best
    double totalX = 0;
    double totalY = 0;
    double totalTh = 0;
    for (int i = 0; i < numParticles; ++i) {
        double currWeight = points[i].getWeight();
        totalX += points[i].getX() * currWeight;
        totalY += points[i].getY() * currWeight;
        totalTh += points[i].getTh() * currWeight;
    }
    
    pose[0] = totalX / numParticles;
    pose[1] = totalY / numParticles;
    pose[2] = totalTh / numParticles;
}



//Three different methods to determine the pose: We are planning on
void MCFilter::determinePose() {
    switch (particleAlgo) { //based on which algorithm the user has chosen
        case WEIGHTED_MEAN:
            weightedMean();
        case ROBUST_MEAN:
            robustMean();
        case BEST_PARTICLE:
            bestParticle();
    }
}



void MCFilter::process() {
    resample(); //if our particle population has depleted, resample
    predict(); //move particels based on odometry + introduce noise
    update(); //use pose info from sesnsors to weight each particle and then normalize
    determinePose(); //analyze points
    publishPose();
}

void MCFilter::publishPoseArray() {
    pose_array->publish(points);
}

