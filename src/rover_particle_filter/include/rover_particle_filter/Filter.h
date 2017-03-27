//
//  filter.h
//  ParticleFilter
//
//  Created by Kishore B. Rao on 3/25/17.
//  Copyright © 2017 Kishore B. Rao. All rights reserved.
//

#ifndef filter_h
#define filter_h

#include <utility>
#include "particle.h"
#include <vector>

class MCFilter {
    
public:
    
    enum SelectionAlgorithm_t
    {
        WEIGHTED_MEAN,
        ROBUST_MEAN,
        BEST_PARTICLE
    };
    
    struct modelParam { //characterizes the error of our odometry sources
        //parameters that characterize our angle odometry information
        double meanRot = 0;
        double stdDevRot = 0;
        double numTransSteps = 0; //the higher this parameter is the more computation time required
        //parameters that characterize our translation odometry information in terms of translation AND drift
        double meanTrans = 0;
        double stdDevTrans = 0;
        double meanDrift = 0;
        double stdDevDrift = 0;
        //parametrs that charaterize our pose estimation (LIDAR + Beacons)
        double stdDevDist = 0;
        double stdDevAngle = 0;
        //if robust mean is chosen, user will have to choose how close to the max particle weight will be included
        double weightThreshold = 0; //optional
        //at what point do we resample
        double resamplingPercent = 0;
    };

    MCFilter(int numParticlesIn, modelParam modelIn, SelectionAlgorithm_t algoIn);
    void resample();
    void predict();
    void update();
    void determinePose();
    void process();
    
    
    
    
    
private:
    static const std::pair<double, double> dimensions;
    std::vector <double> pose; //intialized to 3 in constructor
    std::vector<Particle> points;
    void generateOutputArray(std::vector<double>& indices);
    double calcESS();
    void renormalize();
    double calcCoeffVar();
    void modelOrientation(); //models the change in the robots angle
    void modelTranslation(); //models the translation of the robot towards the destination
    int numParticles;
    modelParam model;
    double getTheta(); //obtains orientation information from the ROS network
    double getTransData(); //obtains translation information from the ROS network
    void getPoseEstimate(std::vector<double>& vin);
    void updateParticleWeight(Particle& point, std::vector<double>& pose);
    double calcWeight(const double& particleVal, const double& sensorVal, const double& stdDev);
    SelectionAlgorithm_t particleAlgo;
    void bestParticle();
    void weightedMean();
    void robustMean();
    void publishPose();
    
    
    
};



#endif /* filter_h */
