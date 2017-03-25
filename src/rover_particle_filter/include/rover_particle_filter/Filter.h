#ifndef FILTER_H
#define FILTER_H

#include "rover_particle_filter/Particle.h"

#include "rover_particle_filter/PoseArray.h"

#include <utility>
#include <vector>

class Filter {
    
public:
    
    Filter(int numParticles);
    void resample();
    void predict();
    void update();
    void determinePose();
    void process();
    void publishPoseArray();
    
private:
    static const std::pair<double, double> dimensions;
    std::vector<Particle> points;
    
    PoseArray * pose_array; 
};



#endif /* filter_h */
