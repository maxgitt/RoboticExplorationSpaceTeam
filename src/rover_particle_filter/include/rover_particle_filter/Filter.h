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
    
    MCFilter(int numParticles);
    void resample();
    void predict();
    void update();
    void determinePose();
    void process();
    
    
    
private:
    static const std::pair<double, double> dimensions;
    std::vector<Particle> points;
    
    
    
};



#endif /* filter_h */
