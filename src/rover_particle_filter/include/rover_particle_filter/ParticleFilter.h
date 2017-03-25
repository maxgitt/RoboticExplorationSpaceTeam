#ifndef PARTICLEFILTER_H
#define PARTICLEFILTER_H

#include "ros/ros.h"
// #include "rover_particle_filter/OdometrySource.h"

class ParticleFilter {
public:
	ParticleFilter();
	~ParticleFilter();

	void update();
private:


};

#endif