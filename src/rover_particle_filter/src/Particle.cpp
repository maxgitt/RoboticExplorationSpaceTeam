//
//  particle.cpp
//  ParticleFilter
//
//  Created by Kishore B. Rao on 3/25/17.
//  Copyright © 2017 Kishore B. Rao. All rights reserved.
//

#include <stdio.h>
#include "particle.h"


Particle::Particle(double xPosIn, double yPosIn, double thetaIn) : xPos(xPosIn), yPos(yPosIn), theta(thetaIn), weight(0){}

double Particle::getX() {
    return xPos;
}

double Particle::getY() {
    return yPos;
}

double Particle::getTh() {
    return theta;
}

double Particle::getWeight() {
    return weight;
}

void Particle::setPos(double xPosIn, double yPosIn) {
    xPos = xPosIn;
    yPos = yPosIn;
}

void Particle::setAngle(double thetaIn) {
    theta = thetaIn;
}

void Particle::setWeight(double weightIn) {
    weight = weightIn;
}