#include "rover_particle_filter/Particle.h"

#include <stdio.h>


Particle::Particle(double xPosIn, double yPosIn, double thetaIn, double weightIn) : xPos(xPosIn), yPos(yPosIn), theta(thetaIn), weight(weightIn){}

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
