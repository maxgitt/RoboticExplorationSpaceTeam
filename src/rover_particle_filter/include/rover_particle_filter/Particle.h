#ifndef PARTICLE_H
#define PARTICLE_H

class Particle {
public:
    Particle(double xPosIn, double yPosIn, double thetaIn);
    
    //Setter functions
    void setPos(double xPosIn, double yPosIn);
    void setAngle(double thetaIn);
    void setWeight(double weightIn);
    
    //Getter functions
    double getX();
    double getY();
    double getTh();
    double getWeight();
    
private:
    double xPos;
    double yPos;
    double theta;
    double weight;
       
};

#endif