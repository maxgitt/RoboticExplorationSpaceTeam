#ifndef DIG_CONTROL_H
#define DIG_CONTROL_H
#include <iostream>
#include <vector>
#include <time.h>
#include <ros/ros.h>
#include "rover_excavation/finish_dig.h"
#include "excavationSerial.h"

using namespace std;

class dig_control
{
public:
    dig_control();
    ~dig_control();
    float scale(float num);
    void send(float num);
    float calcP(float Kp, int i);
    float calcI(float Ki, int i);
    float calcD(time_t time1, float Kd, int i);
    float calcPID(float P, float I, float D);
    float getWeight();
    void waitForServiceRequest();
    bool digSubRoutine();
    void sendEndOfService();
	bool dumpSubRoutine();

private:
    ros::NodeHandle nh;
    ros::Publisher dig_pub;

	SerialComm linearActuator; 

	int timeout = 40; //TODO set timout seconds
	float target_weight = 10; //set through testing TODO

    vector <float> e;         //error vector
    int i;                  //index

    float tolerance;

    float maxWeight;       //Set max weight
    float weight;            //initialize current weight

    float P;                 //initialze proportional part of control
    float Kp;                //initialize constant of proportional control

    float I;                 //initialize integral part of control
    float Ki;                //initialize integral constant of control

    float D;                 //initialize derivative part of control
    float Kd;                //initialize derivative contstant of control

    float T;                 //Time constant initialization

    int motorControllerValue; //Initialize output value
    int sum;
    time_t time1;

};

#endif // DIG_CONTROL_H
