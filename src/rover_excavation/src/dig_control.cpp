#include <ros/ros.h>
#include "dig_control.h"
#include "rover_excavation/dig_progress.h"

using namespace ros;

dig_control::dig_control()
{
    dig_pub = nh.advertise<rover_excavation::dig_progress>("progress", 10);
    e.push_back(maxWeight-weight);
    e.push_back(maxWeight-weight);
    i = 0;                  //index

    tolerance = .025;

    maxWeight = 100;       //Set max weight
    weight = 0;            //initialize current weight

    P = 0;                 //initialze proportional part of control
    Kp = 45;                //initialize constant of proportional control

    I = 0;                 //initialize integral part of control
    Ki = 0;                //initialize integral constant of control

    D = 0;                 //initialize derivative part of control
    Kd = 0;                //initialize derivative contstant of control

    T = 0;                 //Time constant initialization

    motorControllerValue = 0; //Initialize output value
    sum = 0;
}

dig_control::~dig_control()
{

} 

    
bool dig_control::dumpSubRoutine()
{
	linearActuator.commandLinearActuator("extend");
	double secs = ros::Time::now().toSec();
	while(ros::Time::now().toSec() - secs < 5) //sleep for 5 seconds
	{
	}
	linearActuator.commandLinearActuator("retract";
	return true;
}
bool dig_control::digSubRoutine()
{
	rover_excavation::dig_progress prog_msg;
	double secs = ros::Time::now().toSec();
	double runtime = ros::Time::now().toSec();
    while(e.at(i) >= 0+tolerance) //Check value of i?
    {
        time(&time1);

        e.push_back(maxWeight-weight); //add the error to the error to the error vector

        i++;

        P = calcP(Kp, i);           //Calculate proportional part of control
        //cout <<"P = " << P << endl;

        I = calcI(Ki, i);           //Calculate Integral part of control
        //cout <<"I = "<< I << endl;

        D = calcD(time1, Kd, i);    //Calculate Derivative part of control
       // cout <<"D = " << D << endl;

        motorControllerValue = calcPID(P, I, D);      //Calculate number to send to microcontroller

        //Scaling function 
        //motorControllerValue = scale(motorControllerValue);

        weight = getWeight();

        cout << "Speed = " << motorControllerValue << endl;
        cout << "Error = " << e.at(i) << endl;
		if(ros::Time::now().toSec() - secs > .8)
		{
			prog_msg.percentFull = weight/target_weight;
			dig_pub.publish(prog_msg);
			secs = ros::Time::now().toSec();
		}
		if(ros::Time::now().toSec() - runtime > timeout)
		{
			return weight >= (double)target_weight / 2.; //return false if load isn't halfway full
		}
		

        //Send motor value over serial
        //send(motorControllerValue);
    }
	return true;
}

float dig_control::scale(float num)
{
    //Implement me!
    return 0;

}

float dig_control::getWeight()
{
    //Code to get weight measurement from Microcontroller or Master-node

    return weight + motorControllerValue*.00001;
}

void dig_control::send(float num)
{
   // CSerial serial;
   //serial.Open(2, 9600);
}

float dig_control::calcP(float Kp, int i)
{
    float P = Kp * e.at(i);                   //Calculate Proportional part of control
    return P;
}

float dig_control::calcI(float Ki, int i)
{
   int sum = 0;                            //clear sum
    for(int cycler = 0; cycler < e.size(); cycler++)
    {
        sum += e.at(cycler);            //Sum up all of error vector
    }

  float I = Ki * sum;                       //Calculate Integral part of control

   return I;
}

float dig_control::calcD(time_t time1, float Kd, int i)
{
    time_t time2;
    time(&time2);

   //float T = difftime(time1,time2);
    float T = .01;

  float  D = Kd * ((e.at(i) - e.at(i-1)) / T);    //Calculate Derivative part of control

    return D;

}

float dig_control::calcPID(float P, float I, float D)
{
    float PID = P + I + D;
    return PID;

}