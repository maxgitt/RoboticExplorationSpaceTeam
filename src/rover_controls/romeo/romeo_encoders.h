#include <Arduino.h>
#include <CommunicationUtils.h>

#define digitalPinToInterrupt(p)  ((p) == 2 ? 0 : ((p) == 3 ? 1 : 0))

#define LEFT 0
#define RIGHT 1

// Encoders
static unsigned long timer = 0;
volatile float coder[2] = {0,0};
float lastSpeed[2] = {0,0};

void
LwheelSpeed() {
  coder[LEFT] ++;  //count the left wheel encoder interrupts
}


void
RwheelSpeed() {
  coder[RIGHT] ++; //count the right wheel encoder interrupts
}

void
encoders_setup() {
	attachInterrupt(2, LwheelSpeed, CHANGE);    //init the interrupt mode for the digital pin 2
	attachInterrupt(4, RwheelSpeed, CHANGE);   //init the interrupt mode for the digital pin 3
}

void
encoders_process() {
  if(millis() - timer > 100){                   
    lastSpeed[LEFT] = ((coder[LEFT]/20) * .235614) ;   //record the latest speed value
    lastSpeed[RIGHT] = ((coder[RIGHT]/20) * .235614) ;
    serialPrintFloatArr(lastSpeed, 2);
    Serial.println();
    coder[LEFT] = 0;                 //clear the data buffer
    coder[RIGHT] = 0;
    timer = millis();
  }
}