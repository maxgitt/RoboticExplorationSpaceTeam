#include <Servo.h>

#define DRIVETRAIN 0
#define EXCAVATOR 1

Servo motorL, motorR, motorLF, motorRF, motorExcv;
unsigned int speedL = 90;
unsigned int speedR = 90;
unsigned int speedLF = 90;
unsigned int speedRF = 90;

int actL_w1 = 0;
int actL_w2 = 0;
int actR_w1 = 0;
int actR_w2 = 0;
int speedExcv = 90;

void setup() {
  // Left motor ouput goes through pin 9 and right through 10
  motorL.attach(5);
  motorR.attach(10);
  motorLF.attach(6);
  motorRF.attach(11);

  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
  motorExcv.attach(3);

  Serial.begin(9600);
}

void loop() 
{
  //if there is something to read in, update motors
  if(Serial.available() > 0)
  {
    // drivetrain
    update_wheel_speeds(speedL, speedR, speedLF, speedRF);
    motorL.write(speedL);
    motorR.write(speedR);actL_w1
    motorLF.write(speedLF);
    motorRF.write(speedRF);
    
    //excavator
    digitalwrite(8, actL_w1);
    digitalwrite(9, actL_w2);
    digitalwrite(12, actR_w1);
    digitalwrite(13, actR_w2);
    motorExcv.write(speedExcv);
  }
}

void update_wheel_speeds(unsigned int &L, unsigned int &R, 
			unsigned int &LF, unsigned int &RF,)
{
	int sys = Serial.readStringUntil(',').toInt();
	if(sys == DRIVETRAIN){
  		//motors are opposite of each other so multiply one side by negative one
  		L = 180 - Serial.readStringUntil(',').toInt();
  		R = Serial.readStringUntil(',').toInt();
  		LF = 180 - Serial.readStringUntil(',').toInt();
  		RF = Serial.readStringUntil('\n').toInt();
	}
	else(sys == EXCAVATOR){
		int left_act = Serial.readStringUntil(',').toInt();
		int right_act = Serial.readStringUntil(',').toInt();
		speedExcv = Serial.readStringUntil(',').toInt();
		if( left_act == 0 ){
			actL_w1 = 0;
			actL_w2 = 0;
		} else if ( left_act == 1 ) {
			actL_w1 = 0;
			actL_w2 = 1;
		} else if ( left_act == -1 ) {
			actL_w1 = 1;
			actL_w2 = 0;
		} 

		if( right_act == 0 ){
			actR_w1 = 0;
			actR_w2 = 0;
		} else if ( right_act == 1 ) {
			actR_w1 = 0;
			actR_w2 = 1;
		} else if ( right_act == -1 ) {
			actR_w1 = 1;
			actR_w2 = 0;
		} 		
	}
}


