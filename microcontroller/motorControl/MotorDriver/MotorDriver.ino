#include <Servo.h>
Servo motorL, motorR, motorLF, motorRF;
unsigned int speedL = 90;
unsigned int speedR = 90;
unsigned int speedLF = 90;
unsigned int speedRF = 90;

void setup() {
  // Left motor ouput goes through pin 9 and right through 10
  motorL.attach(5);
  motorR.attach(10);
  motorLF.attach(6);
  motorRF.attach(11);
  Serial.begin(9600);
}

void loop() 
{
  //if there is something to read in, update motors
  if(Serial.available() > 0)
  {
    update(speedL, speedR, speedLF, speedRF);
    motorL.write(speedL);
    motorR.write(speedR);
    motorLF.write(speedLF);
    motorRF.write(speedRF);
  }
}

void update(unsigned int &L, unsigned int &R, unsigned int &LF, unsigned int &RF)
{
  //motors are opposite of each other so multiply one side by negative one
  L = 180 - Serial.readStringUntil(',').toInt();
  R = Serial.readStringUntil(',').toInt();
  LF = 180 - Serial.readStringUntil(',').toInt();
  RF = Serial.readStringUntil('\n').toInt();
}
