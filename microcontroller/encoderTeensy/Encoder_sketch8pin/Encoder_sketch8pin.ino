#include <Encoder.h>

#include <Encoder.h>

//ROUGH DRAFT OF TEENSY ENCODER READING FOR FOUR MOTORS!!!

#define SEND_VEL 500
long oldPosition1  = -999; //set oldPosition to something that wil make sure
long oldPosition2  = -999; //new position reads the first loop
long oldPosition3  = -999;
long oldPosition4  = -999;

long oldVal1 = 0;
long newVal1 = 0;
long oldVal2 = 0;
long newVal2 = 0;
long oldVal3 = 0;
long newVal3 = 0;
long oldVal4 = 0;
long newVal4 = 0;
long lastCheck = 0;
Encoder myEnc1(5, 6); //set up Encoder from given header file
Encoder myEnc2(7, 8);
Encoder myEnc3(9, 10);
Encoder myEnc4(11, 12);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Encoder Test:"); 
}

void loop() {
  // put your main code here, to run repeatedly:
  long newPosition1 = myEnc1.read();
  long newPosition2 = myEnc2.read();
  long newPosition3 = myEnc3.read();
  long newPosition4 = myEnc4.read();
  Serial.println(myEnc1.read());
  if (newPosition1 != oldPosition1) 
  {
    oldPosition1 = newPosition1;
  }
  if (newPosition2 != oldPosition2) 
  {
    oldPosition2 = newPosition2;
  }
  if (newPosition3 != oldPosition3) 
  {
    oldPosition3 = newPosition3;
  }
  if (newPosition4 != oldPosition4) 
  {
    oldPosition4 = newPosition4;
  }
  if((millis() - lastCheck) >= SEND_VEL)
  {
    lastCheck = millis();
    newVal1 = newPosition1;
    newVal2 = newPosition2;
    newVal3 = newPosition3;
    newVal4 = newPosition4;

    long speed1 = (newVal1 - oldVal1)/SEND_VEL;
    long speed2 = (newVal2 - oldVal2)/SEND_VEL;
    long speed3 = (newVal3 - oldVal3)/SEND_VEL;
    long speed4 = (newVal4 - oldVal4)/SEND_VEL;
    
    
    
    Serial.print(speed4);
    Serial.print(",");
    Serial.print(speed2);
    Serial.print(",");
    Serial.print(speed3);
    Serial.print(",");
    Serial.print(speed1);
    Serial.print("\n");
    // 0.73 cm/s for 1 count
    // for max count (72) = 52.5 cm/s = 1.89 km/h
    oldVal1 = newVal1;
    oldVal2 = newVal2;
    oldVal3 = newVal3;
    oldVal4 = newVal4;
  }
}
