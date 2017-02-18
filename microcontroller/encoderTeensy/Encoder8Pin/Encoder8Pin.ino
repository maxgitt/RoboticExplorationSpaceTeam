//#define ENCODER_USE_INTERRUPTS

#include <Encoder.h>

//ROUGH DRAFT OF TEENSY ENCODER READING FOR FOUR MOTORS!!!

#define ENCODER_PIN1 48
#define ENCODER_PIN2 50
#define RATE 100
#define Pi 3.14159265358979323846264338327950288419716939937510582097494459230781640628620899862803482534211706
#define RADIANS_PER_INTERRUPT (2*Pi)/(94)


Encoder encoder(ENCODER_PIN1, ENCODER_PIN2); //set up Encoder from given header file

// Positive is clockwise, negative is counter clockwise
double pos = 0;
double old_pos = 0;
double old_time = 0;

void setup() {
  // put your setup code here, to run once:
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN1), updateOnPin1, CHANGE);
  Serial.begin(9600);
  Serial.println("Encoder Test:");
}

void loop() {
  // put your main code here, to run repeatedly:
  bool pin1state = digitalRead( ENCODER_PIN1 );
  double new_time = millis();
  double time_diff = new_time - old_time;
  if(pos == old_pos){
    return;
  }
  if (time_diff > RATE){
    double speed = (pos - old_pos)*1000 *RADIANS_PER_INTERRUPT  / (time_diff);
    Serial.print("Speed: ");
    Serial.print(speed);
    Serial.print("\n"); 
//    Serial.print("Speed: ");
  //  Serial.print(speed);
    //Serial.print(" digits\n");    
    old_pos = pos;
    old_time = new_time;   
  }
  
}

void updateOnPin1(){
  bool pin1state = digitalRead( ENCODER_PIN1 ) == HIGH ? 1 : 0;
  bool pin2state = digitalRead( ENCODER_PIN2 ) == HIGH ? 1 : 0;
  if(pin1state){
     pin2state ? pos-- : pos++;
  }else{
     pin2state ? pos++ : pos--;
  }

}

