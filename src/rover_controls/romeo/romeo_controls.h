#include <Arduino.h>

#define ROBOT_WIDTH .1
#define MAX_SPEED 150

// global for romeo controls
int vr = 0;
int vl = 0;  


//Standard PWM DC control
int E1 = 5;     //M1 Speed Control
int E2 = 6;     //M2 Speed Control
int M1 = 4;    //M1 Direction Control
int M2 = 8;    //M12 Direction Control default 7

void set_speed(int m1S,int m2S)
{
  analogWrite(E1,m1S);
  analogWrite(E2,m2S);    
} 
void controls_process() 
{
  int m1S = Serial.parseInt();
  int m2S = Serial.parseInt();

  if(m1S < 0) {
     digitalWrite(M1,LOW);
  }
  else {
     digitalWrite(M1,HIGH);
  }

  if(m2S < 0) {
     digitalWrite(M2,LOW);
  }
  else {
     digitalWrite(M2,HIGH);
  }
  m1S =  abs(m1S);
  m2S =  abs(m2S);  
  set_speed(m1S, m2S);

}
