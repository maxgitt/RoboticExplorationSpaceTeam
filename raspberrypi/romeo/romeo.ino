
//Standard PWM DC control
int E1 = 5;     //M1 Speed Control
int E2 = 6;     //M2 Speed Control
int M1 = 4;    //M1 Direction Control
int M2 = 7;    //M1 Direction Control

void stop(void)                    //Stop
{
  digitalWrite(E1,LOW);   
  digitalWrite(E2,LOW);      
}   

void set_direction(char m1D, char m2D)          //Move forward
{
  if(m1D == 'f') {
    digitalWrite(M1,HIGH);
  }
  else {
    digitalWrite(M1,LOW);
  }
  if(m2D == 'f'){
    digitalWrite(M2,HIGH);
  }
  else {
    digitalWrite(M2,LOW);   
  }
}  

void set_speed(int m1S,int m2S)
{
  analogWrite(E1,m1S);
  analogWrite(E2,m2S);    
} 

void setup(void) 
{ 
  int i;
  for(i=4;i<=7;i++)
    pinMode(i, OUTPUT);  
  Serial.begin(9600);      //Set Baud Rate
  Serial.println("Run keyboard control");
} 

void loop(void) 
{
  if(Serial.available()){
    char m1D = Serial.read();
    Serial.read();
    int m1S = Serial.readStringUntil(',').toInt();
    char m2D = Serial.read();
    Serial.read();
    int m2S = Serial.readStringUntil('\n').toInt();
    if(  m1S != -1 && m2S != -1)
    {
      set_speed(m1S, m2S);
      set_direction(m1D, m2D);
    }
    else {
      stop();  
    }
  }
}


