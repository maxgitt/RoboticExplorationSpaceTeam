#include <Servo.h>

#define DRIVETRAIN 0
#define EXCAVATOR_MAN 1
#define EXCAVATOR_AUTO 2
#define READ_LIGHTSENSOR 3
#define RESET_STRINGPOT_MINS 4
#define RESET_STRINGPOT_MAXS 5
#define READ_STRINGPOTS 6

#define LEFT_MOTOR_PIN 9
#define RIGHT_MOTOR_PIN 10

#define BUCKET_MOTOR_PIN 11

#define LEFT_ACTUATOR_PIN 6
#define RIGHT_ACTUATOR_PIN 5

#define LEFT_STRINGPOT_PIN 0  // analog pin
#define RIGHT_STRINGPOT_PIN 1 // analog pin

#define LIGHTSENSOR_PIN 3 // analog pin

int LEFT_STRINGPOT_MAX = 644;
int LEFT_STRINGPOT_MIN = 351; 

int RIGHT_STRINGPOT_MAX = 620;
int RIGHT_STRINGPOT_MIN = 320;

Servo motor_left, motor_right, motor_excavator, actuator_left, actuator_right;
int speed_left = 90;
int speed_right = 90;
int speed_excavator = 90;

int actuator_left_direction  = 90;
int actuator_right_direction = 90;
double actuator_left_goal = -1;
double actuator_right_goal = -1;

void run_command(int);
void update_actuator_right_direction();
void update_actuator_left_direction();

void
setup() {
	motor_left.attach(LEFT_MOTOR_PIN);
	motor_right.attach(RIGHT_MOTOR_PIN);
	motor_excavator.attach(BUCKET_MOTOR_PIN);
	actuator_left.attach(LEFT_ACTUATOR_PIN);
	actuator_right.attach(RIGHT_ACTUATOR_PIN);
	Serial.begin(9600);
}

void 
loop() {
	//if there is something to read in, update motors
	if(Serial.available() > 0) {
		int sys = Serial.readStringUntil(',').toInt();
		run_command(sys);
	}
  update_actuator_right_direction();
  update_actuator_left_direction();
  //Serial.println("asdf");

  //Serial.println(analogRead(LEFT_STRINGPOT_PIN));
  //Serial.println(analogRead(RIGHT_STRINGPOT_PIN));

}

void
run_command(int sys) {
		if(sys == DRIVETRAIN){
  			//motors are opposite of each other so multiply one side by negative one
  			speed_left = 180 - Serial.readStringUntil(',').toInt();
  			speed_right = Serial.readStringUntil('\n').toInt();
    		motor_left.write(speed_left);
    		motor_right.write(speed_right);
		}
		else if (sys == EXCAVATOR_MAN){
      // Manual Controller Indicated
      actuator_left_goal = -1;
      actuator_right_goal = -1;
			int left_act = Serial.readStringUntil(',').toInt();
			int right_act = Serial.readStringUntil(',').toInt();
			speed_excavator = Serial.readStringUntil('\n').toInt();
			if( left_act == 0 ){
				actuator_left_direction = 90;
			} else if ( left_act == 1 ) {
				actuator_left_direction = 180;
			} else if ( left_act == -1 ) {
				actuator_left_direction = 0;
			} 
			if( right_act == 0 ){
				actuator_right_direction = 90;
			} else if ( right_act == 1 ) {
				actuator_right_direction = 0;
			} else if ( right_act == -1 ) {
				actuator_right_direction = 180;
			}
			motor_excavator.write(speed_excavator);
		}
		else if (sys == EXCAVATOR_AUTO) {
      actuator_left_goal  = Serial.readStringUntil(',').toFloat();
      actuator_right_goal = Serial.readStringUntil(',').toFloat();
      speed_excavator = Serial.readStringUntil('\n').toInt();
      motor_excavator.write(speed_excavator);
		}
    else if (sys == READ_LIGHTSENSOR) {
      Serial.println(analogRead(LIGHTSENSOR_PIN));
    }
    else if (sys == RESET_STRINGPOT_MINS) {
        RIGHT_STRINGPOT_MIN = analogRead(RIGHT_STRINGPOT_PIN);
        LEFT_STRINGPOT_MIN  = analogRead(LEFT_STRINGPOT_PIN);
    }
    else if (sys == RESET_STRINGPOT_MAXS) {
        RIGHT_STRINGPOT_MAX = analogRead(RIGHT_STRINGPOT_PIN);
        LEFT_STRINGPOT_MAX  = analogRead(LEFT_STRINGPOT_PIN);    
    }
    else if (sys == READ_STRINGPOTS) {
        Serial.println(analogRead(RIGHT_STRINGPOT_PIN) + "," + analogRead(LEFT_STRINGPOT_PIN) );
    }    
}

void
update_actuator_right_direction() {
  if( actuator_right_goal != -1) {  
    int actuator_right_reading = analogRead(RIGHT_STRINGPOT_PIN);
    int goal_in_ticks = int(RIGHT_STRINGPOT_MIN + actuator_right_goal * (RIGHT_STRINGPOT_MAX - RIGHT_STRINGPOT_MIN));
    if( actuator_right_reading > goal_in_ticks){
      actuator_right_direction = 180;
      actuator_right.write(actuator_right_direction);     
    }
    else if( actuator_right_reading <  goal_in_ticks){
      actuator_right_direction = 0;
      actuator_right.write(actuator_right_direction);     
    }      
    else {
      actuator_right_direction = 90;
      actuator_right_goal  = -1;
      actuator_right.write(actuator_right_direction);
    }
  }
  else {
    actuator_right.write(actuator_right_direction);
  }
}

void
update_actuator_left_direction() {
  if( actuator_left_goal != -1){
    int actuator_left_reading = analogRead(LEFT_STRINGPOT_PIN);
    int goal_in_ticks = int(LEFT_STRINGPOT_MIN + actuator_left_goal * (LEFT_STRINGPOT_MAX - LEFT_STRINGPOT_MIN));
    if( actuator_left_reading > goal_in_ticks){
      actuator_left_direction = 5;
      actuator_left.write(actuator_left_direction);     
    }
    else if( actuator_left_reading < goal_in_ticks){
      actuator_left_direction = 175;
      actuator_left.write(actuator_left_direction);     
    }      
    else {
      actuator_left_direction = 90;
      actuator_left_goal  = -1;
      actuator_left.write(actuator_left_direction); 
    }
  }
  else {
    actuator_left.write(actuator_left_direction); 
  }
}

