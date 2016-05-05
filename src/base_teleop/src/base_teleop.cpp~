#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include "rover_controls/Excavation.h"
#include <iostream>
#include <cmath>

using std::cout; using std::endl;
//rover physical properties
#define ROBOT_WIDTH .75 //meters
#define WHEEL_DIAMETER .5 //meters

class TeleopREST
{
public:
  TeleopREST();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  bool is_disabled();
  ros::NodeHandle nh_;

  int right;
  int left;
  double l_scale, a_scale;
  ros::Publisher vel_pub;
  ros::Publisher excv_pub;
  ros::Subscriber joy_sub;
  
  // 0 - disable
  // 1 - 
  bool disabled = true;

  int excv_tog = 0;
  int act_tog = 0;
  int actL   = 0;
  int actR   = 0;
};


TeleopREST::TeleopREST():
  right(1),
  left(4)
{

  nh_.param("axis_right", right, right);
  nh_.param("axis_left", left, left);
  nh_.param("scale_angular", a_scale, a_scale);
  nh_.param("scale_linear", l_scale, l_scale);

  excv_pub = nh_.advertise<rover_controls::Excavation>("cmd_excv", 1);
  vel_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  joy_sub = nh_.subscribe<sensor_msgs::Joy>("joy", 10,&TeleopREST::joyCallback, this);

}

bool TeleopREST::is_disabled(){
	if(disabled == true) {
  		geometry_msgs::Twist vel;
		vel.linear.x = 0;
		vel.angular.z = 0;
		vel_pub.publish(vel);
		//add disabling code for excavation  
		act_tog = 0;
  		rover_controls::Excavation excv;
		excv.left_actuator  = 0;
		excv.right_actuator = 0;
		excv.excavator_speed = 0;
		return true;
	}
	else return false;
}

void TeleopREST::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
 cout << "disabled" << disabled << endl;
  if(joy->buttons[7] == 1){
	if(disabled) disabled = false;
	else disabled = true;
  }
  if(is_disabled()) return;

  geometry_msgs::Twist vel;  
  double vl = joy->axes[left];
  double vr = joy->axes[right];
  if(vl < 0)
  {
    vl = -1 * sqrt((vl* -1));
  }
  else
  {
    vl = sqrt(vl);
  }
  if(vr < 0)
  {
    vr = -1 * sqrt((vr* -1));
  }
  else
  {
    vr = sqrt(vr);
  }
	
	if( vl == vr){
		//forward/backward
		vel.linear.x = vl; // or vr
		vel.angular.z = 0;
	}
	else if( (-1 * vl) == vr){
		//turning
		vel.angular.z = 2*vr/ROBOT_WIDTH;
		vel.linear.x = 0;
	}
	else{
		//moving doing arcs
		vel.angular.z = (vr - vl)/ROBOT_WIDTH;
		vel.linear.x = vr - vel.angular.z * ROBOT_WIDTH / 2.0;
	}
  vel_pub.publish(vel);

  if(joy->buttons[0] == 1 && joy->buttons[3] == 1 && !disabled){
	cout << "Actuators Disabled: Only toggle one button" << endl;
	act_tog = 0;	
  }
  else if(joy->buttons[3] == 1 && !disabled){
	cout << "Actuators set to EXTEND" << endl;	
	act_tog = 1;	
  }
  else if(joy->buttons[0] == 1 && !disabled){
	cout << "Actuators set to RETRACT" << endl;	
	act_tog = -1;		
  }
  else{
	cout << "Actuators set to " << act_tog << endl;	
  }

  if(joy->buttons[1] == 1 && joy->buttons[2] == 1 && !disabled){
	cout << "Excavator Disabled: Only toggle one button" << endl;
	excv_tog = 0;	
  }
  else if(joy->buttons[1] == 1 && !disabled){
	cout << "Excavator set to FORWARD" << endl;	
	excv_tog = 1;	
  }
  else if(joy->buttons[2] == 1 && !disabled){
	cout << "Excavator set to REVERSE" << endl;	
	excv_tog = -1;		
  }
  else{
	cout << "Excavator set to " << excv_tog << endl;	
  }


  rover_controls::Excavation excv;
  excv.manual 		= true;
  excv.left_actuator    = act_tog * abs(joy->buttons[4]);
  excv.right_actuator   = act_tog * abs(joy->buttons[5]);
  excv.excavator_speed  = excv_tog * (joy->axes[5] - 1) / 2;
  excv_pub.publish(excv);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_REST");
  TeleopREST teleop_rest;

  ros::spin();
}
