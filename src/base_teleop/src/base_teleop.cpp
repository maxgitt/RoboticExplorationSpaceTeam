#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
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
  ros::Subscriber joy_sub;

};


TeleopREST::TeleopREST():
  right(1),
  left(4)
{

  nh_.param("axis_right", right, right);
  nh_.param("axis_left", left, left);
  nh_.param("scale_angular", a_scale, a_scale);
  nh_.param("scale_linear", l_scale, l_scale);

  vel_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  joy_sub = nh_.subscribe<sensor_msgs::Joy>("joy", 10,&TeleopREST::joyCallback, this);

}

void TeleopREST::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
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


}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_REST");
  TeleopREST teleop_rest;

  ros::spin();
}
