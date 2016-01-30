#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

//rover physical properties
#define ROBOT_WIDTH 1 //meters
#define WHEEL_DIAMETER .5 //meters

class TeleopREST
{
public:
  TeleopREST();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  
  ros::NodeHandle nh_;

  int right_;
  int left_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
  
};


TeleopREST::TeleopREST():
  right_(1),
  left_(4)
{

  nh_.param("axis_right", right_, right_);
  nh_.param("axis_left", left_, left_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);


  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10,&TeleopREST::joyCallback, this);
}

void TeleopREST::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist vel;  
  double vl = joy->axes[left_];
  double vr = joy->axes[right_];
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
  vel_pub_.publish(vel);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_REST");
  TeleopREST teleop_rest;

  ros::spin();
}
