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
  double get_desired_velocity_left();
  double get_desired_velocity_right();
  double get_current_velocity_left();
  double get_current_velocity_right();

  double set_current_velocity_right(double v);
  double set_current_velocity_left(double v);

  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void rampControl(double t1, double t2, double c1, double c2);

  
  
  ros::NodeHandle nh_;

  double desired_velocity_left;
  double desired_velocity_right;

  double current_velocity_left;
  double current_velocity_right;

  int right_;
  int left_;
  double l_scale_, a_scale_;
  
  
};


TeleopREST::TeleopREST():
	right_(1),
	  left_(4)
{
  nh_.param("axis_right", right_, right_);
  nh_.param("axis_left", left_, left_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  	desired_velocity_left = 90;
	desired_velocity_right = 90;
  
	current_velocity_left = 90;
	current_velocity_right = 90;


  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10,&TeleopREST::joyCallback, this);
}

double TeleopREST::get_desired_velocity_left()
{
	return desired_velocity_left;
}

double TeleopREST::get_desired_velocity_right()
{
	return desired_velocity_right;
}

double TeleopREST::get_current_velocity_left()
{
	return current_velocity_left;
}

double TeleopREST::get_current_velocity_right()
{
	return current_velocity_right;
}

double TeleopREST::set_current_velocity_left(double v)
{
	current_velocity_left = v;
}

double TeleopREST::set_current_velocity_right(double v)
{
	current_velocity_right = v;
}

void TeleopREST::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{  
  desired_velocity_left = joy->axes[left_];
  desired_velocity_right = joy->axes[right_];

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_REST");
  TeleopREST teleop_rest;

  //c1 and c2 must come from encoders, c1 is the left motor's current speed, c2 is the right motor's current speed
    teleop_rest.set_current_velocity_left(90);  //From encoders
    teleop_rest.set_current_velocity_right(90);  //From encoders
    
    double error1 = teleop_rest.get_desired_velocity_left() - teleop_rest.get_current_velocity_left();
    double error2 = teleop_rest.get_desired_velocity_right() - teleop_rest.get_current_velocity_right();
    double gain = .1;		//Must be experimentally tuned
    double tolerance = .001;	//Accuracy tolerance

    geometry_msgs::Twist vel;

	//must add stuff before the loop too
	while(ros::ok())
	{
	    	teleop_rest.set_current_velocity_left(teleop_rest.get_current_velocity_left() + (gain * error1));
		teleop_rest.set_current_velocity_right(teleop_rest.get_current_velocity_right() + (gain * error2));
		error1 = teleop_rest.get_desired_velocity_left() - teleop_rest.get_current_velocity_left();
		error2 = teleop_rest.get_desired_velocity_right() - teleop_rest.get_current_velocity_right();

		if( teleop_rest.get_desired_velocity_left() == teleop_rest.get_desired_velocity_right()){
		//forward/backward
		vel.linear.x = teleop_rest.get_desired_velocity_left(); // or vr
		vel.angular.z = 0;
		}
		else if( (-1 * teleop_rest.get_desired_velocity_left()) == teleop_rest.get_desired_velocity_right()){
			//turning
			vel.angular.z = 2*teleop_rest.get_desired_velocity_right()/ROBOT_WIDTH;
			vel.linear.x = 0;
		}
		else{
			//moving doing arcs
			vel.angular.z = (teleop_rest.get_desired_velocity_right() - teleop_rest.get_desired_velocity_left())/ROBOT_WIDTH;
			vel.linear.x = teleop_rest.get_desired_velocity_right() - vel.angular.z * ROBOT_WIDTH / 2.0;
		}	
		teleop_rest.vel_pub_.publish(vel);
		ros::spin();
	}
} 
