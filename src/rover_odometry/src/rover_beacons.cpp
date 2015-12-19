#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <string>
#include <stdio.h>
#include <sstream>
#include <iostream>
#include <ctype.h>


#define ROVER_BEACON_FWD_DEFAULT 23
#define ROVER_BEACON_AFT_DEFAULT 23

#define SIEVE_BEACON_LEFT_DEFAULT 24
#define SIEVE_BEACON_RIGHT_DEFAULT 13

#define SIEVE_OFFSET_LEFT_DEFAULT -1
#define SIEVE_OFFSET_RIGHT_DEFAULT 1

#define SAMPLE_SIZE 10

class RoverOdometry
{
public:
    RoverOdometry();
    void calculateOffset();
    void computeTransform();
    void publishOdometry();
    void updatePosePosition();
    void sampleDriver(char *);
    std::string drivercmd_ = "script -c \"sudo '/home/pascualy/catkin_ws/decawave_driver/dwm1000driver'";
    std::string drivercmd_postfix_ = "\" /dev/null"; 
    
    FILE* driverprogram_;
    ros::Time current_time_;
    ros::Time last_time_;
private:
    ros::NodeHandle nh_;
    
    //Beacon ids
    int rover_beacon_id_fwd_;
    int rover_beacon_id_aft_;
    int sieve_beacon_id_left_;
    int sieve_beacon_id_right_;
    
    int sieve_offset_left_;
    int sieve_offset_right_;
      
    //Beacon distance
    double rover_fwd_sieve_left_  = 0;
    double rover_fwd_sieve_right_ = 0;
    double rover_aft_sieve_left_  = 0;
    double rover_aft_sieve_right_ = 0;
    
    //Odometry variables
    double x_prev_ = 0;
    double y_prev_ = 0;
    double th_prev_ = 0;
    
    double x_ = 0;  //x position
    double y_ = 0;  //y position
    double th_ = 0; //angle of
    
    double vx_ = 0;
    double vy_ = 0;
    double vth_ = 0;
    
    ros::Publisher odom_pub_;
    tf::TransformBroadcaster odom_broadcaster_;
    geometry_msgs::Quaternion odom_quat_;
    geometry_msgs::TransformStamped odom_trans;

};

RoverOdometry::RoverOdometry():
rover_beacon_id_fwd_(ROVER_BEACON_FWD_DEFAULT),
rover_beacon_id_aft_(ROVER_BEACON_AFT_DEFAULT),
sieve_beacon_id_left_(SIEVE_BEACON_LEFT_DEFAULT),
sieve_beacon_id_right_(SIEVE_BEACON_RIGHT_DEFAULT),
sieve_offset_left_(SIEVE_OFFSET_LEFT_DEFAULT),
sieve_offset_right_(SIEVE_OFFSET_RIGHT_DEFAULT)
{
    
    nh_.param("rover_beacon_id_fwd", rover_beacon_id_fwd_, rover_beacon_id_fwd_);
    nh_.param("rover_beacon_aft", rover_beacon_id_aft_, rover_beacon_id_aft_);
    nh_.param("sieve_beacon_left", sieve_beacon_id_left_, sieve_beacon_id_left_);
    nh_.param("sieve_beacon_right", sieve_beacon_id_right_, sieve_beacon_id_right_);
    
    nh_.param("beacon_driver_cmd", drivercmd_, drivercmd_);

    std::stringstream ss(drivercmd_);

    drivercmd_ = drivercmd_ + " " + std::to_string(sieve_beacon_id_left_) + "," + std::to_string(sieve_beacon_id_right_) + drivercmd_postfix_;
    std::cout << drivercmd_ << std::endl;
    odom_pub_     = nh_.advertise<nav_msgs::Odometry>("odom", 50);
    current_time_ = ros::Time::now();
    last_time_    = ros::Time::now();
}

void RoverOdometry::calculateOffset(){
    
    
    
}

void RoverOdometry::updatePosePosition(){
    x_prev_  = x_;
    y_prev_  = y_;
    th_prev_ = th_;
    
    //distance between the sieve beacons
    double x_fwd = pow(sieve_offset_left_,2) - pow(sieve_offset_left_,2) + pow(rover_fwd_sieve_right_, 2) - pow(rover_fwd_sieve_left_, 2);
    x_fwd /= (-2*(-sieve_offset_left_ + sieve_offset_right_));
    double y_fwd = sqrt(abs(pow(rover_fwd_sieve_left_, 2) - pow(x_fwd - sieve_offset_left_, 2)));
    
    double x_aft = pow(sieve_offset_left_,2) - pow(sieve_offset_left_,2) + pow(rover_aft_sieve_right_, 2) - pow(rover_aft_sieve_left_, 2);
    x_aft /= (-2*(-sieve_offset_left_ + sieve_offset_right_));
    double y_aft = sqrt(abs(pow(rover_aft_sieve_left_, 2) - pow(x_aft - sieve_offset_left_, 2)));
    
    
    //calulate position
    x_ = (x_fwd + x_aft)/2;
    y_ = (y_fwd + y_aft)/2;
    
    //calculate pose
    
    x_fwd -= x_aft;
    y_fwd -= y_aft;
    
    th_ = 180 - acos(x_fwd/y_fwd);
    
    vx_  = (x_ - x_prev_)/(current_time_.toSec() - last_time_.toSec());
    vy_  = (y_ - y_prev_)/(current_time_.toSec() - last_time_.toSec());
    vth_ = (th_ - th_prev_)/(current_time_.toSec() - last_time_.toSec());
}
            
void RoverOdometry::computeTransform(){
                //since all odometry is 6DOF we'll need a quaternion created from yaw
                odom_quat_ = tf::createQuaternionMsgFromYaw(th_);
                //send the transform
                odom_trans.header.stamp = current_time_;
                odom_trans.header.frame_id = "odom";
                odom_trans.child_frame_id = "base_link";
                
                odom_trans.transform.translation.x = x_;
                odom_trans.transform.translation.y = y_;
                odom_trans.transform.translation.z = 0.0;
                odom_trans.transform.rotation = odom_quat_;
                
                odom_broadcaster_.sendTransform(odom_trans);
}
            
void RoverOdometry::publishOdometry(){
                nav_msgs::Odometry odom;
                
                odom.header.stamp    = current_time_;
                odom.header.frame_id = "odom";
                
                //set the position
                odom.pose.pose.position.x = x_;
                odom.pose.pose.position.y = y_;
                odom.pose.pose.position.z = 0.0;
                odom.pose.pose.orientation = odom_quat_;
                
                //set the velocity
                odom.child_frame_id        = "base_link";
                odom.twist.twist.linear.x  = vx_;
                odom.twist.twist.linear.y  = vy_;
                odom.twist.twist.angular.z = vth_;
                
                //publish the message
                odom_pub_.publish(odom);
            }

void RoverOdometry::sampleDriver(char * sample){
    //Take in values for determing which beacons are communicating
    int rover_id = rover_beacon_id_fwd_;
    int sieve_id = atoi(strtok(sample,","));
    double distance = atof(strtok(NULL, ""));

    if (rover_id == rover_beacon_id_fwd_ && sieve_id == sieve_beacon_id_left_){
        if( rover_fwd_sieve_left_ == 0 ) {
            distance = SAMPLE_SIZE*distance;
        }
        rover_fwd_sieve_left_ -= rover_fwd_sieve_left_/SAMPLE_SIZE;
        rover_fwd_sieve_left_ += distance/SAMPLE_SIZE;
    }
    else if (rover_id == rover_beacon_id_fwd_ && sieve_id == sieve_beacon_id_right_) {
        if( rover_fwd_sieve_right_ == 0 ){
            distance = SAMPLE_SIZE*distance;
        }
            rover_fwd_sieve_right_ -= rover_fwd_sieve_right_/SAMPLE_SIZE;
            rover_fwd_sieve_right_ += distance/SAMPLE_SIZE;
    }
    else if (rover_id == rover_beacon_id_aft_ && sieve_id == sieve_beacon_id_left_) {
        if( rover_aft_sieve_left_ == 0 ) {
            distance = SAMPLE_SIZE*distance;
        }

            rover_aft_sieve_left_ -= rover_aft_sieve_left_/SAMPLE_SIZE;
            rover_aft_sieve_left_ += distance/SAMPLE_SIZE;
    }
    else if (rover_id == rover_beacon_id_aft_ && sieve_id == sieve_beacon_id_right_) {
        if( rover_aft_sieve_right_ == 0 ) {
            distance = SAMPLE_SIZE*distance;
        }
            rover_aft_sieve_right_ -= rover_aft_sieve_right_/SAMPLE_SIZE;
            rover_aft_sieve_right_ += distance/SAMPLE_SIZE;
    }
}     
int main(int argc, char** argv)
    {
        ros::init(argc, argv, "rover_odometry");
        RoverOdometry rover_odometry;
        int counter = 0;
        ros::Rate r(1.0);
        while(ros::ok()){
            rover_odometry.driverprogram_ = popen(rover_odometry.drivercmd_.c_str(), "r");
	    char data[128]= "23,23,12.009";
            while(!feof(rover_odometry.driverprogram_)){
		if(fgets(data, sizeof data,rover_odometry.driverprogram_) != NULL){
			char* tmp = data;
			while(!isdigit((int)*tmp)) ++tmp;
			if(tmp[0] != 's' && (counter % 10) == 0)
				rover_odometry.sampleDriver(tmp);
				rover_odometry.updatePosePosition();
                    		// get time message from ROS system
                    		rover_odometry.current_time_ = ros::Time::now();
                    		//first, we'll publish the transform over tf
                    		rover_odometry.computeTransform();
                    		//next, we'll publish the odometry message over ROS
                    		rover_odometry.publishOdometry();
			if(counter % 400 == 0 && counter != 0) return 0;		
		}
		++counter;
	    }
        }
    }
            

