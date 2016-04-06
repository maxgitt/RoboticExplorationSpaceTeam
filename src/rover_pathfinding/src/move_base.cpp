#include "move_base.h"
using std::vector;

void
MoveBase::setup(int argc, char ** argv){
	// Set up Odometry Subscriber and cmd_vel Publisher
	ros::init(argc, argv, "odom_listener");
	sub = nh.subscribe("odom", 1000, &MoveBase::odomcb, this);
	pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);   
	srv = nh.advertiseService("pathfinding", MoveBase::pathcb, this);
	// Construct PID objects
	linearx = new PID(&currx, &linearx_out, &destx, 
						Kp, Ki, Kd, AUTOMATIC);
	lineary = new PID(&curry, &lineary_out, &desty,
						Kp, Ki, Kd, AUTOMATIC);
	angularz = new PID(&currth, &angularz_out, &destth,
						Kp, Ki, Kd, AUTOMATIC);
}

MoveBase::MoveBase(int argc, char**argv){
	setup(argc, argv);

	// Listen to network until currx, curry, and currth 
	// has been updated
    	ros::Rate loop_rate(10);
	while(currx == -100 || curry == -100 || currth == -100){
		ros::spinOnce();
		loop_rate.sleep();
	}

	// Set Initial and Current position to values passed 
	// into constructor
	initx  = currx; 
	inity  = curry; 
	initth = currth; 
}

MoveBase::MoveBase(int argc, char **argv, double _initx, double _inity, double _initth){
	setup(argc, argv);
	// Set Initial and Current position to values passed 
	// into constructor
	initx  = currx  = _initx;
	inity  = curry  = _inity;
	initth = currth = _initth;
}

void
extract_path_array(rover_pathfinding::global_goal::Request  &req,
                                    vector<Coordinate> & path_vector){
    for (int i = 0; i < req.x_coordinate_array.size(); ++i) {
        Coordinate tmp(req.x_coordinate_array[i], 
        	       req.y_coordinate_array[i]);
        path_vector.push_back(tmp);
    }
}

bool 
MoveBase::pathcb(rover_pathfinding::global_goal::Request  &req,
    		 rover_pathfinding::global_goal::Response &res){
    bool ret = false;
    if(req.path_sent){
        vector<Coordinate> path_vector
        extract_path_array(req, path_vector);
        rover_mover.move_along(path_vector);
        return true;
    }
    else if(req.goal_sent) {
        rover_mover.move_to(req.global_x, req.global_y);
        return true;
    }
    else if(req.record_path){
        //rover_mover.record_path();
        return true;
    }
    else {
        assert(0);
    }
}



void 
MoveBase::odomcb(const nav_msgs::Odometry::ConstPtr& msg){
	// Update current position and orientation
	currx  = (msg->pose.pose.position.x);
	curry  = (msg->pose.pose.position.x);
	currth = (msg->pose.pose.orientation.z);
	
	// Update linearx, lineary, and linearth velocities
	linearx_in  = msg->twist.twist.linear.x;
	lineary_in  = msg->twist.twist.linear.y;
	angularz_in = msg->twist.twist.angular.z;
}

bool
MoveBase::at_dest(){
    double dist_from_dest;
    double xx = currx - destx;
    double yy = curry - desty;
    dist_from_dest = pow(xx,2)+pow(yy,2);
    //calculating distance by euclidean formula
    dist_from_dest = sqrt(dist_from_dest);                  
	//sqrt is function in math.h
	// y coordinates are only positive so we can take
	// absolute value and check against threshold
	if(dist_from_dest > dist_from_dest_thresh_c){
		return false;
	}
	return true;
}

void
MoveBase::move_along(std::vector<Coordinate>& path_vector){
	for(auto i: path_vector){
		move_to(i.x, i.y);
	}
}

void
MoveBase::move_to(double _destx, double _desty){
	destx = _destx;
	desty = _desty;
	
	double dx = (destx - currx);
	double dy = (desty - curry);
	destth  = atan2(dx, dy) * 180 / 3.141592;
	
	ros::Rate loop_rate(10);
	// While we are not within 2 decimals places of
	// the destination continue running
	while(ros::ok() && !at_dest()){
		// Compute new output
		linearx->Compute();
		lineary->Compute();
		angularz->Compute();
		
		// Publish new output to cmd_vel
		geometry_msgs::Twist msg;
		msg.linear.x  = linearx_out;
		msg.linear.y  = lineary_out;
		msg.angular.z = angularz_out; 
		pub.publish(msg);

		// Get new messages
		ros::spinOnce();
    	loop_rate.sleep();
	}
}


void 
MoveBase::record_path(){
	/*do {
		this_thread::sleep_for (chrono::seconds(2)); //need to calibrate how many seconds to wait
                                                 	 //this waits two secods before every push
		x_pos = currx;
		y_pos = curry;
		recorded_path.push_back(Coordinate(x_pos, y_pos));
		//push coordinates to pathway vector
	} while(abs(global_y - y_pos) <= .1);*/
}



MoveBase::~MoveBase(){
	// Clean Dyanmic Memory allocated by Object
	delete linearx;
	delete lineary;
	delete angularz;
}

