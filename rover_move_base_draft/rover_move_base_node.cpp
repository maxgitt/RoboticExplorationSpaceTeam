#include <rover_move_base/rover_move_base.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "move_base_node");
  tf::TransformListener tf(ros::Duration(10));

  rover_move_base::RoverMoveBase rover_move_base( tf );

  ros::spin();

  return(0);
}
