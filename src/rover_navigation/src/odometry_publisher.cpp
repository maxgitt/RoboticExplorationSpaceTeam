#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>


class OdometryPublisher
{
public:
  OdometryPublisher();

private:
  void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);
  ros::NodeHandle nh_;
  tf::TransformBroadcaster tf_broadcaster;
  ros::Publisher odom_pub;
  ros::Subscriber odom_sub;
    
  ros::Time current_time;
  ros::Time last_time;
};


OdometryPublisher::OdometryPublisher()
{
  odom_pub = nh_.advertise<nav_msgs::Odometry>("odom", 50);

  odom_sub = nh_.subscribe<nav_msgs::Odometry>("/odom", 1, &OdometryPublisher::odomCallback, this);

  current_time = ros::Time::now();
  last_time = ros::Time::now();
}

void OdometryPublisher::odomCallback(const nav_msgs::Odometry::ConstPtr& old_odom)
{
	nav_msgs::Odometry odom = *old_odom;

  tf_broadcaster.sendTransform(
        tf::StampedTransform(
          tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),
          ros::Time::now(),"map", "/gazebo/odom"));

  // tf_broadcaster.sendTransform(
  //       tf::StampedTransform(
  //         tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),
  //         ros::Time::now(),"base_link", "laser"));
  
  //since all odometry is 6DOF we'll need a quaternion created from yaw
  geometry_msgs::Quaternion odom_quat = odom.pose.pose.orientation;

  //first, we'll publish the transform over tf
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = ros::Time::now();
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = odom.pose.pose.position.x;
  odom_trans.transform.translation.y = odom.pose.pose.position.y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  //send the transform
  tf_broadcaster.sendTransform(odom_trans);


  //next, we'll publish the odometry message over ROS
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";
  //publish the message
  odom_pub.publish(odom);

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "odometry_publisher");
  OdometryPublisher odom;
  ros::Rate r(10); 
  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }  
}
