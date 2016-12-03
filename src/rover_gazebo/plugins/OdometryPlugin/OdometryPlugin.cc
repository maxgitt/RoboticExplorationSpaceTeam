#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"

#include <thread>
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <cassert>
#include <string>
#include <iostream>

using std::cerr; using std::endl;

namespace gazebo
{
  class ModelOdometry : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
      cerr << "Odometry plugin loading" << endl;
      // ROS Setup 
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client",
            ros::init_options::NoSigintHandler);
      }

      // Create our ROS node. This acts in a similar manner to
      // the Gazebo node
      this->rosNode.reset(new ros::NodeHandle("gazebo_client"));
      this->rosPub = rosNode->advertise<nav_msgs::Odometry>("odometry", 5);

      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&ModelOdometry::OnUpdate, this, _1));
    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      math::Vector3 pos    = this->model->GetWorldPose().pos;
      math::Quaternion rot = this->model->GetWorldPose().rot;

      math::Vector3 linVel = this->model->GetWorldAngularVel();
      math::Vector3 angVel = this->model->GetWorldLinearVel();

      geometry_msgs::Quaternion odom_quat;
      
      //first, we'll publish the transform over tf
      ros::Time current_time = ros::Time::now();
      geometry_msgs::TransformStamped odom_trans;
      odom_trans.header.stamp = current_time;
      odom_trans.header.frame_id = "odom";
      odom_trans.child_frame_id = "base_link";

      odom_trans.transform.translation.x = pos.x;
      odom_trans.transform.translation.y = pos.y;
      odom_trans.transform.translation.z = pos.z;
      odom_trans.transform.rotation.x = rot.x;
      odom_trans.transform.rotation.y = rot.y;
      odom_trans.transform.rotation.z = rot.z;

      //send the transform
      odom_broadcaster.sendTransform(odom_trans);

      //next, we'll publish the odometry message over ROS
      nav_msgs::Odometry odom;
      odom.header.stamp = current_time;
      odom.header.frame_id = "odom";

      //set the position
      odom.pose.pose.position.x = pos.x;
      odom.pose.pose.position.y = pos.y;
      odom.pose.pose.position.z = pos.z;
      odom.pose.pose.orientation.x = rot.x;
      odom.pose.pose.orientation.y = rot.y;
      odom.pose.pose.orientation.z = rot.z;

      //set the velocity
      odom.child_frame_id = "base_link";
      odom.twist.twist.linear.x = linVel.x;
      odom.twist.twist.linear.y = linVel.y;
      odom.twist.twist.linear.z = linVel.z;
      odom.twist.twist.angular.x = angVel.x;
      odom.twist.twist.angular.y = angVel.y;
      odom.twist.twist.angular.z = angVel.z;

      
      //publish the message
      rosPub.publish(odom);
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;

    // A ROS publisher
    private: ros::Publisher rosPub;

    // Used for broadcasting transform odom frame to base
    private: tf::TransformBroadcaster odom_broadcaster; 

  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelOdometry)
}