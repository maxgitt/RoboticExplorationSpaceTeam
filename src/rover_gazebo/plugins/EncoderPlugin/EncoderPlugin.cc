#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"

#include "rover_encoder/Encoder.h"

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

const std::string joint_name_left_wheel_c = "joint_left_wheel";
const std::string joint_name_right_wheel_c = "joint_right_wheel";

namespace gazebo
{
  class ModelEncoder : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
      // ROS Setup 
      cerr << "Encoder plugin loading" << endl;

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
      this->rosPub = rosNode->advertise<rover_encoder::Encoder>("encoders", 5);

      // Store the pointer to the model
      this->model = _parent;

      if (!this->model->GetJoint(joint_name_left_wheel_c) 
              || !this->model->GetJoint(joint_name_right_wheel_c))
      {
        std::cerr << "joint count invalid. plugin not loaded\n";
        return;
      }

      // #TODO add front wheels
      this->joint_left_wheel = this->model->GetJoint(joint_name_left_wheel_c);
      this->joint_right_wheel = this->model->GetJoint(joint_name_right_wheel_c);

      assert(this->joint_left_wheel);
      assert(this->joint_right_wheel);

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&ModelEncoder::OnUpdate, this, _1));
    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      //cerr << "onupdate called" << endl;
      rover_encoder::EncoderPtr enc(new rover_encoder::Encoder);
      enc->left_wheel = this->joint_left_wheel->GetAngle(0).Radian();
      enc->right_wheel = this->joint_right_wheel->GetAngle(0).Radian();
      this->rosPub.publish(enc);
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the back left wheel
    private: physics::JointPtr joint_left_wheel;

    // Pointer to the back right wheel
    private: physics::JointPtr joint_right_wheel;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;

    // A ROS publisher
    private: ros::Publisher rosPub;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelEncoder)
}