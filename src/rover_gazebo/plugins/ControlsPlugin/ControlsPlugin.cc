#ifndef _CONTROLS_PLUGIN_HH_
#define _CONTROLS_PLUGIN_HH_

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"

#include <thread>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <iostream>

#define ROBOT_WIDTH .5

using std::endl; using std::cerr; using std::cout;

const std::string joint_name_left_wheel_c = "joint_left_wheel";
const std::string joint_name_right_wheel_c = "joint_right_wheel";

namespace gazebo
{
  /// \brief A plugin to control a Velodyne sensor.
  class ControlsPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: ControlsPlugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      cerr << "Controls plugin loading" << endl;
      // Store the pointer to the model
      this->model = _model;

      // #TODO add front wheels
      this->joint_left_wheel = this->model->GetJoint(joint_name_left_wheel_c);
      this->joint_right_wheel = this->model->GetJoint(joint_name_right_wheel_c);

      // Initialize ros, if it has not already bee initialized.
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

      // Create a named topic, and subscribe to it.
      ros::SubscribeOptions so =
        ros::SubscribeOptions::create<geometry_msgs::Twist>( "/vel_cmd", 1,
                        boost::bind(&ControlsPlugin::OnRosMsg, this, _1),
                                          ros::VoidPtr(), &this->rosQueue);
      this->rosSub = this->rosNode->subscribe(so);

      // Spin up the queue helper thread.
      this->rosQueueThread =
        std::thread(std::bind(&ControlsPlugin::QueueThread, this));


      // Setup a P-controller, with a gain of 0.1.
      this->pid_left_wheel = common::PID(0.1, 0, 0);
      this->pid_right_wheel = common::PID(0.1, 0, 0);


      // Apply the P-controller to the joint.
      this->model->GetJointController()->SetVelocityPID(
          this->joint_left_wheel->GetScopedName(), this->pid_left_wheel);

      // Apply the P-controller to the joint.
      this->model->GetJointController()->SetVelocityPID(
          this->joint_right_wheel->GetScopedName(), this->pid_right_wheel);
    }


    /// \brief Handle an incoming message from ROS
    /// \param[in] _msg A float value that is used to set the velocity
    /// of the Velodyne.
    public: void OnRosMsg(const geometry_msgs::TwistConstPtr & _msg)
    {
      // Set the joint's target velocity.
      double vel_x = _msg->linear.x;
      double vel_th = _msg->angular.z;
      double right_vel = 0;
      double left_vel = 0;
      if(abs(vel_x) == 0){
        // turning
        right_vel = vel_th * ROBOT_WIDTH / 2.0 * 5;
        left_vel = 180 - right_vel;
      } else if(abs(vel_th) == 0) {
        // forward / backward
        left_vel  = vel_x * 5;
        right_vel = vel_x * 5;
      } else {
        // moving doing arcs
        left_vel = (vel_x - vel_th * ROBOT_WIDTH / 2.0) * 5;
        right_vel = (vel_x + vel_th * ROBOT_WIDTH / 2.0) * 5;
      }

      this->model->GetJointController()->SetVelocityTarget(
          this->joint_right_wheel->GetScopedName(), right_vel);

      this->model->GetJointController()->SetVelocityTarget(
          this->joint_left_wheel->GetScopedName(), left_vel);

    }

    /// \brief ROS helper function that processes messages
    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the back left wheel
    private: physics::JointPtr joint_left_wheel;

    // Pointer to the back right wheel
    private: physics::JointPtr joint_right_wheel;

    /// \brief A PID controller for the joint.
    private: common::PID pid_left_wheel;

    /// \brief A PID controller for the joint.
    private: common::PID pid_right_wheel;

    /// \brief A node used for transport
    private: transport::NodePtr node;

    /// \brief A subscriber to a named topic.
    private: transport::SubscriberPtr sub;

    /// \brief A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief A ROS subscriber
    private: ros::Subscriber rosSub;

    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;

    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(ControlsPlugin)
}
#endif