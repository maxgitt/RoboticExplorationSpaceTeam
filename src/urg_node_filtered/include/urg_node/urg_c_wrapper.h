/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* 
 * Author: Chad Rockey
 */

#ifndef URG_C_WRAPPER_H
#define URG_C_WRAPPER_H

#include <stdexcept>
#include <sstream>
#include <limits>

#include <cmath>
#include <vector>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/MultiEchoLaserScan.h>

#include <urg_c/urg_sensor.h>
#include <urg_c/urg_utils.h>

 #include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

using namespace std;

namespace urg_node
{ 
  class URGCWrapper
  {
  public:
    URGCWrapper(const std::string& ip_address, const int ip_port, bool& using_intensity, bool& using_multiecho);

    URGCWrapper(const int serial_baud, const std::string& serial_port, bool& using_intensity, bool& using_multiecho);

    ~URGCWrapper();

    void start();

    void stop();

    bool isStarted() const;

    double getRangeMin() const;

    double getRangeMax() const;

    double getAngleMin() const;

    double getAngleMax() const;

    double getAngleMinLimit() const;

    double getAngleMaxLimit() const;

    double getAngleIncrement() const;

    double getScanPeriod() const;

    double getTimeIncrement() const;

    std::string getIPAddress() const;

    int getIPPort() const;

    std::string getSerialPort() const;

    int getSerialBaud() const;

    std::string getVendorName();

    std::string getProductName();

    std::string getFirmwareVersion();

    std::string getFirmwareDate();

    std::string getProtocolVersion();

    std::string getDeviceID();

    ros::Duration getComputedLatency() const;

    ros::Duration getUserTimeOffset() const;

    std::string getSensorStatus();

    std::string getSensorState();

    void setFrameId(const std::string& frame_id);

    void setUserLatency(const double latency);

    bool setAngleLimitsAndCluster(double& angle_min, double& angle_max, int cluster);

    bool setSkip(int skip);

    ros::Duration computeLatency(size_t num_measurements);

    bool grabScan(const sensor_msgs::LaserScanPtr& msg);

    bool grabScan(const sensor_msgs::MultiEchoLaserScanPtr& msg);

 private:
      
    void initialize(bool& using_intensity, bool& using_multiecho);

    bool isIntensitySupported();

    bool isMultiEchoSupported();

    ros::Duration getAngularTimeOffset() const;

    ros::Duration getNativeClockOffset(size_t num_measurements);

    ros::Duration getTimeStampOffset(size_t num_measurements);

    std::string frame_id_; ///< Output frame_id for each laserscan.

    urg_t urg_;
    bool started_;

    std::vector<long> data_;
    std::vector<unsigned short> intensity_;

    bool use_intensity_;
    bool use_multiecho_;
    urg_measurement_type_t measurement_type_;
    int first_step_;
    int last_step_;
    int cluster_;
    int skip_;

    ros::Duration system_latency_;
    ros::Duration user_latency_;

    std::string ip_address_;
    int ip_port_;
    std::string serial_port_;
    int serial_baud_;
  };
  
  
}; // urg_node

// Calculate the distance to the center of the flag 
double get_dist_to_flag_center(vector<double> position);

double get_angle_right_to_center(double dist_to_flag_center, vector<int>& flag_ends, double dist_to_right_flag_end);

// Takes the running avg of window size
// This normalizes/smooths intensity noise
// Shoud be tested with different window sizes
vector<float> smooth_intensities(vector<float> intensities, int num_steps, int window_size);

// Samples 1-2N steps
// Compares the average intensity of (1 to N) vs (N+1 to 2N)
// Determines if avg intensity change was large enough
// Returns: Step indices where intensity changes are above intensity delta threshold,
//              these steps are labeled as edges, decreasing step edge indices are multiplied by -1
vector<int> determine_intensity_edges(vector<float> intensities, int num_steps, int window_size, int intensity_delta_threshold);

// Maintains temporary gap length in steps between edges
// Allows for +-N variability
// Updates temporary gap length as most recent gap between edges
// Resets gap length if new gap length is farther then (temp_gap+-N)
// Returns once k sequential gaps are found without updating gap length
vector<int> find_flag_ends(vector<int>& edge_indices, int gap_epsilon, int exp_edges);

// Takes in steps corresponding to flag ends, and array of distance values for each step
// Uses trig to compute the coordinates of the center of the rover,
//      relative to the center of the sieve at (0,0)
// Returns (x,y) coordinate vector of the rover
vector<double> get_position(vector<int>& flag_ends, vector<float> distance_steps);

// Takes in current rover (x,y) coordinates and step numbers of the flag ends
// Uses width of flag, dist to left end, dist to right end, and
//      the angle between the 540th (or real center) step vs the step corresponding 
//      to the flag center
// Orientation of 0 means rover is perpendicular to sieve
// Postive value means facing left of center
// Negative value means facing right of center
// Returns updated pose vector by appending rover's orientation to its position
double get_orientation(vector<double>& position, vector<int>& flag_ends, vector<float> distance_steps);

// Pose consists of the rover's position and orientation
// Publishes a vector containing the rover's pose (x,y,theta)
geometry_msgs::PoseWithCovarianceStamped publish_pose(vector<double>& pose_in);

#endif
