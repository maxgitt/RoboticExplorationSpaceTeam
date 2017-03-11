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

#include <urg_node/urg_c_wrapper.h>

using namespace urg_node;

URGCWrapper::URGCWrapper(const std::string& ip_address, const int ip_port, bool& using_intensity, bool& using_multiecho){
  // Store for comprehensive diagnostics
  ip_address_ = ip_address;
  ip_port_ = ip_port;
  serial_port_ = "";
  serial_baud_ = 0;


  long baudrate_or_port = (long)ip_port;
  const char *device = ip_address.c_str();

  int result = urg_open(&urg_, URG_ETHERNET, device, baudrate_or_port);
  if (result < 0) {
    std::stringstream ss;
    ss << "Could not open network Hokuyo:\n";
    ss << ip_address << ":" << ip_port << "\n";
    ss << urg_error(&urg_);
    throw std::runtime_error(ss.str());
  }

  initialize(using_intensity, using_multiecho);
}

URGCWrapper::URGCWrapper(const int serial_baud, const std::string& serial_port, bool& using_intensity, bool& using_multiecho){
  // Store for comprehensive diagnostics
  serial_baud_ = serial_baud;
  serial_port_ = serial_port;
  ip_address_ = "";
  ip_port_ = 0;

  long baudrate_or_port = (long)serial_baud;
  const char *device = serial_port.c_str();

  int result = urg_open(&urg_, URG_SERIAL, device, baudrate_or_port);
  if (result < 0) {
    std::stringstream ss;
    ss << "Could not open serial Hokuyo:\n";
    ss << serial_port << " @ " << serial_baud << "\n";
    ss << urg_error(&urg_);
    throw std::runtime_error(ss.str());
  }

  initialize(using_intensity, using_multiecho);
}

void URGCWrapper::initialize(bool& using_intensity, bool& using_multiecho){
  int urg_data_size = urg_max_data_size(&urg_);
  if(urg_data_size  > 5000){  // Ocassionally urg_max_data_size returns a string pointer, make sure we don't allocate too much space, the current known max is 1440 steps
    urg_data_size = 5000;
  }
  data_.resize(urg_data_size * URG_MAX_ECHO);
  intensity_.resize(urg_data_size * URG_MAX_ECHO);

  started_ = false;
  frame_id_ = "";
  first_step_ = 0;
  last_step_ = 0;
  cluster_ = 1;
  skip_ = 0;

  if(using_intensity){
    using_intensity = isIntensitySupported();
  }

  if(using_multiecho){
    using_multiecho = isMultiEchoSupported();
  }

  use_intensity_ = using_intensity;
  use_multiecho_ = using_multiecho;

  measurement_type_ = URG_DISTANCE;
  if(use_intensity_ && use_multiecho_){
    measurement_type_ = URG_MULTIECHO_INTENSITY;
  } else if(use_intensity_){
      measurement_type_ = URG_DISTANCE_INTENSITY;
  } else if(use_multiecho_){
    measurement_type_ = URG_MULTIECHO;
  }
}

void URGCWrapper::start(){
  if(!started_){
    int result = urg_start_measurement(&urg_, measurement_type_, 0, skip_);
    if (result < 0) {
        std::stringstream ss;
        ss << "Could not start Hokuyo measurement:\n";
        if(use_intensity_){
          ss << "With Intensity" << "\n";
        }
        if(use_multiecho_){
          ss << "With MultiEcho" << "\n";
        }
        ss << urg_error(&urg_);
        throw std::runtime_error(ss.str());
      }
    }
    started_ = true;
}

void URGCWrapper::stop(){
  urg_stop_measurement(&urg_);
  started_ = false;
}

URGCWrapper::~URGCWrapper(){
  stop();
  urg_close(&urg_);
}

// Takes the running avg of window size
// This normalizes/smooths intensity noise
// Shoud be tested with different window sizes
vector<int> smooth_intensities(int intensity_steps[], int num_steps, int window_size) {
    if (window_size % 2 == 0) {
        cerr << "Window size cannot be even\n";
    }
    if (window_size > num_steps) {
        cerr << "Window size cannot exceed num steps\n";
    }
    vector<int> smoothed_intensities(num_steps);
    int buf = (window_size/2);
    int running_sum = 0;
    
    //setup first window
    for (int i = 0; i < buf+1; ++i) {
        running_sum += intensity_steps[i];
    }
    smoothed_intensities[0] = running_sum/(buf+1);
    
    //Calculate remaining averages ignoring values outside of step range
    for (int i = 1; i < num_steps; ++i) {
        if (i <= buf) {
            running_sum += intensity_steps[i+buf];
            smoothed_intensities[i] = running_sum/(i+buf+1);
        }
        else if( i >= (num_steps-buf)) {
            running_sum -= intensity_steps[i-buf-1];
            smoothed_intensities[i] = running_sum/(num_steps-i+buf);
        }
        else {
            running_sum += intensity_steps[i+buf];
            running_sum -= intensity_steps[i-buf-1];
            smoothed_intensities[i] = running_sum/(window_size);
        }
    }
    
    return smoothed_intensities;
}

  
// Samples 1-2N steps
// Compares the average intensity of (1 to N) vs (N+1 to 2N)
// Determines if avg intensity change was large enough
// Returns: Step indices where intensity changes are above intensity delta threshold,
//              these steps are labeled as edges, decreasing step edge indices are multiplied by -1
vector<int> determine_intensity_edges(int intensity_steps[], int num_steps, int window_size, int intensity_delta_threshold) {
  // Edges are defined as the step after an intensity jump

  // If too many edges -- increase threshold
  // If not enough edges -- consider taking differences between non-adjacent samples aka account for climbing (or remove smoothing function)

  vector<int> intensity_edges;

  for (int i = 0; i < num_steps-(window_size*2); ++i) {
    
    int sample_sum_1 = 0;
    int sample_sum_2 = 0;

    // Calculate sum of sample 1 and sample 2
    for (int j = 0; j < window_size; ++j) {
      sample_sum_1 += intensity_steps[i+j];
      sample_sum_2 += intensity_steps[i+window_size+j];
    }

    // Determine averages
    int sample_avg_1 = sample_sum_1/window_size;
    int sample_avg_2 = sample_sum_2/window_size;
  
    // Determine if function increased or decreased beyond threshold
    int intensity_delta = sample_avg_2 - sample_avg_1;
    if (abs(intensity_delta) > intensity_delta_threshold) {
      // Intensity is increasing
      if (intensity_delta > 0) {
        // Select first step of second sample as edge index
        intensity_edges.push_back(i+window_size);
      }
      // Intensity is decreasing
      else {
        // Select first step of second sample as edge index
        intensity_edges.push_back(-(i+window_size));
      }
    }
  }
  return intensity_edges;
}

// Maintains temporary gap length in steps between edges
// Allows for +-N variability
// Updates temporary gap length as most recent gap between edges
// Resets gap length if new gap length is farther then (temp_gap+-N)
// Returns once k sequential gaps are found without updating gap length
// Assert that expected oscillation of edge intensity occurs
vector<int> find_flag_ends(vector<int>& edge_indices, int gap_epsilon, int exp_edges) {

  if (edge_indices.size() < 2 ) {
    cerr << "Must have at least 2 edges\n";
  }

  vector<int> flag_ends(2);
  vector<int> flag_pattern_edges;
  // Initialze found edges and push first edge into edge_indices
  int found_edges = 1;
  flag_pattern_edges.push_back(abs(edge_indices[0]));

  // Initialize gap length to #steps (aka distance) between first two edges
  int gap_length = abs(edge_indices[1]) - abs(edge_indices[0]);

  // Count useful edges
  for (int i = 0; i < edge_indices.size()-1 ; ++i) {
    // High->Low or Low->High intensity change 
    if (edge_indices[i] > 0 && edge_indices[i+1] < 0
      || edge_indices[i] < 0 && edge_indices[i+1] > 0) {
      // Valid intensity change 
      if ((abs(edge_indices[i+1]) - abs(edge_indices[i])) < (gap_length + gap_epsilon)
        && (abs(edge_indices[i+1]) - abs(edge_indices[i])) > (gap_length - gap_epsilon)) {
          // Valid gap_length
          // Update gap_length
          gap_length = abs(edge_indices[i+1]) - abs(edge_indices[i]);
          // Update found_edges and push back validated edge
          found_edges++;
          flag_pattern_edges.push_back(abs(edge_indices[i+1]));
      }
      // Gap_length was too large or small 
      else {
        // Reset to initial values
        found_edges = 1;
        flag_pattern_edges.clear();
        flag_pattern_edges.push_back(abs(edge_indices[i+1]));
      }
    }
    // Low->low or high->high intensity change
    else {
        // Reset to initial values
        found_edges = 1;
        flag_pattern_edges.clear();
        flag_pattern_edges.push_back(abs(edge_indices[i+1]));
    }
  }

  if (found_edges != exp_edges) {
    cerr << "Did not detect flag\n";
  }
  else {
    flag_ends.push_back(flag_pattern_edges[0]);
    flag_ends.push_back(flag_pattern_edges[exp_edges]);
  }
  return flag_ends;
}

// Takes in steps corresponding to flag ends, and array of distance values for each step
// Uses trig to compute the coordinates of the center of the rover,
//      relative to the center of the sieve at (0,0)
// Returns (x,y) coordinate vector of the rover
vector<double> get_position(vector<int>& flag_ends, int distance_steps[]) {

  double dist_left_end = distance_steps[flag_ends[0]];
  double dist_right_end = distance_steps[flag_ends[1]];

  vector<double> coordinates(2);

  coordinates.push_back((pow(dist_right_end, 2) - pow(dist_left_end, 2) + 6.111) / 3.15);
  coordinates.push_back(sqrt(-2560000*pow(dist_left_end,4) + 3200*pow(dist_left_end,2)*(1600*pow(dist_right_end,2) + 3969) - pow((3969 - 1600*pow(dist_right_end,2)),2))/5040);
  return coordinates;
}

// Takes in current rover (x,y) coordinates and step numbers of the flag ends
// Uses width of flag, dist to left end, dist to right end, and
//      the angle between the 540th (or real center) step vs the step corresponding 
//      to the flag center
// Orientation of 0 means rover is perpendicular to sieve
// Postive value means 
// Returns updated pose vector by appending rover's orientation to its position
vector<double> get_orientation(vector<int>& position, vector<int>& flag_ends, int distance_steps[]) {

  //
  
}

// Pose consists of the rover's position and orientation
// Publishes a vector containing the rover's pose (x,y,theta)
// 
void publish_pose(vector<double>& pose)

// Calculate the step closest to the center of the flag 
int calc_flag_center();

bool URGCWrapper::intensity_inrange(int low, int high, int arr[], int length) {
    int sum = 0;
    for (int i = 0; i < length; ++i) {
        sum += arr[i];
    }
    double avg = sum/length;
    if (low <= avg && avg <= high) {
        return true;
    }
    return false;
}

bool URGCWrapper::intensity_increases(int arr1[], int arr2[], int length){
    int sum = 0;
    for (int i = 0; i < length; ++i) {
        sum = arr2[i] - arr1[i];
    }
    if (sum > 0) {
        return true;
    }
    return false;
}

bool URGCWrapper::grabScan(const sensor_msgs::LaserScanPtr& msg){
  msg->header.frame_id = frame_id_;
  msg->angle_min = getAngleMin();
  msg->angle_max = getAngleMax();
  msg->angle_increment = getAngleIncrement();
  msg->scan_time = getScanPeriod();
  msg->time_increment = getTimeIncrement();
  msg->range_min = getRangeMin();
  msg->range_max = getRangeMax();
  
  // Grab scan
  int num_beams = 0;
  long time_stamp = 0;
  unsigned long long system_time_stamp = 0;

  if(use_intensity_){
    num_beams = urg_get_distance_intensity(&urg_, &data_[0], &intensity_[0], &time_stamp, &system_time_stamp);
  } else {
    num_beams = urg_get_distance(&urg_, &data_[0], &time_stamp, &system_time_stamp);
  } 
  if (num_beams <= 0) {
    return false;
  }

  // Fill scan
  msg->header.stamp.fromNSec((uint64_t)system_time_stamp);
  msg->header.stamp = msg->header.stamp + system_latency_ + user_latency_ + getAngularTimeOffset();
  msg->ranges.resize(num_beams);
  if(use_intensity_){
    msg->intensities.resize(num_beams);
  }
  
  for (int i = 0; i < num_beams; i++) {
    if(data_[(i) + 0] != 0){
      msg->ranges[i] = (float)data_[i]/1000.0;
      if(use_intensity_){
        msg->intensities[i] = intensity_[i];
      }
    } else {
      msg->ranges[i] = std::numeric_limits<float>::quiet_NaN();
      continue;
    }  
  }
  return true;
}

bool URGCWrapper::grabScan(const sensor_msgs::MultiEchoLaserScanPtr& msg){
  msg->header.frame_id = frame_id_;
  msg->angle_min = getAngleMin();
  msg->angle_max = getAngleMax();
  msg->angle_increment = getAngleIncrement();
  msg->scan_time = getScanPeriod();
  msg->time_increment = getTimeIncrement();
  msg->range_min = getRangeMin();
  msg->range_max = getRangeMax();
  
  // Grab scan
  int num_beams = 0;
  long time_stamp = 0;
  unsigned long long system_time_stamp;

  if(use_intensity_){
    num_beams = urg_get_multiecho_intensity(&urg_, &data_[0], &intensity_[0], &time_stamp, &system_time_stamp);
  } else {
    num_beams = urg_get_multiecho(&urg_, &data_[0], &time_stamp, &system_time_stamp);
  } 
  if (num_beams <= 0) {
    return false;
  }

  // Fill scan (uses vector.reserve wherever possible to avoid initalization and unecessary memory expansion)
  msg->header.stamp.fromNSec((uint64_t)system_time_stamp);
  msg->header.stamp = msg->header.stamp + system_latency_ + user_latency_ + getAngularTimeOffset();
  msg->ranges.reserve(num_beams);
  if(use_intensity_){
    msg->intensities.reserve(num_beams);
  } 

  for (size_t i = 0; i < num_beams; i++) {
    sensor_msgs::LaserEcho range_echo;
    range_echo.echoes.reserve(URG_MAX_ECHO);
    sensor_msgs::LaserEcho intensity_echo;
    if(use_intensity_){
      intensity_echo.echoes.reserve(URG_MAX_ECHO);
    }
    for(size_t j = 0; j < URG_MAX_ECHO; j++){
      if(data_[(URG_MAX_ECHO * i) + j] != 0){
        range_echo.echoes.push_back((float)data_[(URG_MAX_ECHO * i) + j]/1000.0f);
        if(use_intensity_){
          intensity_echo.echoes.push_back(intensity_[(URG_MAX_ECHO * i) + j]);
        }
      } else {
        break;
      }
    }
    msg->ranges.push_back(range_echo);
    if(use_intensity_){
      msg->intensities.push_back(intensity_echo);
    }
  }

  return true;
}

bool URGCWrapper::isStarted() const{
  return started_;
}

double URGCWrapper::getRangeMin() const{
  long minr;
  long maxr;
  urg_distance_min_max(&urg_, &minr, &maxr);
  return (double)minr/1000.0;
}

double URGCWrapper::getRangeMax() const{
  long minr;
  long maxr;
  urg_distance_min_max(&urg_, &minr, &maxr);
  return (double)maxr/1000.0;
}

double URGCWrapper::getAngleMin() const{
  return urg_step2rad(&urg_, first_step_);
}

double URGCWrapper::getAngleMax() const{
  return urg_step2rad(&urg_, last_step_);
}

double URGCWrapper::getAngleMinLimit() const{
  int min_step;
  int max_step;
  urg_step_min_max(&urg_, &min_step, &max_step);
  return urg_step2rad(&urg_, min_step);
}

double URGCWrapper::getAngleMaxLimit() const{
  int min_step;
  int max_step;
  urg_step_min_max(&urg_, &min_step, &max_step);
  return urg_step2rad(&urg_, max_step);
}

double URGCWrapper::getAngleIncrement() const{
  double angle_min = getAngleMin();
  double angle_max = getAngleMax();
  return cluster_*(angle_max-angle_min)/(double)(last_step_-first_step_);
}

double URGCWrapper::getScanPeriod() const{
  long scan_usec = urg_scan_usec(&urg_);
  return 1.e-6*(double)(scan_usec);
}

double URGCWrapper::getTimeIncrement() const{
  int min_step;
  int max_step;
  urg_step_min_max(&urg_, &min_step, &max_step);
  double scan_period = getScanPeriod();
  double circle_fraction = (getAngleMaxLimit()-getAngleMinLimit())/(2.0*3.141592);
  return cluster_*circle_fraction*scan_period/(double)(max_step-min_step);
}


std::string URGCWrapper::getIPAddress() const{
  return ip_address_;
}

int URGCWrapper::getIPPort() const{
  return ip_port_;
}

std::string URGCWrapper::getSerialPort() const{
  return serial_port_;
}

int URGCWrapper::getSerialBaud() const{
  return serial_baud_;
}

std::string URGCWrapper::getVendorName(){
  return std::string(urg_sensor_vendor(&urg_));
}

std::string URGCWrapper::getProductName(){
  return std::string(urg_sensor_product_type(&urg_));
}

std::string URGCWrapper::getFirmwareVersion(){
  return std::string(urg_sensor_firmware_version(&urg_));
}

std::string URGCWrapper::getFirmwareDate(){
  return std::string(urg_sensor_firmware_date(&urg_));
}

std::string URGCWrapper::getProtocolVersion(){
  return std::string(urg_sensor_protocol_version(&urg_));
}

std::string URGCWrapper::getDeviceID(){
  return std::string(urg_sensor_serial_id(&urg_));
}

ros::Duration URGCWrapper::getComputedLatency() const{
  return system_latency_;
}

ros::Duration URGCWrapper::getUserTimeOffset() const{
  return user_latency_;
}

std::string URGCWrapper::getSensorStatus(){
  return std::string(urg_sensor_status(&urg_));
}

std::string URGCWrapper::getSensorState(){
  return std::string(urg_sensor_state(&urg_));
}

void URGCWrapper::setFrameId(const std::string& frame_id){
  frame_id_ = frame_id;
}

void URGCWrapper::setUserLatency(const double latency){
  user_latency_.fromSec(latency);
}

// Must be called before urg_start
bool URGCWrapper::setAngleLimitsAndCluster(double& angle_min, double& angle_max, int cluster){
  if(started_){
    return false; // Must not be streaming
  }

  // Set step limits
  first_step_ = urg_rad2step(&urg_, angle_min);
  last_step_ = urg_rad2step(&urg_, angle_max);
  cluster_ = cluster;

  // Make sure step limits are not the same
  if(first_step_ == last_step_){
    // Make sure we're not at a limit
    int min_step;
  int max_step;
  urg_step_min_max(&urg_, &min_step, &max_step);
  if(first_step_ == min_step){ // At beginning of range
    last_step_ = first_step_ + 1;
  } else { // At end of range (or all other cases)
    first_step_ = last_step_ - 1;
  } 
  }

  // Make sure angle_max is greater than angle_min (should check this after end limits)
  if(last_step_ < first_step_){
    double temp = first_step_;
    first_step_ = last_step_;
    last_step_ = temp;
  }

  angle_min = urg_step2rad(&urg_, first_step_);
  angle_max = urg_step2rad(&urg_, last_step_);
  int result = urg_set_scanning_parameter(&urg_, first_step_, last_step_, cluster);
  if(result < 0){
    return false;
  }
  return true;
}

bool URGCWrapper::setSkip(int skip){
  skip_ = skip;
}

bool URGCWrapper::isIntensitySupported(){
  if(started_){
    return false; // Must not be streaming
  }

  urg_start_measurement(&urg_, URG_DISTANCE_INTENSITY, 0, 0);
  int ret = urg_get_distance_intensity(&urg_, &data_[0], &intensity_[0], NULL, NULL);
  if(ret <= 0){
    return false; // Failed to start measurement with intensity: must not support it
  }
  urg_stop_measurement(&urg_);
  return true;
}

bool URGCWrapper::isMultiEchoSupported(){
  if(started_){
    return false; // Must not be streaming
  }

  urg_start_measurement(&urg_, URG_MULTIECHO, 0, 0);
  int ret = urg_get_multiecho(&urg_, &data_[0], NULL, NULL);
  if(ret <= 0){
    return false; // Failed to start measurement with multiecho: must not support it
  }
  urg_stop_measurement(&urg_);
  return true;
}

ros::Duration URGCWrapper::getAngularTimeOffset() const{
  // Adjust value for Hokuyo's timestamps
  // Hokuyo's timestamps start from the rear center of the device (at Pi according to ROS standards)
  double circle_fraction = 0.0;
  if(first_step_ == 0 && last_step_ == 0){
    circle_fraction = (getAngleMinLimit()+3.141592)/(2.0*3.141592);
  } else {
    circle_fraction = (getAngleMin()+3.141592)/(2.0*3.141592);
  }
  return ros::Duration(circle_fraction * getScanPeriod());
}

ros::Duration URGCWrapper::computeLatency(size_t num_measurements){
  system_latency_.fromNSec(0);

  ros::Duration start_offset = getNativeClockOffset(1);
  ros::Duration previous_offset;

  std::vector<ros::Duration> time_offsets(num_measurements);
  for (size_t i = 0; i < num_measurements; i++){
    ros::Duration scan_offset = getTimeStampOffset(1);
    ros::Duration post_offset = getNativeClockOffset(1);
    ros::Duration adjusted_scan_offset = scan_offset - start_offset;
    ros::Duration adjusted_post_offset = post_offset - start_offset;
    ros::Duration average_offset;
    average_offset.fromSec((adjusted_post_offset.toSec() + previous_offset.toSec())/2.0);

    time_offsets[i] = adjusted_scan_offset - average_offset;

    previous_offset = adjusted_post_offset;
  }

  // Get median value
  // Sort vector using nth_element (partially sorts up to the median index)
  std::nth_element(time_offsets.begin(), time_offsets.begin() + time_offsets.size()/2, time_offsets.end());
  system_latency_ = time_offsets[time_offsets.size()/2];
  return system_latency_ + getAngularTimeOffset(); // Angular time offset makes the output comparable to that of hokuyo_node
}

ros::Duration URGCWrapper::getNativeClockOffset(size_t num_measurements){
  if(started_){
    std::stringstream ss;
    ss << "Cannot get native clock offset while started.";
    throw std::runtime_error(ss.str());
  }

  if(urg_start_time_stamp_mode(&urg_) < 0){
    std::stringstream ss;
    ss << "Cannot start time stamp mode.";
    throw std::runtime_error(ss.str());
  }

  std::vector<ros::Duration> time_offsets(num_measurements);
  for (size_t i = 0; i < num_measurements; i++)
  {
    ros::Time request_time = ros::Time::now();
    ros::Time laser_time;
    laser_time.fromNSec(1e6*(uint64_t)urg_time_stamp(&urg_)); // 1e6 * milliseconds = nanoseconds
    ros::Time response_time = ros::Time::now();
    ros::Time average_time;
    average_time.fromSec((response_time.toSec()+request_time.toSec())/2.0);
    time_offsets[i] = laser_time - average_time;
  }

  if(urg_stop_time_stamp_mode(&urg_) < 0){
    std::stringstream ss;
    ss << "Cannot stop time stamp mode.";
    throw std::runtime_error(ss.str());
  };

  // Return median value
  // Sort vector using nth_element (partially sorts up to the median index)
  std::nth_element(time_offsets.begin(), time_offsets.begin() + time_offsets.size()/2, time_offsets.end());
  return time_offsets[time_offsets.size()/2];
}

ros::Duration URGCWrapper::getTimeStampOffset(size_t num_measurements){
  if(started_){
    std::stringstream ss;
    ss << "Cannot get time stamp offset while started.";
    throw std::runtime_error(ss.str());
  }

  start();

  std::vector<ros::Duration> time_offsets(num_measurements);
  for(size_t i = 0; i < num_measurements; i++){
    long time_stamp;
    unsigned long long system_time_stamp;
    int ret = 0;

    if(measurement_type_ == URG_DISTANCE){
      ret = urg_get_distance(&urg_, &data_[0], &time_stamp, &system_time_stamp);
    } else if(measurement_type_ == URG_DISTANCE_INTENSITY){
      ret = urg_get_distance_intensity(&urg_, &data_[0], &intensity_[0], &time_stamp, &system_time_stamp);
    } else if(measurement_type_ == URG_MULTIECHO){
      ret = urg_get_multiecho(&urg_, &data_[0], &time_stamp, &system_time_stamp);
    } else if(measurement_type_ == URG_MULTIECHO_INTENSITY){
      ret = urg_get_multiecho_intensity(&urg_, &data_[0], &intensity_[0], &time_stamp, &system_time_stamp);
    }

    if(ret <= 0){
      std::stringstream ss;
      ss << "Cannot get scan to measure time stamp offset.";
      throw std::runtime_error(ss.str());
    }

    ros::Time laser_timestamp;
    laser_timestamp.fromNSec(1e6*(uint64_t)time_stamp);
    ros::Time system_time;
    system_time.fromNSec((uint64_t)system_time_stamp);

    time_offsets[i] = laser_timestamp - system_time;
  }

  stop();

  // Return median value
  // Sort vector using nth_element (partially sorts up to the median index)
  std::nth_element(time_offsets.begin(), time_offsets.begin() + time_offsets.size()/2, time_offsets.end());
  return time_offsets[time_offsets.size()/2];
}
