#include <Arduino.h>

#include <Wire.h>
#include <CommunicationUtils.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>
#include <FreeSixIMU.h>
#include <DebugUtils.h>

float accel[3]; // yaw pitch roll
float angle[3]; // yaw pitch roll
FreeSixIMU sixDOF = FreeSixIMU();

void
imu_setup() {
  Wire.begin();
  delay(5);
  sixDOF.init(); //begin the IMU
  delay(5);
}

void
imu_process() {
    sixDOF.getValues(accel);
    sixDOF.getAngles(angle);
    serialPrintFloatArr(accel, 4);
    Serial.println(""); 
    serialPrintFloatArr(angle, 4);
    Serial.println(""); //line break
    //imu_msg.header.frame_id = "imu";
    // imu_msg.orientation.x = ns.pitch();
    // imu_msg.orientation.y = ns.roll();
    // imu_msg.orientation.z = ns.yaw();
    // imu_msg.orientation.w = 0.0;
    // //imu_msg.angular_velocity_covariance = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    // imu_msg.angular_velocity.x = ns.omgx();
    // imu_msg.angular_velocity.y = ns.omgy();
    // imu_msg.angular_velocity.z = ns.omgz();
    // //imu_msg.angular_velocity_covariance = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    // imu_msg.linear_acceleration.x = ns.accx();
    // imu_msg.linear_acceleration.y = ns.accy();
    // imu_msg.linear_acceleration.z = ns.accz();
    //imu_msg.angular_velocity_covariance = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};  
    //publish the new imu message
}