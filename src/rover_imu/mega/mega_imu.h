#include <Arduino.h>

#include <Wire.h>
#include <CommunicationUtils.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>
#include <FreeSixIMU.h>
//#include <DebugUtils.h>

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
    serialPrintFloatArr(accel, 3);
    serialPrintFloatArr(angle, 3);
    Serial.println(""); //line break
}