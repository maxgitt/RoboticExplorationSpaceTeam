#include <Arduino.h>

#include <Wire.h>
#include <CommunicationUtils.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>
#include <FreeSixIMU.h>
//#include <DebugUtils.h>

float q[3] = {0, 0, 0};
float g[3] = {0 , 0, 0};
int a[3] = {0, 0, 0};

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
	sixDOF.getYawPitchRoll(q);
	sixDOF.gyro.readGyro(&g[0], &g[1], &g[2]);
    sixDOF.acc.readAccel(&a[0], &a[1], &a[2]);

	serialPrintFloatArr(q, 3);
	serialPrintFloatArr(g,3);
	for(int i = 0; i < 3; ++i) {
		Serial.print(" ");
		Serial.print(a[i]);
	}
	Serial.println(); //line break
}