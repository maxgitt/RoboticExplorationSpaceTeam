#include <Arduino.h>
#include "mega_imu.h"
#include <assert.h>

void 
setup() {
  Serial.begin(9600);
  imu_setup();
}

void 
loop() {
  	imu_process();
}

