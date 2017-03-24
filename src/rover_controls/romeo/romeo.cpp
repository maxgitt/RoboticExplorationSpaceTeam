#include <Arduino.h>

#include "romeo_controls.h"
#include "romeo_encoders.h"

void 
setup() {
  Serial.begin(9600);
  encoders_setup();
}

void 
loop() {
  int cmd = 0;
  if(Serial.available()){
    cmd = Serial.parseInt();
  }

  if(cmd == 1 ) {
    controls_process();
  }

  encoders_process();
}

