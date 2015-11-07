CONTENTS OF THIS FILE
---------------------
   
 * Introduction
 * Requirements
 * Installation
 * Configuration
 * Troubleshooting
 * FAQ
 * Maintainers

 INTRODUCTION
 ---------------------
 This program reads encoder positions and calculates a "speed" value.

 REQUIREMENTS
 ---------------------
 Load the "Encoder" file from the dependencies folder into your Arduino/libraries folder. 

 INSTALLATION
 ---------------------
 Download the teensy arduino driver for your OS from this link:
 https://www.pjrc.com/teensy/td_download.html
 Make sure "Encoder" is in your Arduino/libraries directory.

 CONFIGURATION
 ---------------------
 There's encoder 1-4
 Pin config:
 1: pin 5 and 6 
 2: pin 7 and 8
 3: pin 9 and 10
 4: pin 11 and 12
 The output is (encoder 1 speed), (encoder 2 speed), (encoder 3 speed), (encoder 4 speed)\n

 FAQ
 ---------------------
 A very rough approximation gave us 68000 encoder counts to get one full rotation.
 The program prints every 500ms.
 It prints X,X,X,X (X stands for a number (current encoder count - last encoder count) / 500
 Units are encoder counts/milliseconds

 MAINTAINERS
 ---------------------
 Peter Larsen - plarsen@umich.edu - 2487193488 - beastiegirls666@hotmail.com
 Kishore.....???? 



