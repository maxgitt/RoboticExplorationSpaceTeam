CONTENTS OF THIS FILE
---------------------
   
 * Introduction
 * Requirements
 * Recommended modules
 * Installation
 * Configuration
 * Troubleshooting
 * FAQ
 * Maintainers

 INTRODUCTION
 ---------------------
 This code will drive the motors. It takes serial input (from ROS)and will output to pins 5, 6, 10 and 11 of our Arduino Uno.

 REQUIREMENTS
 ---------------------
 Requires serial input in format: [unsigned int (0-180) delimiter (,)unsigned int (0-180) delimiter (,) unsigned int (0-180) delimiter (,) unsigned int (0-180) delimiter (\n)]

 Input:
 Generically: [X,X,X,X\n]
 Example: [25,10,25,10\n]

 Each spot in the serial input maps to a specific motor:
 Back Left motor: 1st number (speedL)
 Back Rigth motor: 2nd number (speedR)
 Front Left motor: 3rd number (speedLF)
 Front Right motor: 4th number (speedRF)

 I.e. [speedL,speedR,speedLF,speedRF]

 OUTPUT
 -----------------------
 Pin 5: Back Left motor (speedL) - PWM
 Pin 6: Front Left motor (speedLF) - PWM
 Pin 10: Back Right motor (speedR) - PWM
 Pin 11: Front Right motor (speedRF) - PWM

 TROUBLESHOOTING
 -----------------------
 Call or text  Peter Danger Larsen - 248-719-3488
