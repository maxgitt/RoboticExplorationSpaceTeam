rosparam set /rover_odometry/sieve_beacon_ids "13,24";
rosparam set /rover_odomety/x13 "0,0";
rosparam set /rover_odomety/x24 "0,0";
rosparam set /rover_odomety/rover_beacon_id "48c36259,c3e3c227";
rosparam set /rover_odomety/r48c3625 ".25,0";
rosparam set /rover_odomety/rc3e3c227 "0,0";
rosparam set /rover_odomety/48c36259_24 "6.898045";
rosparam set /rover_odomety/c3e3c227_24 "7.00";
rosparam set /rover_odomety/48c36259_13 "4.38407";
rosparam set /rover_odomety/c3e3c227_13 "5.68";
sudo pkill dwm1000driver;
unbuffer sudo '/home/pascualy/catkin_ws/decawave_driver/dwm1000driver' 13 | ./devel/lib/rover_odometry/rover_odometry;


