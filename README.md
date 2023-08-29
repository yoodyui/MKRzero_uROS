# MKRzero_uROS
This package contains some sample codes to publish data with various types, including yaw angle in ROS2 platform. The MKR zero arduino was tested succesfully.

1)	Install micro-ROS in the Ubuntu as steps in https://micro.ros.org/docs/tutorials/core/teensy_with_arduino/
2)	Installation of Arduino IDE ( Arduino 1.8.15 ) in Ubuntu using the link: https://docs.arduino.cc/software/ide-v1/tutorials/Linux
3)	Download the micro-ros client package from 
https://github.com/micro-ROS/micro_ros_arduino/releases
4) Extract and put the micro-ros client package in arduino/libraries folder
parallels@ubuntu22-04:~/arduino-1.8.19/libraries/micro_ros_arduino-2.0.7-humble
5) Open publishing example from the package in Arduino IDE and upload the code to MKR Zero
6) Running micro-ROS agent in ROS 2 host, according to https://micro.ros.org/docs/tutorials/core/teensy_with_arduino/


#Here is how to use the codes
1)	Upload code (either imu_uros.ino or uros_pub,ino) to MKR zero using Arduino IDE
2)	Open a terminal and create your workspace: $ cd microros_ws
3)	souce using: $ source install/local_setup.bash
4)	run the node using : $ ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
   
6)	Open another terminal: $ ros2 topic list
7)	Echo the ros message: $ ros2 topic echo yaw
