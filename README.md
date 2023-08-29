# MKRzero_uROS
This package contains some sample codes to publish data with various types, including yaw angle in ROS2 platform. The MKR zero arduino was tested succesfully.

#Here is how to use the codes
1)	Upload code (either imu_uros.ino or uros_pub,ino) to MKR zero using Arduino IDE
2)	Open a terminal and create your workspace: $ cd microros_ws
3)	souce using: $ source install/local_setup.bash
4)	run the node using : $ ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
