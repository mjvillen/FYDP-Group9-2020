# Capstone

This is the git repository for Group 9 Fourth Year Design Project: Roboperation. The overall software flow of this project involves reading raw IMU data, linearizing this data with quaternion math, and then combining the readings with sensor fusion to produce reliable measurements. This process will be performed using the Arduino Due, and the code can be found in the Arduino sub directory.

The data will then be passed to a Panda Arm (Powertool) Controller by Franka Emika and provided by the University of Waterloo RoboHub for this project. The Panda controller then processes the passed position and actuates the arm.

Communication from the Arduino to a local laptop will be done using the default Serial USB protocol. This data will then be passed to the Panda Controller using a ROS node setup (Arduino ROS as well).
