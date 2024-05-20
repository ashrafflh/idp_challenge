# Detection of Position Jumps and Simulation of Spoofing/Jamming Attacks

## Description

Developing a ROS2 node that detects sudden jumps in position data from Extended Kalman Filters (EKFs) and simulate spoofing/jamming attacks on GPS inputs using the MOBSTA package.

## ROS2 bag

We read the information about the provided bag with ros2 bag info. We find multiple topics:

| topic name | message type | utility |
|-----------------|-----------------|-----------------|
| /clock  | rosgraph_msgs/msg/Clock | provides the system time in the ROS network |
| diffbot_base_controller/cmd_vel_out | geometry_msgs/msg/TwistStamped | provides velocity information (linear and angular) with a timestamp to the robot's base controller |
| /diffbot_base_controller/cmd_vel_unstamped | geometry_msgs/msg/Twist | similar as the topic above but without a timestamp |
| /diffbot_base_controller/odom_corrected | nav_msgs/msg/Odometry | provides corrected odemetry information for the robot including the robot's position, orientation (pose) and velocity
| /gps/fix  | sensor_msgs/msg/NavSatFix | provides GPS information including latitude, longitude, altitude and position invariance. |
| /gps/reversed_navheading  | sensor_msgs/msg/Imu | provides IMU data including orientation, angular velocity, and linear acceleration in a reversed format |
| /imu/data  | sensor_msgs/msg/Imu | provides IMU data not reversed |
| /joy_vel  | geometry_msgs/msg/Twist | provides velocity commands (linear and angular) derived from a joystick input.  |
| /odometry/global  | nav_msgs/msg/Odometry | provides global odometry information, providing the robot's pose and velocity in a global reference frame. |
| /odometry/gps  | nav_msgs/msg/Odometry | provides odometry data derived from GPS information. It includes the robot's pose and velocity |
| /odometry/local  | nav_msgs/msg/Odometry | provides local odometry information, providing the robot's pose and velocity in a local reference frame.  |
| /tf  | tf2_msgs/msg/TFMessage | provides the transformation tree, which includes all the coordinate frames of the robot and their relative positions and orientations. |
| /tf_static  | tf2_msgs/msg/TFMessage | provides static transformations, which are transformations that do not change over time.  |


## Node development

We create a package for the jump detection task:
ros2 pkg create jump_detection --build-type ament_python --dependencies rclpy

We write the node in the jump_detection.py file in the jump_detection package, documentation to the code can be found in the source .py file.
The node currently monitors and detects jumps in the latitude and longitude data provided from the /gps/fix topic, as this is the topic that will be mutated by MOBSTA. The same code logic can be used to work on other position data topics such as /odometry/local and /odometry/global.

## Testing with MOBSTA

We download the repo

we build the mobsta package, it fails at rosbag2_py, probably due to conflicting versions:
Error message: error: invalid use of incomplete type ‘PyFrameObject’ {aka ‘struct _frame’}   482 |             frame = frame->f_back;


We create the configuration folder for our test case:

We play the data from the bag:
ros2 bag play rosbag2_2024_04_24-12_23_04_0.db3




