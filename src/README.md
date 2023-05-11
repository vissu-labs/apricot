# Apricot Interview Task

## Setup
Clone the repository into your workspace and make 
```
git clone https://github.com/Apricot-Robotics/ar1.git
```
Launch the main file. The robot should spawn in a warehouse environment
```
roslaunch ar1_description main.launch
```

## About the Robot
- The main frame of reference is 'base_link'
- The robot has only 1 sensor and it publlishes its data to the /scan topic
- The drive topic of the robot is '/cmd_vel'
- The odometry topic and frame are 'odom'
- The footprint of the robot is 810mmx510mm with 'base_link' at its center

## Tasks
- [ ] There's an error in the robot's description package which won't show up as an error, but it might hinder your other tasks. Find out the error and correct it
- [ ] Setup a navigation stack for the robot. Use only the lidar data for mapping and navigation
  - Create a map with any package of your choice
  - Setup a navstack with any packages of your choice
  - Tune the parameters for the robot
- [ ] Write a Python/C++ script to send a sequence of navigation goals with the ROS action interface. Take at list of waypoints as input from the user and navigate to these points in order
- [ ] Write a Python/C++ script to publish RVIZ markers which tracks the path/area covered by the robot during the previous task
