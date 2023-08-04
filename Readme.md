# Extended Kalman Filter for State Estimation

## Implementation of Extended Kalman Filter to estimate x,y coordinates and the yaw of a differential drive robot

  * This project implements the extended kalman filter algorithm for a simulation of a robot moving in a circle. 
  * Python3 along with Robot Operating System (ROS) has been used to write the programs.
  * Turtlebot3 Burger was simulated in Gazebo
  * The robot uses the Odoemtery data from the simulation to get noisy position measurements and uses EKF to filter out the noise.

## Simulation environment

https://drive.google.com/file/d/1UnnRG-Ny9yWYHSDsCXH-D2pNmq8XANGd/view?usp=drive_link

## x-coordinate estimation

https://drive.google.com/file/d/1_lNyuT6mZ6KBt8gQanZW1CJ0rdTcXlDZ/view?usp=drive_link

## y-coordinate estimation

https://drive.google.com/file/d/1tXPtqlzsjHx7p0svbdGymxj4eaRX5MXA/view?usp=sharing

## yaw estimation

https://drive.google.com/file/d/1xxwmwqsUD_AgQlHpRAJh0bsUTzVtMaSd/view?usp=drive_link

* **filtered_odom/data is the state variable after filtering the noise**
* **noisy_odom/data is the state variable before filtering the noise**

## ROS Dependencies 

1. rospy
2. geometry_msgs
3. nav_msgs
4. tf

## Instructions to run the program

1. Make sure to have ROS Noetic installed in the system. http://wiki.ros.org/noetic/Installation/Ubuntu
2. Set up a ROS workspace. http://wiki.ros.org/catkin/Tutorials/create_a_workspace
3. Git clone this repository into the workspace.
4. Run **catkin_make** from the workspace directory in the terminal.
5. Run the command **roslaunch package_name ekf.launch** to launch the file that launches all the necessary nodes. Replace **package_name** with your package name
6. Use rqt_plot to visualize the unfiltered and filtered data. 


