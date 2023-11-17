# Sensors_Control
Subject 41014 Sensors and Control for Mechatronics Systems 

## Check out our demonstration 
- [Video](https://youtu.be/3MIpA-Jp_GI)
## Matlab
    matlab

## Project Description
This project aimed to develop the control system for a Turtlebot robot to autonomously follow a straight line, in addition to a complex trajectory, based on a image reading of a square. 
This was utilized by obtaining readings from an RGB-D sensor placed on the robot. 
This sensor can observe the corners of a square placed in front of the robot which prompts the Turtlebot to navigate towards a predefined target location by determining where the straight line on the floor would be. 
The main objective was to take sensor readings, filter out noise and control the robot to travel in a straight line. This project also involved creating two complex trajectories. 
This project was completed using the simulated Turtlebot. 


## ros 
    roscore

    rosbag play -l 'bag file'

    roslaunch hector_slam_launch tutorial.launch 


## Activate the Serial Port for RPLidar

    https://medium.com/robotics-with-ros/installing-the-rplidar-lidar-sensor-on-the-raspberry-pi-32047cde9588

    http://wiki.ros.org/rplidar

First, check the location of the RPlidar serial port, use the command

    ls -l /dev|grep ttyUSB

## Depanding on usb port
    sudo chmod 666 /dev/ttyUSB0 

    roslaunch rplidar_ros view_rplidar.launch

## check IP Address
    
    ifconfig

    export ROS_MASTER_URI=http://172.19.114.96:11311
    
    export ROS_HOSTNAME=172.19.114.96



## Open simulation

    roslaunch turtlebot3_gazebo test_world.launch

    roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch

    roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
