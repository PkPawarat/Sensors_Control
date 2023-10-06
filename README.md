# Sensors_Control
Subject 41014 Sensors and Control for Mechatronics Systems 


## Matlab
 a
    matlab




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