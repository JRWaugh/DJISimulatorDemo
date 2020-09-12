# DJI Drone with Robot Operating System Demo
This project demonstrates how the Robot Operating System can be used with DJI's Mobile Software Development Kit (Android) to enhance its capabilities. This project
was implemented using ROS Kinetic.

# Prerequisites
## Install Ubuntu 16.04
ROS Kinetic requires Ubuntu 16.04. NOTE: Windows Subsystem Linux 2 will NOT work. Windows Subsystem Linux (the first) was not tested.

## Install ROS Kinetic On Your Ubuntu Machine
The ROS Wiki has a good tutorial for installing ROS Kinetic [here](http://wiki.ros.org/kinetic/Installation/Ubuntu). This demo only requires the Desktop install, 
but it will be useful to install Desktop-Full if you plan to use ROS in the future.

## Install ROS Joy Node and Dualshock 4 Drivers
NOTE: If python (2.7 or 3.3+) and pip (or pip3) are not already installed, they must be installed first

```shell
sudo apt-get install ros-kinetic-joy
sudo pip install ds4drv
```

## OPTIONAL: Install OpenVSLAM
Note that this installation could take some time as you will be compiling a lot of source code.

Follow the Linux install instructions [here](https://openvslam.readthedocs.io/en/master/installation.html). This project used PangolinViewer, but either viewer
should be fine. Then follow the ROS Package install guide [here](https://openvslam.readthedocs.io/en/master/ros_package.html). You can test that the package is 
working using the examples at the bottom of the page, if desired.

# Using ROS
## ROS Joy Node
If roscore is not already running, open a new Terminal and enter
```shell
roscore
```

In a **new terminal**, start the DS4 Driver (ds4drv) that you installed earlier
```shell
sudo ds4drv
```

In a **new terminal**, start publishing joy data through your roscore server
```shell
rosrun joy joy_node
```

In a **new terminal**, you can verify that joy data is flowing through the ROS network by entering the following
```shell
rostopic echo joy
```

That's it! So long as the mobile app is connected to the computer running roscore, it will subscribe to these messages.
