# DJI Drone with Robot Operating System Demo
This project demonstrates how the Robot Operating System can be used with DJI's Mobile Software Development Kit (Android) to enhance its capabilities. This project
was implemented using ROS Kinetic, although newer versions should also work. 

When the app is opened, it will wait for the phone to connect to a DJI remote controller (by USB) and for the remote controller to connect to a drone. Once everything is connected, the OPEN button will become enabled. Clicking the button will take you to the ROS connection screen where you can put the IP of the computer running roscore. If connecting via the internet, you will need to forward port 11311 to the computer running roscore.

![No Drone Connected](/images/homescreen1.jpg) ![Drone Connected](/images/homescreen2.jpg) ![ROS Connect Screen](/images/rosscreen.jpg)

## Prerequisites
### Install Ubuntu 16.04
ROS Kinetic requires Ubuntu 16.04. NOTE: Windows Subsystem Linux 2 will NOT work. Windows Subsystem Linux (the first) was not tested.

### Install ROS Kinetic On Your Ubuntu Machine
The ROS Wiki has a good tutorial for installing ROS Kinetic [here](http://wiki.ros.org/kinetic/Installation/Ubuntu). This demo only requires the Desktop install, 
but it will be useful to install Desktop-Full if you plan to use ROS in the future.

### Install ROS Joy Node and Dualshock 4 Drivers

```shell
sudo apt-get install ros-kinetic-joy
sudo pip install ds4drv
```

### OPTIONAL: Install OpenVSLAM and Prepare Drone Camera for Computer Vision
Note that this installation could take some time as you will be compiling a lot of source code.

Follow the Linux install instructions [here](https://openvslam.readthedocs.io/en/master/installation.html). This project used PangolinViewer, but either viewer
should be fine. Then follow the ROS Package install guide [here](https://openvslam.readthedocs.io/en/master/ros_package.html). You can test that the package is 
working using the examples at the bottom of the page, if desired.


This project has only included the camera configuration file necessary for computer vision for the Mavic Pro monocular camera. If you are using a different drone with a different camera, you will need to generate your own .yaml file. Instructions to do so can be found [here](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration). Note that the app is publishing **compressed** images, but this software is subscribing only to **raw** images. To get around this, the camera data from the app will need to be republished in the following way:

```shell
rosrun image_transport republish compressed in:=/camera/image out:=/camera/image
```

Obviously this will not restore the data lost from the initial compression, but that will not matter.

## Using ROS
### Subscribing to Images from App
If roscore is not already running, open a new terminal and enter
```shell
roscore
```

In a **new terminal**, subscribe to the /camera/image/compressed ROS topic with rqt_image_view
```shell
rqt_image_view /compressed/image/compressed
```

Alternatively, you can simply open rqt_image_view and select the topic from the dropdown box in the top left corner.
![Using rqt_image_view](/images/rqt_image_view.png) 

If everything worked, you should see images coming in from the app. This will effectively be a low latency, near-30fps video feed, although uneven frame pacing might make the video seem a little stuttery.

### Publishing Joystick Data to App
If roscore is not already running, open a new terminal and enter
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
