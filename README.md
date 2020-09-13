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

### OPTIONAL: Install OpenVSLAM and Calibrate Drone Camera
Note that this installation could take some time as you will be compiling a lot of source code.

Follow the Linux install instructions [here](https://openvslam.readthedocs.io/en/master/installation.html), and the ROS package install guide [here](https://openvslam.readthedocs.io/en/master/ros_package.html).

#### If you plan to use this project with the Mavic Pro, you can skip this next step as I have included the file necessary for the Mavic Pro's camera.

To use OpenVSLAM or any other computer vision, a .yaml file detailing the qualities of a camera must first be generated. Follow the instructions [here](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration) to install the software needed, as well as to find a checkerboard pattern that you can use. The grid is printed on a very large piece of paper in the guide, but it will work at any size (A4, for example).

Note that this software subscribes to **raw** images, but the app publishes **compressed** images. To get around this, the camera data from the app will need to be republished to the raw image topic. Obviously this will not restore the data lost from the initial compression, but that will not matter.
```shell
rosrun image_transport republish compressed in:=/camera/image out:=/camera/image
```

Once you have your checkerboard pattern and you are publishing raw images, open a **new terminal** and start the camera_calibration software. Replace the --square parameter with the size (in millimetres) of a square in your own checkerboard.
```shell
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.108 image:=/camera/image camera:=/camera --no-service-check
```

Lastly, to use the .yaml file with OpenVSLAM will you need to reorganise the data according to the way [this post](https://github.com/xdspacelab/openvslam/issues/110#issuecomment-530214545) describes. It will be annoying to have to do this by hand. Sorry!

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

**If you chose to install OpenVSLAM**, you can follow the subscriber instructions [here](https://openvslam.readthedocs.io/en/master/ros_package.html) to find how to subscribe to the images with OpenVSLAM. OpenVSLAM subscribes only to raw images, so be sure to republish the images from the app with the following first:
```shell
rosrun image_transport republish compressed in:=/camera/image out:=/camera/image_raw
```

### Publishing Joystick Data to App
If roscore is not already running, open a new terminal and enter
```shell
roscore
```

In a **new terminal**, start the DS4 Driver (ds4drv) that you installed earlier
```shell
sudo ds4drv
```
ds4drv will now search for devices and establish connections. To create a new connection, hold the SHARE and PS buttons until the terminal says "[info][bluetooth] Found device ...".

In a **new terminal**, start publishing joy data through your roscore server
```shell
rosrun joy joy_node
```

In a **new terminal**, you can verify that joy data is flowing through the ROS network by entering the following
```shell
rostopic echo joy
```

That's it! So long as the mobile app is connected to the computer running roscore, it will subscribe to these messages.
