## sarah-bella
The *sarah-bella* (multi-brained) project is a collection of applications based on a Turtlebot3 robot and Android control application. This repository contains both ROS code for the robot and
Android code for a tablet controller. It also includes documentation outlining the toolchain and its setup.

The repository code is divided into 2 sections:

1) robot: Linux, C++ and Python ROS code for running the robot. We are using the ROS Kinetic distribution. The repository is shared between a Linux development system and the robot's Raspberry Pi control board. Once the new application is checked out on the development system, it is loaded, re-compiled and installed on the Pi. On a re-boot the robot runs autonomously.

2) android: An Android tablet is used as the robot user interface, communicating via wi-fi and ROS messaging. The android application
 "SBAssistant" makes use of ROSJava. The device is assumed to be Android 7.0 or higher. The single application may be used to execute any of the robot packages.

 For further details see [applications](http://github.com/chuckcoughlin/sarah-bella/tree/master/docs/applications.md),[toolchain](https://github.com/chuckcoughlin/sarah-bella/tree/master/docs/toolchain.md) and [extensions](https://github.com/chuckcoughlin/sarah-bella/tree/master/docs/extensions.md).
