## sarah-bella
The *sarah-bella* (multi-brained) project is a collection of applications designed for a Turtlebot3 robot and Android control application. This repository contains both ROS code for the robot and
Android code for a tablet controller. It also includes documentation outlining the toolchain and its setup.

The repository code is divided into 2 sections:

1) robot: Linux, C++ and Python ROS code for running the robot. We are using the ROS Kinetic distribution.
The repository is shared between a Linux development system and the robot's Raspberry Pi control board. New applications are coded,
debugged and added to the repository on the development system.  From there the application is loaded, re-compiled and installed on the Pi. On a re-boot the robot runs autonomously.

2) android: An Android tablet provides the robot user interface, communicating via wi-fi and ROS messaging. The Android application
 "SBAssistant" makes use of ROSJava. The device is assumed to be Android 8.0 or higher. This single application may be used to execute any of the robot packages.

 As of October, 2018 this project is under active development. In general, documentation precedes completed features. The following major features can be considered "done":
  * SBAssistant: android controller, panel navigation, robot Bluetooth or WiFi connection, select/change active robot application.
  * system: tablet displays robot status - CPU, memory, battery and GPIO configuration. Tablet gets, sets and subscribes to GPIO values.
  * logging: tablet displays ROS log messages generated on the robot.
  * lidar: tablet displays a scalable laser scan output.
  * follow: one of the options on the "teleop" page. Robot follows a slow walker.
  * park: one of the options on the "teleop" page. Robot executes a "parallel park" pattern positioning itself between two posts.
  * teleop: use a virtual joystick on the tablet to control the robot. Alternately, use voice commands in English or Russian. The robot will automatically detect
  obstacles and stop before hitting them.


 For further details see [applications](http://github.com/chuckcoughlin/sarah-bella/tree/master/docs/applications.md), [toolchain](https://github.com/chuckcoughlin/sarah-bella/tree/master/docs/toolchain.md) and [extensions](https://github.com/chuckcoughlin/sarah-bella/tree/master/docs/extensions.md).
