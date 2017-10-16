# sarah-bella
The sarah-bella project is a collection of applications involving a TurtleBot3 robot and an Android control application. The project contains both ROS code for the robot and
Android code for a tablet controller. It also includes documentation outlining the toolchain and its setup.
The repository contains code divided into 2 sections:
1) robot: Linux, C++ ROS code for running the robot. We are using the ROS Kinetic distribution. The result of a build is flashed onto an SD card that is inserted into
          the robot's Raspberry Pi control board.
2) android: An Android tablet is used as the robot user interface, communicating to the robot via wi-fi. The android application "SBAssistant"
         makes use of ROSJava. The device is assumed to be Android 7.0 or higher (version 24).
