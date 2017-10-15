# sarah-bella
sarah-bella is a project combining a TurtleBot3 robot and an Android control application. In addition to both ROS code for the robot and
Android code for a tablet controller, the project includes documentation regarding the toolchain and its setup.
The repository contains code divided into 2 sections:
1) robot: Linux, C++ ROS code for running the robot. We are using the ROS Lunar version. This code is flashed onto an SD card that is inserted into
          the Raspberry Pi control board.
2) android: An Android tablet is used as the robot user interface, communicating to the robot via wi-fi. The android application "SBAssistant"
         makes use of ROSJava. The device is assumed to be Android 7.0 or higher (version 24).
