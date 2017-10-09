# sarah-bella
sarah-bella is a control application for a TurtleBot3 robot. In addition to documentation regarding the toolchain and its setup,
the repository contains code divided into 2 sections:
1) robot: Linux, C++ ROS code for actually running the robot. This code is flashed onto an SD card that is inserted into
          the Raspberry Pi control board.
2) android: An Android tablet is used as the robot user interface, communicating to the robot via wi-fi. The android application "SBAssistant"
         makes use of ROSJava. The device is assumed to be Android 7.0 or higher (version 24).
