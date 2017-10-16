# Application Descriptions

The robot repertoire consists of a set of separate activities or applications, each independent of the others. Each activity requires a separate build
on the development virtual machine, and a separate SD card image for the Raspberry Pi on the robot. The applications are denoted by separate "roslaunch"
files that co-ordinate underlying packages. Packages are separately developed libraries for control of specific ROS nodes or devices.
that are re-usable 

The android assistant, on the other hand, contains control and monitoring code for all applications. 

### 01-HardwareCheck
This activity attempts to excercize all of the devices on the robot.

