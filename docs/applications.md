## Application Descriptions

The robot repertoire consists of a set of separate activities or applications, each independent of the others. Each activity requires a separate build
on the development virtual machine, and a transfer bundle for installation on the robot's Raspberry Pi. The applications are denoted by separate "roslaunch"
files that co-ordinate underlying packages. Packages are separately developed, reusable libraries for control of specific ROS nodes or devices.

The android assistant contains control and monitoring code for all applications.

#### 01 - SystemCheck
This activity is designed to exercise all of the devices on the robot.

**topic:**
```
sb_system_check:
    String hostname
    String ip_address
```
##### Android
**node:** sb_get_system_parameters

##### robot

**package:** ```$: catkin_create_pkg system_check rospy turtlebot3```
**node:** sb_get_system_parameters
