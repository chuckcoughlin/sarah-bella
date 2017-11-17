## Application Descriptions

The robot repertoire consists of a set of separate activities or applications, each independent of the others. Each activity requires a separate build
on the development virtual machine, and a transfer bundle for installation on the robot's Raspberry Pi. The applications are denoted by separate "roslaunch"
files that co-ordinate underlying packages. Packages are separately developed, reusable libraries for control of specific ROS nodes or devices.

The android assistant contains control and monitoring code for all applications.

******************************************************
### 01 - SystemCheck
This activity is designed to exercise all of the devices on the robot.

**topic:**
```
/sb_system_parameters
      String hostname
      String ip_address
```

#### ----------------------- notepad -------------------------

**node:** sb_subscribe_system_parameters <br/>

#### ---------------------- robot  --------------------------

**package:** ```catkin_create_pkg system_check rospy turtlebot3_msgs    turtlebot3_navigation```<br/>
**node:** sb_publish_system_parameters <br/>

******************************************************
