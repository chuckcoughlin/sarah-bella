# Application Descriptions

The robot repertoire consists of a set of separate activities or applications, each independent of the others. Each activity requires a separate build
on the development virtual machine, and a transfer bundle for installation on the robot's Raspberry Pi. The applications are denoted by separate "roslaunch"
files that co-ordinate underlying packages. Packages are separately developed, reusable libraries for control of specific ROS nodes or devices.

The android assistant contains control and monitoring code for all applications. 

### 01 - SystemCheck
This activity attempts to exercise all of the devices on the robot.
To the robot, the node is a service.

To the notepad, the activity is collection of requests.

sb_system_check {
    String hostname
    String ip_address
}
