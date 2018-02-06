# Extensions to Core Configuration

This document summarizes hardware, software and configuration extensions to support specific applications on the Sarah-Bella application.
Refer to [applications](http://github.com/chuckcoughlin/sarah-bella/tree/master/docs/applications.md) for general descriptions.

We have drawn heavily from common message definitions that are available from the repository: https://github.com/ros/common_msgs. We pick and choose from
the collection as needed.

***************************************************************
## Table of Contents <a id="table-of-contents"></a>
  * [Discovery](#discovery)
  * [System Check](#systemcheck)

*********************************************************
### 00 - Discovery <a id="discovery"></a>
Discovery makes use of the following robot parameters. These are
 expected to be defined/published in every robot launch script:

* robot/name  - the name of the robot
* robot/type  - the robot type (e.g. turtlebot3)
* robot/platform - the hardware platform (e.g. Linux, RaspberryPi)
* robot/application - the name of the application currently running on the robot

Application selection and restart of the robot depend on a *ssh* login as super-user and execution of the following commands:
```
  ~/robotics/robot/bin/set_ros_application <app_name>
  /etc/init.d/ros restart
```


### 01 - System Check <a id="systemcheck"></a>
*psutil* is used to obtain system performance metrics to support the sb_system/System message. This package is installed by default on the Linux virtual machine, but not on the
Raspberry Pi. For a complete description see: https://psutil.readthedocs.io/en/latest. To install on the robot:
```
   sudo apt-get install build-essential python-dev python-pip
   sudo pip install psutil
```
From the common messages we use  sensor_msgs/BatteryState. For a full description, see
 http://wiki.ros.org/sensor_msgs.


The following messages are exchanged:

```
System
      String hostname
      String ip_address
      Float32 cpu_percent
      Float32 memory_percent_used
      Uint32 free_memory_bytes
      Float32 swap_memory_percent_used
      Float32 disk_percent_used
      Uint32 packets_sent
      Uint32 packets_received
      Uint32 in_packets_dropped
      Uint32 out_packets_dropped
```

 To build the package:
```
    catkin_create_pkg system_check rospy turtlebot3_msgs  turtlebot3_navigation
```
##### ----------------------- tablet -------------------------

**node:** sb_subscribe_system (sb_system/System)<br/>

#####---------------------- robot  --------------------------
**node:** sb_publish_system (sb_system/System)<br/>

### 02 - Follow <a id="follow"></a>
 See turtlebot follower at: https://github.com/turtlebot/turtlebot_apps.git. Modifications were made to support turtlebot3_msg dependencies.

 The follow applications will cause the TurtleBot3 to look for objects in a window in front of it. And it will seek to keep the centroid of the observed objects directly in front of it and a fixed distance away. If the centroid of the object is too far away it will drive forward, too close backward, and if offset to the side it will turn toward the centroid.

##### 04 - Park <a id="park"></a>
See Turtlebot3 Automatic Parking at: https://github.com/ROBOTIS-GIT/turtlebot3_applications
