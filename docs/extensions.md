# Extensions to Core Configuration

This document summarizes hardware, software and configuration extensions to support specific applications on the Sarah-Bella application.
Refer to [applications](http://github.com/chuckcoughlin/sarah-bella/tree/master/docs/applications.md) for general descriptions.

We have drawn heavily from common message definitions that are available from the repository: https://github.com/ros/common_msgs. We pick and choose from
the collection as needed.

***************************************************************
## Table of Contents <a id="table-of-contents"></a>
  * [Discovery](#discovery)
  * [System Check](#systemcheck)
  * [Headlamp](#headlamp)

*********************************************************
### 00 - Discovery <a id="discovery"></a>
Discovery makes use of the following robot parameters. These are
 expected to be defined/published in every robot launch script:

* robot/name  - the name of the robot
* robot/type  - the robot type (e.g. turtlebot3)
* robot/platform - the hardware platform (e.g. Linux, RaspberryPi)
* robot/application - the name of the application currently running on the robot

Application selection and restart of the robot depends on a *ssh* login as super-user and execution of the following commands:
```
  ~/robotics/robot/bin/set_ros_application <app_name>
  /etc/init.d/ros restart
```


### 01 - System Check <a id="systemcheck"></a>

*psutil* is used on the robot to obtain system performance metrics in support of the custom *System* message type. This package is installed by default on the Linux virtual machine, but not on the
Raspberry Pi. For a complete description see [here](https://psutil.readthedocs.io/en/latest). To install on the robot:
```
   sudo apt-get install build-essential python-dev python-pip
   sudo pip install psutil
```

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
The *SensorState* message from turtlebot3_msgs is built into the Turtlebot3 firmware (updated 1/17/2018). For a full description, see [here]( http://docs.ros.org/hydro/api/kobuki_msgs/html/msg/SensorState.html). The tablet subscribes to a throttled stream. Not all properties are displayed.

```
SensorState
  Header header
  Uint16 time_stamp    # milliseconds from start (rollover at 65536)
  Uint8  bumper        # bumper state
  Uint8  cliff         # cliff sensor state
  Uint16 left_encoder  # accumulated ticks left wheel (max. 65535)
  Uint16 right_encoder # accumulated ticks right wheel (max. 65535)
  Uint8  battery       # battery voltage in 0.1V (ex. 16.1V -> 161)
```

The *GPIOState* message is a custom message that provides the current state and configuration of the GPIO header.
It consists of a list of *GPIOPin* messages. Normally the pin messages represent only
those output channels that have changed. However, the the ROS parameter
```/gpio_msgs/publish_all``` is set to "True", then the message will contain the
entire list of 40 pins. This parameter is handled like a semaphore, and is immediately
reset to "False" by the robot. The GPIOState message is designed to have a response
time on the order of 0.1 seconds.

Each GPIOPin sub-message consists of:

```
GPIOPin
  String name
  Uint32 channel
  Bool value
  String mode              # IN, OUT, GND, PWR
```
The panel subscribes to the following topics:
 * /gpio_msgs/GPIOState
 * /sb_system/System
 * /sensor_state_throttle/SensorState

The panel provides control of GPIO values with the action:
 * /gpio_msgs/GPIOPin

The initial construction of the package files was accomplished using:
```
    catkin_create_pkg system_check rospy turtlebot3_msgs  turtlebot3_navigation
```

##### ----------------------- tablet --------------------------<br/>
**node:** sb_subscribe_battery_state(sensor_msgs/BatteryState)<br/>
**node:** sb_subscribe_gpio (gpio_msgs/GPIOState)<br/>
**node:** sb_subscribe_system (sb_system/System)<br/>

##### ---------------------- robot  --------------------------<br/>
**node:** sb_publish_sensor_state (turtlebot3_msgs/SensorState)<br/>
**node:** sb_publish_gpio_state (gpio_msgs/GPIOState)<br/>
**node:** sb_publish_system (sb_system/System)<br/>
**node:** sb_serve_gpio_set (GPIOSetRequest/GPIOSetResponse)<br/>

### 02 - Headlamp <a id="headlamp"></a>
 The current limit for the GPIO board is about 2ma, thus a lamp must controlled through
 a relay or some other isolating mechanism.

 ### 03- Follow <a id="follow"></a>
 The *follower* application is one of the ROBOTIS demonstrations. I chose the version from: https://github.com/NVIDIA-Jetson/turtlebot3/tree/master/turtlebot_apps/turtlebot_follower. I modified references from *turtlebot_msgs* to *turtlebot3_msgs*.

 The *follow* application will cause the TurtleBot3 to look for objects in a window 50cm in front of it. And it will seek to keep the centroid of the observed objects directly in front of it and a fixed distance away. If the centroid of the object is too far away it will drive forward, too close backward, and if offset to the side it will turn toward the centroid.

 https://github.com/jjones646/turtlebot-follower.git Class project outline. Python.

 https://github.com/NVIDIA-Jetson/turtlebot3/blob/master/turtlebot_apps/turtlebot_follower/src/follower.cpp Jetson-NVIDIA. cpp, turtlebot3, switch.py. rapp

 https://github.com/ROBOTIS-GIT/turtlebot3_applications. Official demo. python. follow_filter


##### 04 - Park <a id="park"></a>
See Turtlebot3 Automatic Parking at: https://github.com/ROBOTIS-GIT/turtlebot3_applications
