## Application Descriptions

The robot repertoire consists of a set of separate activities or applications, each largely independent of the others. In general, activities require separate ROS packages
on the development virtual machine for subsequent transfer and compilation on the robot's Raspberry Pi. Packages are separately developed, reusable libraries for control of specific ROS nodes or devices. ROS messaging, the communication mechanism between packages and for the Android control application, is defined below for each application. The robot initiates all packages on startup.

![SB Assistant](/images/sb-cover.png)
````                        SB Assistant - Cover Page ````

The android assistant contains command and monitoring code for all applications installed on the robot.

******************************************************
### 01 - SystemCheck
This activity is designed to read system parameters and exercise all devices on the robot.

**topic:**
```
/sb_system
      String hostname
      String ip_address
```

#### ----------------------- notepad -------------------------

**node:** sb_subscribe_system <br/>

#### ---------------------- robot  --------------------------

**package:** ```catkin_create_pkg system_check rospy turtlebot3_msgs    turtlebot3_navigation```<br/>
**node:** sb_publish_system <br/>

******************************************************
