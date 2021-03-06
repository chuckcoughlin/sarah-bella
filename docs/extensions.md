# Extensions to Core Configuration

This document summarizes hardware, software and configuration extensions to support specific operations within the Sarah-Bella application.
Refer to [applications](http://github.com/chuckcoughlin/sarah-bella/tree/master/docs/applications.md) for general usage descriptions.

In addition to our own custom messages, we have drawn heavily from common message definitions that are available from the repository: https://github.com/ros/common_msgs. We pick and choose from
the collection as needed. The publish/subscribe topics that are listed below
do not include those topics that are included in the robot by default.

***************************************************************
## Table of Contents <a id="table-of-contents"></a>
  * [Discovery](#discovery)
  * [Logging](#logging)
  * [Lidar](#lidar)
  * [System Check](#systemcheck)
  * [Headlamp](#headlamp)
  * [Teleop](#teleop)
  * [Follow](#follow)
  * [Park](#park)

*********************************************************
### a - Discovery <a id="discovery"></a>
Discovery makes use of the following robot parameters. These are
 expected to be defined/published in every robot launch script:

* robot/name  - the name of the robot
* robot/type  - the robot type (e.g. turtlebot3)
* robot/platform - the hardware platform (e.g. Linux, RaspberryPi)
* robot/application - the name of the application currently running on the robot

A SQLite database on the tablet contains a list of applications expected to be supported by the robot. The **discovery** panel provides an opportunity to select and start applications from this list.

Application selection and restart of the robot depends on a *ssh* login as super-user and execution of the following commands:
```
  ~/robotics/robot/bin/set_ros_application <app_name>
  sudo ~/robotics/robot/bin/restart_ros
```
These commands set the application and restart *rosCore*.

*********************************************************
### b - Logging <a id="logging"></a>
The logging panel subscribes to the ROS topic ```/rosout``` no matter what application is running. This is an
aggregated stream of all log messages generated by the robot.

The log message is contained in *rosgraph_msgs*.
```
Log
      Header header
      byte level            # severity level (1-DEBUG,2-INFO,4-WARN,8-ERROR)
      string name           # name of the node
      string msg            # text message
      string file           # name of originating source file
      string function       # originating function
      uint32 line           # line number
      string[] topics       # topics published by this node
```
### c - Lidar <a id="lidar"></a>
The lidar panel displays output from the Lidar device. The *VisualizationView* is based on code found [here](https://github.com/rosjava/android_core/tree/kinetic/android_15/src/org/ros/android/view/visualization).

```
LaserScan
# Single scan from a planar laser range-finder
# Frame = frame_id, angles are measured around
# the positive Z axis (counterclockwise, if Z is up)
# with zero angle being forward along the x axis
   Header header            # timestamp is the acquisition
                            # time of the first ray

   float32 angle_min        # 0. - start angle of the scan [radians]
   float32 angle_max        # 2π - end angle of the scan [radians]
   float32 angle_increment  # 0.12 - angular distance between
                            # measurements [radians]

   float32 time_increment   # time between measurements [seconds]
                            # if scanner is moving, this
                            # will be used in interpolating position
                            # of 3d points
   float32 scan_time        # time between scans [seconds]
   float32 range_min        # 0.  - min range value [m]
   float32 range_max        # 3.5 - max range value [m]

   float32[] ranges         # range data [m] (360 values)
                            # Note: values < range_min or >
                            # range_max should be discarded)
   float32[] intensities    # intensity data [device-specific units].
```

The tfMessage contains nested classes from *geometry_msgs*. In the list below we've used indentation to show the expanded definitions.
```
tfMessage
   TransformStamped[] transforms
      Header header
      string child_frame_id
      Transform transform
         Vector3 translation
            float64 x
            float64 y
            float64 z
         Quaternion rotation
            float64 x
            float64 y
            float64 z
            float64 w
```
##### ----------------------- tablet --------------------------<br/>
**subscribe:** /tf_throttle (tfMessage)<br/>
**subscribe:** /scan_throttle (LaserScan)<br/>

### 01 - System Check <a id="systemcheck"></a>

*psutil* is used on the robot to obtain system performance metrics in support of the custom *System* message type. This package is installed by default on the Linux virtual machine, but not on the
Raspberry Pi. For a complete description see [here](https://psutil.readthedocs.io/en/latest). To install on the robot:
```
   sudo apt-get install build-essential python-dev python-pip
   sudo pip install psutil
```

```
System
      string hostname
      string ip_address
      float32 cpu_percent
      float32 memory_percent_used
      uint32 free_memory_bytes
      float32 swap_memory_percent_used
      float32 disk_percent_used
      uint32 packets_sent
      uint32 packets_received
      uint32 in_packets_dropped
      uint32 out_packets_dropped
```
The *SensorState* message from turtlebot3_msgs is built into the Turtlebot3 firmware (updated 1/17/2018). For a full description, see [here]( http://docs.ros.org/hydro/api/kobuki_msgs/html/msg/SensorState.html). The tablet subscribes to a throttled stream. Not all properties are displayed.

```
SensorState
  Header header
  uint16 time_stamp    # milliseconds from start (rollover at 65536)
  uint8  bumper        # bumper state
  uint8  cliff         # cliff sensor state
  uint16 left_encoder  # accumulated ticks left wheel (max. 65535)
  uint16 right_encoder # accumulated ticks right wheel (max. 65535)
  uint8  battery       # battery voltage in 0.1V (ex. 16.1V -> 161)
```

The *GPIOState* message is a custom message that provides the current state and configuration of the GPIO header.
It consists of a list of *GPIOPin* messages. Normally the pin messages represent only
those input/output channels that have changed state. However, on every 100th cycle, the robot will return messages for the
entire list of 40 pins. The GPIOState topic is designed to have a response
time on the order of 0.1 seconds.

Each *GPIOPin* sub-message consists of:

```
GPIOPin
  string name
  uint32 channel
  bool value
  string mode              # IN, OUT, GND, PWR
```
The initial construction of the package files was accomplished using:
```
    catkin_create_pkg system_check rospy turtlebot3_msgs  turtlebot3_navigation
```

##### ----------------------- tablet --------------------------<br/>
**subscribe:** /sensor_state_throttle (SensorState)<br/>
**subscribe:** /gpio_msgs (GPIOState)<br/>
**subscribe:** /sb_system (System)<br/>
**action:**  /sb_serve_gpio_set (GPIOSet)<br/>

##### ---------------------- robot  --------------------------<br/>
**publish:** /gpio_msgs (GPIOState)<br/>
**publish:** /sb_system (System)<br/>
**service:** /sb_serve_gpio_set (GPIOSetRequest/GPIOSetResponse)<br/>

### 02 - Headlamp <a id="headlamp"></a>
 The current limit for the GPIO board is about 2ma, thus a lamp must controlled through
 a relay or some other isolating mechanism.

 Parts list:</br>
 ```
   lamp - 1157 6V LED motorcycle tail light, 51 mA
   battery - LIPO 11.1V 1000mA LB-010 (ROBOTIS)
   breadboard - 60x170 mm, solder-less
   relay module - Ivoldar Labs 5V
 ```
 We've extended the TurtleBot3 "waffle" structures to add space for mounting the
 headlamp and battery. The figures below are rendered from the CAD files. 3D-print sources for these structures are contained in
 the source code repository.

 ![Porch (lamp holder)](/images/porch-cad.png)
 ````Porch````
 ![Mezzanine (auxiliary battery holder)](/images/mezzanine-cad.png)
 ````Mezzanine````

![Headlamp circuit](/images/headlamp_circuit.svg).

 ### 03- Teleop <a id="teleop"></a>
The base code for the joystick view widget came from [here](https://github.com/rosjava/android_core/tree/kinetic/android_15/src/org/ros/android/view). We have modified the control so that "up" always correlates to the
front of the robot. The joystick control may be augmented with voice commands. Additionally there is a fail-safe mechanism that prevents
the robot from crashing into obstacles, no matter what the user does.


On the robot we use a service for control. It simply transforms requests into ```/cmd_vel (Twist)``` messages. The service is required because, with ROS, remote publishers must be established before local (on robot) subscribers. In our design the application is started from the tablet.

The robot publishes custom *ObstacleDistance* messages. The distance returned is the minimum distance to an object detected by *Lidar* within a rectangular area in front of the robot. The
width is set by the parameter ```/robot/width```.

```
ObstacleDistance
  float4 distance
```

##### ----------------------- tablet --------------------------<br/>
**action:** /sb_serve_behavior_command (Behavior)<br/>
**action:** /sb_serve_twist_command (TwistCommandRequest)<br/>
**subscribe:**  /sb_teleop (ObstacleDistance)<br/>
**subscribe:**  /sb_teleop_status (TeleopStatus)<br/>

##### ----------------------- robot --------------------------<br/>
**publish:**  /cml_vel (Twist)<br/>
**publish:**  /sb_obstacle_distance (ObstacleDistance)<br/>
**publish:**  /sb_teleop_status (TeleopStatus)<br/>

### 04- Follow <a id="follow"></a>
The *follower* application is one of the ROBOTIS demonstrations. I chose the version from: https://github.com/pirobot/ros-by-example/blob/master/rbx_vol_1/rbx1_apps/nodes/follower.py as my starting point, making modifications to fit into my application framework.

The *follow* application will cause the TurtleBot3 to look for objects in a window 50cm in front of it. And it will seek to keep the centroid of the observed objects directly in front of it and a fixed distance away. If the centroid of the object is too far away it will drive forward, too close backward, and if offset to the side it will turn toward the centroid.

##### ----------------------- tablet --------------------------<br/>
 **action:** /sb_serve_behavior_command (Behavior)<br/>
 **subscribe:**  /sb_teleop_status (TeleopStatus)<br/>

##### ----------------------- robot --------------------------<br/>
**service:**  /cml_vel (Twist)<br/>
**publish:**  /sb_teleop_status (TeleopStatus)<br/>


### 05 - Park <a id="park"></a>
The ``automatic park`` application uses lidar readings to locate two towers which must be the closest objects in its scan range. Once located the robot uses odometry to trace a rectangular pattern in front of the towers, then back into a "parking space" halfway between the towers.

The figure below shows the layout, labels and some of the trig formulae used in the code.

![SB Assistant](/images/sb-park-geometry.png)
```                        SB Assistant - Parking Geometry ```</br>


##### ----------------------- tablet --------------------------<br/>
**action:** /sb_serve_behavior_command (Behavior)<br/>
**subscribe:**  /sb_teleop_status (TeleopStatus)<br/>

##### ----------------------- robot --------------------------<br/>
**subscribe:**  /scan_throttle (LasarScan)<br/>
**subscribe:**  /odom (Odometry)<br/>
**service:**  /cml_vel (Twist)<br/>
**publish:**  /sb_teleop_status (TeleopStatus)<br/>


### 06 - Come <a id="come"></a>

The 3D printed ears are derived from [OpenBiaural](https://github.com/CarlosGS/OpenBinaural) by Carlos Garcia Saura modified to accept the microphone and mount on our mezzanine.
