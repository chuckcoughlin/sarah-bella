## Application Descriptions

The robot repertoire consists of set of applications constructed and debugged
on a Linux virtual machine.  These are subsequently transferred to and compiled on the Turtlebot3. The robot launch configuration determines which application will execute.

The sections below contain general descriptions
of the applications with a brief view of applicable control screens on the Android tablet.
ROS nodes and messaging details as well as descriptions of hardware modifications are described in  [extensions](https://github.com/chuckcoughlin/sarah-bella/tree/master/docs/extensions.md).  

![SB Assistant](/images/sb-cover.png)
```                        SB Assistant - Cover Page ```

The Android assistant contains command and monitoring code for all applications installed on the robot. It is a single activity application, featuring sliding panels (Fragments), one for each application.
***************************************************************
## Table of Contents <a id="table-of-contents"></a>
  * [Discovery](#discovery)
  * [Logging](#logging)
  * [Lidar](#Lidar)
  * [System Check](#systemcheck)
  * [Headlamp](#headlamp)
  * [Teleop](#teleop)

*********************************************************
### a - Discovery <a id="discovery"></a>
![SB Assistant](/images/sb-discovery.png)
```                        SB Assistant - Discovery Page ```</br>
[toc](#table-of-contents)

The **discovery** panel allows connection to a single running robot based on a MasterURI configured in the *Settings*. It reads values published from the robot through its ParameterServer to validate the device.  

The CONNECT button initiates a network connection (Bluetooth or Wifi). The triangle indicator at the top of the screen shows whether or not a ROS messaging connection has been made.

Selecting an application is accomplished by clicking the applicable line in
the displayed list.  This operation connects to the robot and configures it for the desired operation. This interaction requires that hostname, *sudoer* username
and password for the robot have been entered in the *Settings* panel on the tablet.

The ON/OFF button that appears in the application row starts and stops
subscriptions on the
tablet. It has no effect on the robot itself.

******************************************************
### b - Logging <a id="logging"></a>
![SB Assistant](/images/sb-logging.png)
```                        SB Assistant - Log Message Page ```</br>
[toc](#table-of-contents)

The **logging** panel is a general-purpose feature that supports all applications.
It displays the most recent 100 log messages from the robot, collecting messages
whether or not the panel is showing. Clicking on a message displays more detail.
All of the standard
applications have been scrubbed to incorporate standard ROS log messages.

******************************************************
### c - Lidar <a id="lidar"></a>
![SB Assistant](/images/sb-lidar.png)
```                        SB Assistant - LIDAR Display Page ```</br>
[toc](#table-of-contents)

The **lidar** panel supports several applications that make use of the Lidar
scanner. The display is scalable by the slider to the right. Touching the
screen centers the image on the touch point.

******************************************************
### 01 - System Check <a id="systemcheck"></a>
![SB Assistant](/images/sb-system.png)
```                        SB Assistant - System Page ```</br>
[toc](#table-of-contents)

The **system** panel is designed to read system parameters and interactively exercise devices on the robot. It displays system CPU and memory performance metrics; it also shows a table that displays configuration and status of GPIO pins. The gray dots refer to inputs, the red/green to outputs.  Selecting an output changes its state.

******************************************************
### 02 - Headlamp <a id="headlamp"></a>
![SB Assistant](/images/sarah-bella-porch.png)
```                        Sarah-Bella Porch Structure ```</br>
![SB Assistant](/images/sarah-bella-mezzanine.png)
```                        Sarah-Bella Mezzanine Structure ```</br>
[toc](#table-of-contents)
The **headlamp** application has no separate control panel. Instead, the *system* panel is used. Simply press the GPIO channel labelled ```HEADLAMP``` to turn the
lamp on and off.

The photos show the "porch" add-on structure used to hold the headlamp and relay module. The "mezzanine" holds a breadboard and auxiliary battery.

******************************************************
### 03 - Teleop <a id="teleop"></a>
![SB Assistant](/images/sb-teleop.png)
```                        SB Assistant - Teleop Control ```</br>
[toc](#table-of-contents)

The **teleop** application provides remote-control of the robot via the virtual joystick. Pressing further from the center results in increasing speed in the direction pressed. Lifting the finger, stops the robot.

******************************************************
### 04 - Follow <a id="follow"></a>
With the *follow* application running, walk in front of the TurtleBot3. Then, slowly walk away from the TurtleBot. The robot should move forward. Moving close to the TurtleBot will cause it to back away. Moving slowly to the left or right will cause the TurtleBot to turn. To stop the robot from following, walk quickly away from the robot.

******************************************************
### 05 - Auto-park <a id="follow"></a>
When this application is running, the robot will search for a white tag that marks its intended parking space.
