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
  * [System Check](#systemcheck)
  * [Headlamp](#headlamp)

*********************************************************
### a - Discovery <a id="discovery"></a>
![SB Assistant](/images/sb-discovery.png)
```                        SB Assistant - Discovery Page ```</br>
[toc](#table-of-contents)

The **discovery** panel allows connection to a single running robot based on a MasterURI configured in the *Settings*. It reads values published from the robot through its ParameterServer to validate the device.  

A SQLite database on the tablet contains a list of applications expected to be supported by the robot. This panel provides an opportunity to select and start applications from this list. The robot is configured for the desired application via a *ssh* connection where a script is run to set the application and restart *rosCore*. This operation requires that a hostname, *sudoer* username and password for the robot be entered in the *Settings* panel on the tablet.

******************************************************
### b - Logging <a id="logging"></a>
[toc](#table-of-contents)

The **logging** panel is a general-purpose feature that supports all applications.
It displays the most recent 100 log messages from the robot. All of the standard
applications have been scrubbed to incorporate standard ROS log messages.

******************************************************
### 01 - System Check <a id="systemcheck"></a>
![SB Assistant](/images/sb-system.png)
```                        SB Assistant - System Page ```</br>
[toc](#table-of-contents)

The **system** panel is designed to read system parameters and interactively exercise devices on the robot. It displays system CPU and memory performance metrics; it also sports a table that displays status of GPIO outputs and provides control actions for GPIO inputs.

******************************************************
### 02 - Headlamp <a id="headlamp"></a>
The **headlamp** application has no separate control panel. Instead, the *system* panel is used. Simply press the GPIO channel labelled ```HEADLAMP``` to turn the
lamp on and off.

******************************************************
### 03 - Follow <a id="follow"></a>
With the *follow* application running, walk in front of the TurtleBot3. Then, slowly walk away from the TurtleBot. The robot should move forward. Moving close to the TurtleBot will cause it to back away. Moving slowly to the left or right will cause the TurtleBot to turn. To stop the robot from following, walk quickly away from the robot.

******************************************************
### 04 - Auto-park <a id="follow"></a>
When this application is running, the robot will search for a white tag that marks its intended parking space.
