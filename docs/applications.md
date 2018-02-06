## Application Descriptions

The robot repertoire consists of set of applications constructed
on a Linux virtual machine running on the development system.  These are subsequently transferred to and compiled on the Turtlebot3. The robot launch configuration determines which application will execute.

The sections below contain general descriptions
of the applications with a brief view of applicable control screens on the Android tablet.
ROS nodes and messaging details as well as descriptions of hardware modifications are described in  [extensions](https://github.com/chuckcoughlin/sarah-bella/tree/master/docs/extensions.md).  

![SB Assistant](/images/sb-cover.png)
````                        SB Assistant - Cover Page ````

The android assistant contains command and monitoring code for all applications installed on the robot. It is a single activity application, featuring sliding panels (Fragments), one for each application.
***************************************************************
## Table of Contents <a id="table-of-contents"></a>
  * [Discovery](#discovery)
  * [System Check](#systemcheck)

*********************************************************
### 00 - Discovery <a id="discovery"></a>
![SB Assistant](/images/sb-discovery.png)
````                        SB Assistant - Discovery Page ````
[toc](#table-of-contents)

The discovery panel allows connection to a single running robot based on a MasterURI configured in the *Settings*. It reads values published from the robot through its ParameterServer to validate the device.  

The  SQLite database on the tablet contains a list of applications expected to be supported by the robot. The panel provides an opportunity to select and start applications from this list. The robot is configured for the desired application via a *ssh* connection where a script is run to set the application and restart *rosCore*. This operation requires that a sudoer username and password for the robot be entered in the *Settings* panel on the tablet.


******************************************************
### 01 - System Check <a id="systemcheck"></a>
[toc](#table-of-contents)

The **system** panel is designed to read system parameters and exercise devices on the robot. The panel subscribes to the following topics:
 * /sb_system/System

******************************************************
### 02 - Follow <a id="follow"></a>
With the *follow* application running, walk in front of the TurtleBot3. Then, slowly walk away from the TurtleBot. The robot should move forward. Moving close to the TurtleBot will cause it to back away. Moving slowly to the left or right will cause the TurtleBot to turn. To stop the robot from following, walk quickly away from the robot.

******************************************************
### 03 - Auto-park <a id="follow"></a>
When this application is running, the robot will search for a white tag that marks its intended parking space.
