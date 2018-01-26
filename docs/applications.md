## Application Descriptions

The robot repertoire consists of set of applications constructed
on a Linux virtual machine running on the development system.  These are subsequently transferred to and compiled on the Turtlebot3. The robot launch configuration determines which application will execute.

The sections below contain general descriptions
of the applications with a brief view of applicable control screens on the Android tablet.
ROS nodes and messaging details as well as descriptions of hardware modifications are described in  extensions](https://github.com/chuckcoughlin/sarah-bella/tree/master/docs/extensions.md).  

![SB Assistant](/images/sb-cover.png)
````                        SB Assistant - Cover Page ````

The android assistant contains command and monitoring code for all applications installed on the robot. It is a single activity application, featuring sliding panels (Fragments), one for each application.
***************************************************************
## Table of Contents <a id="table-of-contents"></a>
  * [Discovery](#discovery)
  * [System Check](#systemcheck)

*********************************************************
### 00 - Discovery <a id="discovery"></a>
[toc](#table-of-contents)

The discovery panel connects to a single running robot based on a MasterURI configured in the *Settings*. It reads values published through the ParameterServer which define the robot configuration.  

The  SQLite database contains a list of applications supported by the robot. The panel provides the ability to select and start applications from this list. Application selection is made via a *ssh* connection to the robot, specification of the application and a restart of *rosCore*. The sudoer username and password must be entered in the *Settings* panel.


******************************************************
### 01 - System Check <a id="systemcheck"></a>
[toc](#table-of-contents)

The **system** panel is designed to read system parameters and exercise all devices on the robot.

The following nodes are defined (shown with topics):

#### ----------------------- tablet -------------------------

**node:** sb_subscribe_system (sb_system/System)<br/>

#### ---------------------- robot  --------------------------

**node:** sb_publish_system (sb_system/System)<br/>

******************************************************
##### 02 - Follow <a id="follow"></a>
With the *follow* application running, walk in front of the TurtleBot3. Then, slowly walk away from the TurtleBot. The robot should move forward. Moving close to the TurtleBot will cause it to back away. Moving slowly to the left or right will cause the TurtleBot to turn. To stop the robot from following, walk quickly away from the robot. 
