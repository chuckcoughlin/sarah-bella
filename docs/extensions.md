# Extensions to Core Configuration

This document summarizes hardware and software extensions to support specific applications on the Sarah-Bella application.

*********************************************************
### Discovery
The process of discovery is to determine what applications a connected robot supports. We wanted to use an existing app_manager, but were unable to assemble all the pieces for our kinetic version of ROS. Consequently we created a low-budget version based on statically configured database tables.
 * robot publishes a list of supported apps as AppList in the ParameterTree
 * Android reads the AppList and does a database lookup to determine the correct compliment of Publishers and Subscribers for that application
 * User enables the desired application from the displayed list.


### Information
A compendium of java source files may be found at: http://do.dellin.net/ros-indigo-20150929T22:30:44Z.html. The following libraries have served as source resources, but were not imported literally.<br/>
* https://github.com/ros-android/android_app_manager
* https://github.com/ollide/rosjava_android_template
* https://github.com/rosjava/rosjava_mvn_repo/tree/master/org/ros/rosjava_messages/rocon_app_manager_msgs/0.9.0 rocon_app_manager_msgs-0.9.0.jar, rocon_app_manager_msgs-0.6.0.jar
