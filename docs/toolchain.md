# Toolchain and Build Notes

This document summarizes the tools and build procedures required to build the Sarah-Bella application suite. Current hardware consists of the following:
  * Build system - iMac running OSX Sierra (10.12).
  * Tablet - Samsung Galaxy S3 - 10"
  * Robot - ROBOTIS Turtlebot3

We will use the kinetic version of ROS. This is the current version used by ROBOTIS for Turtlebot.

### Android
The control application is a standard Android application built using Android Studio 2.3.3. The studio may be downloaded from http://developer.android.com. It runs directly on the host build system. 
#### SB-Assistant
The notepad application is designed to command the robot, perform compute-intensive analyses and display results. 

### Linux
Creation of ROS control code for the robot requires a Linux machine. We have implemented this as a pair of virtual machines on the host build system. 
The first machine is the development area. Here C++ code for the entire repertoire of applications and support packages is compiled and tested. Testing is accomplished via simulation.

The second machine is sized to fit the robot's Raspberry Pi. Here, a particular application is compiled into an executable image and copied to an SD card.
On startup. the Raspberry Pi boots from that card.

#### VirtualBox Setup
We use VirtualBox ion an iMac host to implement our Linux virtual machines. The application may be downloaded from http://www.oracle.com/technetwork/server-storage/virtualbox/downloads/index.html. 

The VirtualBox installation requires an additional package in order to support a file system shared with the host. From https://www.virtualbox.org/wiki/Downloads download, then install, the VirtualBox extension pack.
You will need to navigate to /media/<username>VBOXADDITIONS_5.1.28_117968 and execute:

```
		./VBoxLinuxAdditions.run
```

Manipulate the VirtualBox menu until "Devices" shows. Under "Shared Folders" create a virtual device, say "share" pointing to an existing directory on the host system, say "/Users/<username>/robotics/share". 
This provides a way to transfer our build products to the host so that it can flash SD cards.

Then within the virtual machine:

```
		sudo mkdir /home/<username>/robotics/share
		sudo echo 'share /home/<username>/robotics/share vboxsf umask=0022,uid=1000,gid=1000'>>/etc/fstab
```

This will mount the shared filesystems whenever the virtual machine starts, giving file ownership to user 1000 and group 1000. The equivalent command to mount manually is:  
```
        sudo mount -F vboxfs robotics /home/<username>/robotics/share -o umask=0022,uid=1000,gid=1000
```
The VirtualBox additions also provide for a shared clipboard. 

#### ROSDev Setup
Create a virtual machine "ROSDev" to house ROS development activities. These activities include package development as well as construction of the entire suite of 
applictions that will eventually run on the robot.
Use an Unbuntu 16.04 boot image downloaded from https://www.ubuntu.com/download/desktop/contribute?version=16.04.3&architecture=amd64.
Create a virtual machine sized at 6gb of RAM and 50gb of disk.

The Robot Operation System (ROS) build environment runs on the Linux virtual machine.
For installation, see http:wiki.ros.org/kinetic/installation, then Ubuntu. Complete the steps in section 1. Install the ros-kinetic-desktop-full suite.

Install the dependent packages for Turtlebot3 control:
```
        sudo apt-get install ros-kinetic-joy ros-kinetic-teleop-twist-joy ros-kinetic-teleop-twist-keyboard ros-kinetic-laser-proc ros-kinetic-rgbd-launch \
		ros-kinetic-depthimage-to-laserscan ros-kinetic-rosserial-arduino ros-kinetic-rosserial-python ros-kinetic-rosserial-server ros-kinetic-rosserial-client \
		ros-kinetic-rosserial-msgs ros-kinetic-amcl ros-kinetic-map-server ros-kinetic-move-base ros-kinetic-urdf ros-kinetic-xacro ros-kinetic-compressed-image-transport \
		ros-kinetic-rqt-image-view ros-kinetic-gmapping ros-kinetic-navigation
```

#### ROS Development Setup
The instructions below describe setup of a "catkin" workspace for application and package development. We have constructed the following directories:

###### ~/robotics/common: Install the turtlebot3 messages and catkin build files here. 
###### ~/robotics

```
		cd ~/robotics/src/applications/APP1/src
		git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
		git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
		cd ~/catkin_ws && catkin_make
```

The instructions below describe setup of a "catkin" workspace for a typical package. Unlike an application, the package has to
be bundled and added to the ROS depeendencies..

#### PiBoot
"PiBoot" is a Linux image specifically designed as a boot image for the Raspberry Pi.
Image preparation:

Download "Etcher" from https://etcher.io. Use this to transfer the image to the SD card.
