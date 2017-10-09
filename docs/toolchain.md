# Toolchain and Build Notes

This document summarizes the tools and build procedures required to build the Sarah-Bella application suite. Current hardware consists of the following:
  * Build system - iMac running OSX Sierra (10.12).
  * Tablet - Samsung Galaxy S3 - 10"
  * Robot - Turtlebot 3

We will use the lunar version of ROS.

### SB-Assistant
The control application is a standard Android application built using Android Studio 2.3.3. The studio may be downloaded from http://developer.android.com. It runs directly on the build system. The purpose of the application 

### Flasher
Creation of the ROS control code for the robot requires a Linux machine. We have implemented this as a virtual machine on the build system. The C++ control code is compiled and combined into an
image which is then flashed onto the robot's Raspberry Pi.

#### VirtualBox Setup
"flasher" is a VirtualBox downloaded from http://www.oracle.com/technetwork/server-storage/virtualbox/downloads/index.html. Its boot image is Unbuntu 16.04 https://www.ubuntu.com/download/desktop/contribute?version=16.04.3&architecture=amd64.
The virtual machine is sized at 2gb of RAM and 20gb of disk.

The VirtualBox installation requires an additional package in order to support a file system shared with the host. From https://www.virtualbox.org/wiki/Downloads download, then install, the VirtualBox extension pack.
u will need to navigate to /media/VBOXADDITIONS_5.1.28_117968 and execute:

```
		./VBoxLinuxAdditions.run
```

Manipulate the VirtualBox menu until "Devices" shows. Under "Shared Folders" create a virtual device, say "robotics" pointing to an existing directory on the host system, say "/Users/<username>/robotics/share". 

Then within the virtual machine:

```
		sudo mkdir /home/<username>/robotics/share
		sudo echo 'robotics /home/<username>/robotics/share vboxsf defaults 0 0'>>/etc/fstab
```

This will mount the shared filesystem whenever the virtual machine starts. The command to mount manually is:  
```
        sudo mount -F vboxfs robotics /home/<username>/robotics/share
```


#### ROS Setup
The Robot Operation System (ROS) build enviromnent runs on the Linux virtual machine.
For installation, see http:wiki.ros.org/lunar/installation, then Ubuntu. Complete the steps in section 1. Install the ros-lunar-desktop-full suite.

