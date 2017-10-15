# Toolchain and Build Notes

This document summarizes the tools and build procedures required to build the Sarah-Bella application suite. Current hardware consists of the following:
  * Build system - iMac running OSX Sierra (10.12).
  * Tablet - Samsung Galaxy S3 - 10"
  * Robot - ROBOTIS Turtlebot3

We will use the kinetic version of ROS. This is the current version used by ROBOTIS for Turtlebot.

### SB-Assistant
The control application is a standard Android application built using Android Studio 2.3.3. The studio may be downloaded from http://developer.android.com. It runs directly on the build system. The purpose of the application 

### Flasher
Creation of the ROS control code for the robot requires a Linux machine. We have implemented this as a virtual machine on the build system. The C++ control code is compiled and combined into an
image which is then flashed onto the robot's Raspberry Pi.

#### VirtualBox Setup
"flasher" is a VirtualBox downloaded from http://www.oracle.com/technetwork/server-storage/virtualbox/downloads/index.html. Its boot image is Unbuntu 16.04 https://www.ubuntu.com/download/desktop/contribute?version=16.04.3&architecture=amd64.
The virtual machine is sized at 6gb of RAM and 20gb of disk.

The VirtualBox installation requires an additional package in order to support a file system shared with the host. From https://www.virtualbox.org/wiki/Downloads download, then install, the VirtualBox extension pack.
you will need to navigate to /media/<username>VBOXADDITIONS_5.1.28_117968 and execute:

```
		./VBoxLinuxAdditions.run
```

Manipulate the VirtualBox menu until "Devices" shows. Under "Shared Folders" create a virtual device, say "share" pointing to an existing directory on the host system, say "/Users/<username>/robotics/share". 
This provides a way to transfer our build products to the host so that it can flash SD cards.

Then within the virtual machine:

```
		sudo mkdir /home/<username>/robotics/share
		sudo echo 'share /home/<username>/robotics/share vboxsf defaults 0 0'>>/etc/fstab
```

This will mount the shared filesystems whenever the virtual machine starts. The command to mount manually is:  
```
        sudo mount -F vboxfs robotics /home/<username>/robotics/share
```
The VirtualBox additions also provide for a shared clipboard. 

#### ROS Setup
The Robot Operation System (ROS) build environment runs on the Linux virtual machine.
For installation, see http:wiki.ros.org/kinetic/installation, then Ubuntu. Complete the steps in section 1. Install the ros-kinetic-desktop-full suite.

