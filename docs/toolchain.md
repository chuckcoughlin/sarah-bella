# Toolchain and Build Notes

This document summarizes the tools and build procedures required to build the Sarah-Bella application suite. Current hardware consists of the following:
  * Build system - iMac running OSX Sierra (10.12).
  * Tablet - Samsung Galaxy S3 - 10"
  * Robot - Turtlebot 3

### SB-Control
The control application is a standard Android application built using Android Studio 2.3.3. The studio may be downloaded from http://developer.android.com. It runs directly on the build system. The purpose of the application 

### Flasher
Creation of the ROS control code for the robot requires a Linux machine. We have implemented this with as a virtual machine on the build system.

## Setup
"flasher" is a VirtualBox downloaded from http://www.oracle.com/technetwork/server-storage/virtualbox/downloads/index.html. Its boot image is Unbuntu 16.04 https://www.ubuntu.com/download/desktop/contribute?version=16.04.3&architecture=amd64.
