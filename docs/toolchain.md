# Toolchain and Build Notes

This document summarizes the tools and build procedures required to build the Sarah-Bella application suite. Current hardware consists of the following:
  * Build system - iMac running OSX Sierra (10.12).
  * Tablet - Samsung Galaxy S3 - 10"
  * Robot - ROBOTIS Turtlebot3, monitor, USB keyboard

We will use the kinetic version of ROS. This is the current version used by ROBOTIS for Turtlebot.

### Android
The control application is a standard Android application built using Android Studio 3.0. The studio may be downloaded from http://developer.android.com (preview). It runs directly on the host build system. 
Make the Android home environment variable available by adding 
```ANDROID_HOME=~/Library/Android/sdk``` to ~/.bashrc.
From https://github.com/rosjava/rosjava_mvn_repo/blob/master/org/ros/android_core/android_15/0.3.3 download android_15-0.3.3.aar and place it in ~/robotics/sarah-bella/android/app/libs.

#### SB-Assistant
This notepad application is designed to command the robot, perform compute-intensive analyses and display results. 

### Linux
Creation of ROS control code for the robot requires a Linux machine. We have implemented this as a pair of virtual machines on the host build system. 
The first machine is the development area. Here C++ code for the entire repertoire of applications and support packages is compiled and tested. Testing is accomplished via simulation.

The second machine is sized to fit the robot's Raspberry Pi. Here, a particular application is compiled into an executable image and copied to an SD card.
On startup. the Raspberry Pi boots from that card.

#### VirtualBox Setup
We use VirtualBox on an iMac host to implement our Linux virtual machines. The application may be downloaded from http://www.oracle.com/technetwork/server-storage/virtualbox/downloads/index.html. 

The VirtualBox installation requires an additional package in order to support a file system shared with the host. From https://www.virtualbox.org/wiki/Downloads download, then from the VirtualBox "devices" menu, 
mount the VirtualBox Extension Packaage.  Either let the script run automatically or 
navigate to /media/<username>VBOXADDITIONS_5.1.28_117968 and execute:

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

This will mount the shared filesystems whenever the virtual machine starts, giving file ownership to user 1000 and group 1000. Note that we've seen cases where the sharing is not in effect
on startup. In this case 'sudo umount share' and mount manually. The equivalent command to mount manually is:  
```
        sudo mount -t vboxsf share /home/<username>/robotics/share -o umask=0022,uid=1000,gid=1000
```
The VirtualBox additions also provide for a shared clipboard. 

#### ROSDev Setup
Create a virtual machine "ROSDev" to house ROS development activities. These activities include package development as well as construction of the entire suite of 
applictions that will eventually run on the robot.
Use an Unbuntu 16.04 boot image downloaded from https://www.ubuntu.com/download/desktop/contribute?version=16.04.3&architecture=amd64r.
Create a virtual machine sized at 6gb of RAM and 50gb of disk.

The Robot Operation System (ROS) build environment runs on the Linux virtual machine.
For installation, see http:wiki.ros.org/kinetic/installation/Ubuntu. Complete the steps in section 1. Install the ros-kinetic-desktop-full suite.

Install the dependent packages for Turtlebot3 control:
```
        sudo apt-get install ros-kinetic-joy ros-kinetic-teleop-twist-joy ros-kinetic-teleop-twist-keyboard ros-kinetic-laser-proc ros-kinetic-rgbd-launch \
		ros-kinetic-depthimage-to-laserscan ros-kinetic-rosserial-arduino ros-kinetic-rosserial-python ros-kinetic-rosserial-server ros-kinetic-rosserial-client \
		ros-kinetic-rosserial-msgs ros-kinetic-amcl ros-kinetic-map-server ros-kinetic-move-base ros-kinetic-urdf ros-kinetic-xacro ros-kinetic-compressed-image-transport \
		ros-kinetic-rqt-image-view ros-kinetic-gmapping ros-kinetic-navigation
```

#### Turtlebot3 Support
Install the source code for turtlebot support packages.

```
		mkdir -p ~/robotics/catkin_ws/src
		cd ~/robotics/catkin_ws/src
		git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
		git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
		cd .. && catkin_make
```
#### Package Creation
A package encapsulates a related set of robot functionality. Ultimately we need to transfer the package definition onto the
robot, but first we checkout and build on the virtual machine. 

Custom packages all reside as sub-directories of a "catkin" workspace. By convention, this parent 
directory for all development of custom packages is "catkin_ws". 
A package build involves the entire tree. In order to make our packages portable to the target Raspberry Pi, we will use
python as much as possible.

The narrative below describes the general steps involved in creating a package. For further details see:
http://wiki.ros.org/catkin/Tutorials. Make sure that the following lines have been added to ~/.bashrc:
```
    source /opt/ros/kinetic/setup.bash
	source ~/robotics/catkin_ws/devel/setup.bash
```
Note that the standard ROS packages are installed in /opt/ros/kinetic/share. These can be browsed to determine
message contents and other details.

To create a new package (assuming a python and turtlebot top-level dependencies):
```
		cd ~/robotics/catkin_ws/src
        catkin_create_pkg <package name> rospy turtlebot3_msgs turtlebot3_navigation
```
Note that the metapackage turtlebot3 should not be listed as a dependency. Use its sub-packages instead.

Edit the resulting package.xml file appropriately. Then:
```
		cd ~/robotics/catkin_ws
		catkin_make
```

Once the package has been created and compiles with its dependencies, proceed to define its custom code. See 
http:://wiki.ros.org/rospy_tutorials for guidance. The custom code consists of execution nodes, messages,
services and topic files, as appropriate.

###### Transfer to the Robot
Once the package has been defined and tested on the development system, it needs to be packaged and transferred to the
robot. Several scripts are provided for this purpose.

###### System Configuration
We keep system-specific configurations in ~/robotics/conf.d. On startup each file in this directory is source'd to create
environment variables that are referenced by the ROS packages and build scripts where necessary.

### Raspberry Pi
"ROSPi" (our name) is the RaspberryPi that is the robot's single board computer (SBC). After an initial flash and configuration
this device must be separately configured for each application. A wi-fi connection is used for file transfer with commands
entered via external keyboard, mouse and monitor.

#### Initial Image
On the host machine, download and install "Etcher" from https://etcher.io. We will use this to transfer images to the SD card.
Download Ubuntu Mate for RaspberryPi 3 from https://ubuntu-mate.org/download. This is the initial boot image.
For the actual download, you will need a torrent translator like qBittorent at https://www.qbittorrent.org/download.php.
The downloaded file has an ".xz" extension, meaning that it is compressed. On a mac, the "xz" utility can be downloaded
from homebrew as:

```
    ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)" < /dev/null 2> /dev/null
    brew install xz
```
Use "unxz" to de-compress the image.

Using Etcher, flash the decompressed image onto the SD card. On my system, this is /dev/disk7. Flash time is approximately 5 minutes.

#### ROSPi Network Configuration
We woud like to have ROSPi connect automtically to the wireless network on startup so as to not require any user intervention for normal operations.
We have found that the selection of wi-fi networks is easiest during the initial system configuration. Select the desired network and mark it to
automatically connect if found. It may not be available during the first login, but will connect on subsequent system restarts. 

To test and to determine the IP address, restart ROSPi and execute "ifconfig".

#### ROS Development Setup
Like the main deveopment system, the Raspberry Pi requires the ROS development libraries. Follow the instructions at:
http://turtlebot3.readthedocs.io/en/latest/sbc_software.html, sections 6.3.1, 6.3.3 and 6.3.4.. These steps can be expected to
take an hour or more. Make sure the robot's battery is charged.

After I executed the "sudo apt upgrade", I discovered that my Firefox now crashed on start. To revert to a prior version:
```
    apt-cache show firefox | grep Version
	sudo apt-get purge firefox
	sudo apt-get install firefox=45.0.2+build1-0ubuntu1   # The older version shown from previos command
	sudo apt-mark hold firefox                            # Prevents auto-update to newer version
```
When complete, execute:
```
    rosdep update
```
Replace similar lines in ~/.bashrc:
```
   IP_ADDRESS=`hostname-I`
   export ROS_MASTER_URI="http://${IP_ADDRESS}:11311"
   export ROS_HOSTNAME=${IP_ADDRESS}
```

The ROS installation places the ROS workspace at /home/<username>/catkin_ws. Unfortunately it updates firefox, so it is necessary
to revert the version again.

#### FTP
We use ftp to transfer files from the host to the Raspberry Pi. To do this install 
the vsftpd package.

#### Backup
To backup an SD card, mount it on the host system. Then use the Disk Utility application to save the SD card contents
to an image file on disk. Be sure to select the entire device, not just the named partition. Save as "compressed".
