# Toolchain and Build Notes

This document summarizes the tools and build procedures required to build the Sarah-Bella application suite. Current hardware consists of the following:
  * Build system - iMac running OSX Sierra (10.12).
  * Tablet - Samsung Galaxy S3 - 10"
  * Robot - ROBOTIS Turtlebot3, monitor, USB keyboard

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

#### ROS Development Setup
The instructions below describe setup of a "catkin" workspace for application and package development. We have constructed the following directories:

###### common 
Install the turtlebot3 messages and catkin build files here. 
###### config 
We keep system-specific configurations here. When applications require system-specific parameters we include the vaules from here.
###### applications
Source code for the robot applications.
###### packages
Source code for support packages.


```
		mkdir -p ~/robotics/common/src
		cd ~/robotics/common/src
		git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
		git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
		cd .. && catkin_make
```

The instructions below describe setup of a "catkin" workspace for a typical package. Unlike an application, the package has to
be bundled and added to the ROS depeendencies..

#### ROSPi
"ROSPi" is the RaspberryPi that is the robot's single board computer (SBC). After an initial flash and configuration
this device must be configured for each application. A wi-fi connection is used for file transfer with commands
entered via external keyboard, mouse and monitor.

##### Initial Image
On the host machine, download and install "Etcher" from https://etcher.io. Use this to transfer images to the SD card.
Download Ubintu Mate for RaspberryPi 3 from https://ubuntu-mate.org/download.
For the actual download, you will need a torrent translator like qBittorent at https://www.qbittorrent.org/download.php.
The downloaded file has an ".xz" extension, meaning that it is compressed. On a mac, the "xz" utility can be downloaded
from homebrew as:

```
        ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)" < /dev/null 2> /dev/null
		brew install xz
```
Use "unxz" to de-compress the image.

Using Etcher flash the decompressed image onto the SD card. On my system, this is /dev/disk7. Flash time is approximately 5 minutes.

##### ROSPi Network Configuration
We woud like to have ROSPi connect automtically to the wireless network on startup so as to not require any user intervention for normal operations.
The following material has been taken from: http://weworkweplay.com/play/automatically-connect-a-raspberry-pi-to-a-wifi-network. Note that the 
Pi comes with the "nano" editor. To save a file use ctrl-X, then Y.

Execute: sudo nano /etc/network/interfaces
```
        auto wlan0

        iface lo inet loopback
        iface eth0 inet dhcp

        allow-hotplug wlan0
        iface wlan0 inet static
        address 192.168.1.155    # Desired static IP
        netmask 255.255.255.0
        gateway 192.168.1.1      # IP of our router
        wpa-conf /etc/wpa_supplicant/wpa_supplicant.conf
        iface default inet dhcp
```

Execute: sudo nano /etc/wpa_supplicant/wpa_supplicant.conf
```
        ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
        update_config=1
        
        network={
	        ssid="MY_SSID"
	        psk="topsecret"
	        proto=RSN
	        key_mgmt=WPA-PSK
	        pairwise=CCMP
	        auth_alg=OPEN
        }
```
To test, restart ROSPi and execute "ifconfig".
