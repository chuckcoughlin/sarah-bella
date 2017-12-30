# Toolchain and Build Notes

This document summarizes the tools and build procedures required to build the Sarah-Bella application suite. Current hardware consists of the following:
  * Host system - iMac running OSX High Sierra (10.13).
  * Tablet - Samsung Galaxy S3 - 10" Android 7.0 (SDK version 24).
  * Robot - ROBOTIS Turtlebot3, monitor, USB keyboard

We use the kinetic version of ROS. This is the current version used by ROBOTIS for Turtlebot.

![sarah-bella - Turtlebot3](/images/sarah-bella-large.png)
````                        sara-bella - Turtlebot3 ````

Here is a diagram that shows the relationships between the various development components.
![System Architecture for Development](/images/development-layout.png)
````                        Development - System Architecture ````

On the host system:
 * Android Studio is used to create the Android application 'SBAssistant' which executes on the tablet and is used as the user interface to command the robot.
 * The Linux VM is a Virtual Box virtual machine with the Robot Operating System (ROS) libraries. It is used to create and test-compile the robot code. When done, the code is checked into ''git'' and subsequently checked out and built on the robot.

This drawing and others is constructed using **InkScape** from https://inkscape.org/en/release/0.92.2.
*********************************************************
### Android
The control application is a standard Android application built using Android Studio 3.0. The studio may be downloaded from http://developer.android.com. It runs on the OSX build system, creating a run image
for the android tablet.

Configure the host build system, making the Android home environment variable available by adding
```ANDROID_HOME=~/Library/Androd/sdk``` to ~/.bashrc.

#### External Libraries
The next steps make ROS libraries available to the Android build environment. Note that the following are necessary only if creating a project from scratch. For an existing project, the modules are already part of the SBAssistant repository.

Load these libraries directly into the project as source. They were not distributed as .jar or .aar file and have been pruned drastically to reduce user interface dependencies.<br/>
* https://github.com/ros-android/android_app_manager
* https://github.com/ollide/rosjava_android_template

The android core package may be found at (you may have to download the entire Maven repository to get at the individual files):  https://github.com/rosjava/rosjava_mvn_repo/tree/master/org/ros/rosjava_core/rosjava/0.3.5. Download rosjava-0.3.5.jar (org.ros) to ~/robotics/sara-bella/SBAssistant/app/libs. Then from inside Android Studio:
```
  File->Synchronize
  right-click on app/libs/rosjava-0.3.5.jar and select "Import as Library"
```

Create libraries in a similar way from the following:<br/>
* https://hc.apache.org/downloads.cgi httpclient-4.5.4.jar, httpcore-4.4.7.jar (download httpcomponents-4.5.4, then untar to extract)
* https://github.com/rosjava/rosjava_mvn_repo/tree/master/org/ros/rosjava_messages/app_manager/1.0.2 app_manager-1.0.2.jar (app_manager)
* https://github.com/rosjava/rosjava_mvn_repo/tree/master/org/ros/rosjava_bootstrap/message_generation/0.3.0 message_generation-0.3.0.jar (org.ros.internal.message)
* http://repo1.maven.org/maven2/org/jboss/netty/netty/3.2.9.Final netty.3.2.9.Final.jar (org.jboss.netty)


The following are sources of various ROS message definitions. Some class definitions were decompiled into java and moved to package ros.android.
* https://github.com/rosjava/rosjava_mvn_repo/tree/master/org/ros/rosjava_messages/rocon_app_manager_msgs/0.9.0 rocon_app_manager_msgs-0.9.0.jar, rocon_app_manager_msgs-0.6.0.jar


#### Genjava
``genjava``is used to convert ROS message files into java classes. Instructions for installation and use are based on: http://wiki.ros.org/kinetic/Installation/OSX/Homebrew/Source, http://wiki.ros.org/wstool#Installation and http://wiki.ros.org/rosjava/Tutorials/indigo/RosJava%20Message%20Artifacts.
```
    brew update
    brew install cmake
    brew tap ros/deps
    brew install Python
    sudo -H python2 -m pip install -U wstool rosdep rosinstall rosinstall_generator rospkg catkin-pkg sphinx

    wstool set genjava --git https://github.com/rosjava/genjava --version=kinetic
```
#### SB-Assistant
This notepad application is designed to command the robot, perform compute-intensive analyses and display results. The SBAssistant project is contained in the overall project
repository (```git clone http://github.com/chuckcoughlin/sarah-bella``` Load android/SBAssistant into Android Studio). Internet access is required to build.

#### Emulator
In order to test the application in the emulator, after configuring a suitable target device with the AVD manager, select the studio menu Tools->Android->Enable ADB Integration and Run->Edit Configuration. Thereafter use the green "run" arrow in the toolbar to launch the application in the emulator. Output is viewable directly in the studio's ''logcat'' tab.

#### Persistent Storage
Configuration parameters, maps and other data that are meant to remain in place even through application changes, are stored in a SQLite database accessible through the tablet application. The database can also be read externally.

On the tablet:
```
/data/user/0/chuckcoughlin.sb.assistant/databases/SBAssistant.db
```

For the emulator (running the debug version):
```
   cd ~/Library/Android/sdk/platform-tools
   ./adb shell
   run-as chuckcoughlin.sb.assistant
   cd /data/data/chuckcoughlin.sb.assistant/databases
   sqlite3 SBAssistant.db
       (query the data and schema with SQLite commands)
   .q
   exit
   exit
```

#### Transfer to Tablet
The tablet must be set in "developer" mode. This is accomplished under Settings->About Tablet. Tap on "Build number" 7 times. (Yes, really). Under Settings->Developer options, enable USB debugging. Connect the tablet and host using the same USB cable that is used to charge the device. Once the cable is connected a dialog should popup asking you to allow file transfer. (If this does not appear, you may have to fiddle with Developer options->USB Configuration).

On the build system, configure Android Studio (Tools->Run>Edit Configurations) to target the build output directly to a USB device. After a successful build, merely select the "run" button to transfer the **apk** executable to the tablet.

To transfer non-apk files download and install Android File Transfer from http://www.android.com/filetransfer. This tool facilitates transfer of files between the host build system and tablet.

***************************************************************

### Linux
The robot's ROS control code is developed on a Linux machine. We implement this as a virtual machine on the OSX host build system.
This machine contains a development area which is linked to the robot's Raspberry Pi via a shared *git* source code repository.
Python (mostly) and C++ code for the entire repertoire of applications and support packages is edited and compiled here. Actual tryout and testing must take place on the robot.

##### VirtualBox Setup
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

##### ROSDev Setup
On the host OSX system, create a virtual machine "ROSDev" to house ROS development activities. These activities include package development as well as construction of the entire suite of
applications that will eventually run on the robot.
Use an Unbuntu 16.04 boot image downloaded from https://www.ubuntu.com/download/desktop/contribute?version=16.04.3&architecture=amd64r.
Create a virtual machine sized at 6gb of RAM and 50gb of disk.

Next, install the Robot Operation System (ROS) build environment.
For details, see http:wiki.ros.org/kinetic/installation/Ubuntu. Complete the steps in section 1. Install the *ros-kinetic-desktop-full* suite.

Now install dependent packages for Turtlebot3 control and Java message generation:
```
    sudo apt-get install ros-kinetic-joy ros-kinetic-teleop-twist-joy ros-kinetic-teleop-twist-keyboard ros-kinetic-laser-proc ros-kinetic-rgbd-launch \
		ros-kinetic-depthimage-to-laserscan ros-kinetic-rosserial-arduino ros-kinetic-rosserial-python ros-kinetic-rosserial-server ros-kinetic-rosserial-client \
		ros-kinetic-rosserial-msgs ros-kinetic-amcl ros-kinetic-map-server ros-kinetic-move-base ros-kinetic-urdf ros-kinetic-xacro ros-kinetic-compressed-image-transport \
		ros-kinetic-rqt-image-view ros-kinetic-gmapping ros-kinetic-navigation
```

##### RosJava Support
This section configures the virtual machine to build .jar files suitable for use in the Android build environment. These files are Java equivalents of the ROS message files. Install ``rosjava``.

```
    sudo apt-get install ros-kinetic-catkin ros-kinetic-rospack python-wstool openjdk-8-jdk
    sudo apt-get install ros-kinetic-genjava
    sudo apt-get install --only-upgrade ros-kinetic-*
```

Build a catkin workspace specifically for ``rosjava``.
```
  cd ~/robotics
  mkdir -p rosjava_messages/src
  cd rosjava_messages/src
  source /opt/ros/kinetic/setup.bash
  cd ..


```

##### Turtlebot3 Support
Install the source code for turtlebot ROS support packages.

```
		mkdir -p ~/robotics/catkin_ws/src
		cd ~/robotics/catkin_ws/src
		git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
		git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
		cd .. && catkin_make
```
##### Package Creation
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

A typical sequence for creating a new package will look like:
```
    cd ~/robotics/catkin_ws/src
    catkin_create_pkg <package name> rospy turtlebot3_msgs turtlebot3_navigation
```
Note that the metapackage turtlebot3 should not be listed as a dependency. Use its sub-packages instead.

Edit the resulting package.xml file appropriately. Then:
```
		cd ~/robotics/robot/catkin_ws
		catkin_make
```

Once the package has been created and compiles with its dependencies, proceed to define its custom code. See
http:://wiki.ros.org/rospy_tutorials for guidance. The custom code consists of execution nodes, messages,
services and topic files, as appropriate.

###### Transfer to the Robot
Once the package has been defined and tested on the development system, it needs to be packaged and transferred to the
robot. This is accomplished by simply checking the changes into *git* (pushing them, of course) and checking them out on the robot. Once checked out on the RaspberryPi, the new code must be built and executed.

###### System Configuration
We keep system-specific configurations in ~/robotics/conf. These files are referenced as necessary when the ROS packages are built on the robot.

**********************************************************
### Raspberry Pi
"ROSPi" (our name) is the RaspberryPi that is the robot's single board computer (SBC). After an initial flash and configuration
this device must be separately configured for each application. A wi-fi connection is used for file transfer with commands
entered via external keyboard, mouse and monitor.

##### Initial Image
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

##### ROSPi Network Configuration
We woud like to have ROSPi connect automtically to the wireless network on startup so as to not require any user intervention for normal operations.
We have found that the selection of wi-fi networks is easiest during the initial system configuration. Select the desired network and mark it to
automatically connect if found. It may not be available during the first login, but will connect on subsequent system restarts.

To test and to determine the IP address, restart ROSPi and execute "ifconfig".

##### ROS Development Setup

Make **robotics** the root of our **git** repository. We will use **git** as the integrating mechanism with our development machine. Establish
links within **catkin_ws** in order to merge our repository with the standard ROS packages. Make sure that the link is created before the
ROS package downloads.
```
  cd
  git clone http://github.com/chuckcoughlin/sarah-bella robotics
  git checkout --track origin/robot      # Always use the 'robot' branch
  mkdir catkin_ws
  cd catkin_ws
  ln -s ~/robotics/repo/robot/config config
  ln -s ~/robotics/repo/robot/src src
```

Like the main development system, the Raspberry Pi requires the ROS development libraries. Follow the instructions at:
http://turtlebot3.readthedocs.io/en/latest/sbc_software.html, sections 6.3.1, 6.3.3 and 6.3.4. These steps can be expected to
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

We want **roscore** to start automatically when the RaspberryPi is booted. Install the init file from the git repository as follows:
```
   cd ~/robotics/repo/robot/src/bin
   sudo cp ros /etc/init.d
   sudo chmod 755 /etc/init.d/ros
   sudo update-rc.d ros defaults
```
The package to be started is set by editing ```catkin_ws/config/launch.conf```, setting the desired package and launch file.

##### GPIO
The GPIO pin layout is shown below:

![GPIO](/images/RaspberryPi-GPIO.png)
````                        RaspberryPi GPIO Pin Assignments ````

Download the **wiringPi** GPIO library and build the 'gpio' tool. Copy into our **bin** area for use in ROS scripts as needed.

```
  mkdir -p ~/external
  cd ~/external
  git clone git://git.drogon.net/wiringPi
  cd wiringPi
  sudo ./build
  cp gpio/gpio ~/robotics/repo/robot/bin
```

##### FTP
We use ftp to transfer miscellaneous files from the host to the Raspberry Pi. To do this install
the vsftpd package.

##### Backup
To backup an SD card, mount it on the host system. Then use the Disk Utility application to save the SD card contents
to an image file on disk. Be sure to select the entire device, not just the named partition. Save as "compressed".
