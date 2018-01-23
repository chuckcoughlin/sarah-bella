# Extensions to Core Configuration

This document summarizes hardware, software and configuration extensions to support specific applications on the Sarah-Bella application.

***************************************************************
## Table of Contents <a id="table-of-contents"></a>
  * [Parameters](#parameters)
  * [Topics](#topics)
  * [Applications](#applications)

*********************************************************
### Parameters <a id="parameters"></a>
[toc](#table-of-contents)

The following parameters are expected to be defined/published in every robot launch script:

* /robot/name  - the name of the robot
* /robot/type  - the robot type (e.g. turtlebot3)
* /robot/platform - the hardware platform (e.g. Linux, RaspberryPi)
* /robot/application - the name of the application currently running on the robot

### Topics <a id="topics"></a>
[toc](#table-of-contents)

The following custom topics are defined on the robot:
```
/sb_system
      String hostname
      String ip_address
      Float32 cpu_percent
      Float32 memory_percent_used
      Uint32 free_memory_bytes
      Float32 swap_memory_percent_used
      Float32 disk_percent_used
      Uint32 packets_sent
      Uint32 packets_received
      Uint32 in_packets_dropped
      Uint32 out_packets_dropped
```
These topics are derived from third-party sources:

### Applications <a id="applications"></a>
[toc](#table-of-contents)

This section lists each application and documents the extensions required to
implement it.

##### Discovery
Discovery makes use of the robot parameters that are expected to be part of every application.

##### 01 - System Check
*psutil* is used to obtain system performance metrics. This package is installed by default on the Linux virtual machine, but not on the
Raspberry Pi. For a complete description see: https://psutil.readthedocs.io/en/latest. To install:
```
   sudo apt-get install build-essential python-dev python-pip
   sudo pip install psutil
```
