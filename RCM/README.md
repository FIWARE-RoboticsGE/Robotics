RCM
===
RCM stands for *Robot Clone Manager*, a distributed platform able to manage multiple 
Service Spaces ("Robot Clones") to host all the functions necessary to control a set 
of robots which support ROS (*Robot Operating System*, http://www.ros.org/).

Within the Robotics GE, RCM works together with FIROS in order to connect robots to 
the FIWARE world through the FIWARE Context Broker 
(http://catalogue.fiware.org/enablers/publishsubscribe-context-broker-orion-context-broker) 
as a way to publish and listen robot's data.  

Installing RCM
==============

Requirements
------------

- MASTER
 - Ubuntu 14.04 LTS/12.04 LTS (other versions are not tested)
 - ROS Indigo/Groovy (other versions are not tested)
 - Python 2.7.3
 - OpenVPN Server
   - VPN interface: **tun0**
 - WAN (Internet) interface: **eth0**

- Robots
 - Ubuntu 14.04 LTS/12.04 LTS (other versions are not tested)
 - ROS Indigo/Groovy (other versions are not tested)
 - Python 2.7.3
 - OpenVPN Client
    - A certificate emitted by the OpenVPN Server (MASTER)
    - VPN interface: **tun0**

Creation of the RCM distribution package
----------------------------------------

The following command creates the file *rcm_platform.zip*:
  - ` scripts/tmp_create_pkg`


Installation
------------

Unzip *rcm_platform.zip* and execute the following command:
- `rcm_platform_pkg/install`

Follow the step-by-step process to choose the role of the RCM instance (MASTER or Robot).
 
Uninstallation
--------------

Unzip *rcm_platform.zip* and execute the following command:
- `rcm_platform_pkg/uninstall`

Configuring RCM
===============

RCM uses a configuration file which is automatically created during the installation process.
- Robots: the file is mandatory and it is located at */opt/rcm-platform/init.cfg*.
- MASTER: the file is not mandatory, it is located at */opt/rcm-platform/.init.cfg* and contains only the information about *inbound_open_port*.

The format of the configuration file is:
```
[main]

# the name for the instance (rcm platform node)
# name=instance_name

# specify a list of ports that are previously opened for inbound communication
# inbound_open_ports=10100, 10101, 10102, 10103

# specify the type of the instance (rcm platform node) in case the system we are running on is a robot;
# used as flag to distinguish a robot from a vm instance (the master doesn't have this configuration file so 
# it is already different)
# robot=yes

# the ip of the master rcm platform node
# ip_master=192.168.0.1
```

RCM Specification API
=====================

http://docs.rdapi.apiary.io/#
