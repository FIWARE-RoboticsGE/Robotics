===================================
Installation & Administration Guide
===================================

This guide tries to define the procedure to install and configure
RCM. The most important thing to understand before beginning is
that this is a distribute platform so for each device involved in
the platform must be installed RCM that should be seen as an agent
of the more complex architecture of the platform. From now on we
call the platform rcm platform and the single agent on a device
rcm instance.

------------
Requirements
------------

There should be at least one kind of machine involved in the
rcm platform, a server who hosts the rcm instance we call
rcm master. We call this machine master.
This is the base configuration in which you can use only
simulated robots on that machine.
The master must have the following software installed in order
to install rcm master:

	- ``Ubuntu 14.04 LTS/12.04 LTS`` (other versions are not tested)

	- ``Python 2.7``

	- ``ROS Indigo/Groovy`` (other versions are not tested)

	- ``OpenVPN Server``

	- ``Sqlite``

In order to add real robots to the rcm platform there should be
added other kind of machines called robots. Each robot hosts an
rcm instance called rcm robot.
The robot must have the following software installed in order
to install rcm robot:

	- ``Ubuntu 14.04 LTS/12.04 LTS`` (other versions are not tested)

	- ``Python 2.7``

	- ``ROS Indigo/Groovy`` (other versions are not tested)

	- ``OpenVPN client``

Python and sqlite should be already available in the Ubuntu OS
but in any case for all the software requirements refer to the
products pages to find out how download, install and configure
them.

In addition to the instruction provided by the products you
have to take care of a couple of things regarding RCM:

	- rcm master internally runs a web server to expose the
	  RDAPI. This web server is forced to use the network
	  interface called 'eth0' so make sure that this is the
	  main network interface accessible by the user.

	- rcm instances, as agents of the platform, need to deal
	  with the other agents in the platform and to do so they
	  use the VPN provided by OpenVPN server installed on the
	  master. They are forced to use the network interface
	  called 'tun0' so make sure to configure properly the
	  OpenVPN server and client to use that network interface.
	  Moreover remember to emit the certificates for all the
	  OpenVPN clients using the OpenVPN server on the master
	  in order to use the same VPN for all the platform.

------------
Installation
------------

The installation package is provided in a compressed file
named rcm_platform.zip with the following content:

::

	rcm_platform_pkg/
		rcm_platform/
			__init__.py
			rcmp_command.py
			rcmp_event.py
			rcmp_ext_connector.py
			rcmp_inter_communication.py
			rcmp_ms.py
			rcmp_node.py
			rcmp_platform_command.py
			rcmp_robotics_data.py
			rcmp_service_command.py
			rcmp_wd.py
		ros/
			rosconsole.config
		install.sh
		sub_inst.sh
		uninstall.sh
		sub_uninst.sh
		rcmpd
		rcmp_n

unzip the file and launch install.sh in rcm_platform_pkg
folder:

::

	~/rcm_platform_pkg$ ./install.sh

The procedure installs the package dependences and places all the
needed files in the right places: some of these steps require
the administration permission so after the first steps it will
be asked you the sudo password.
The installer provide a step by step configuration process
and can be used as a wizard to perform changes in the
configuration files. At the end of the process it will ask
if it has to start the rcm daemon. You can also change
directly the available configuration files or create them
from scratch.
If you don't let the installer start the rcm daemon or you
changed manually the configuration files, you have to start
or restart the rcm daemon manually as follow:

::

	~$ sudo service rcmpd start

or

::

	~$ sudo service rcmpd restart

.. _package dependences:

Package dependences
===================

RCM has two main dependences that are automatically installed
using apt-get tool during the installation:

	- ``python-netifaces`` for the access of the network interfaces
	  (tun0, eth0)

	- ``python-twisted`` to implement the web service

Installation files
==================

As said before, the installer places all the needed files in the
installation package in the right place and creates all the needed
folders; in this section we will list all the files and folders
arranged by the installation process and will give a brief
explanation for each of them.

	- ``~/rcmp_ws/`` this folder is created in the installer home
	  and is used by the platform as default workspace for ROS
	  custom nodes and launchers

	- ``/usr/local/bin/rcmp_n`` is the rcm instance start point,
	  the agent itself, that is started as daemon through rcmpd:
	  it uses the python modules in the python package rcm_platform
	  to do its work

	- ``/usr/local/lib/python2.7/dist-packages/rcm_platform/`` is
	  the core python package that implements all the feature of
	  RCM and contains all the python modules. The content will
	  be the following:

	  ::

		__init__.py
		rcmp_command.py
		rcmp_event.py
		rcmp_ext_connector.py
		rcmp_inter_communication.py
		rcmp_ms.py
		rcmp_node.py
		rcmp_platform_command.py
		rcmp_robotics_data.py
		rcmp_service_command.py
		rcmp_wd.py

	- ``/opt/rcm-platform/`` this folder is created in the
	  opt folder and is the base directory for the application;
	  it contains the configuration files, the robotics data,
	  the logs and the ROS logging configuration file. Remember
	  that init.cfg is only available in robots while .init.cfg
	  and robotics data are available only in the master

	- ``/etc/init.d/rcmpd`` is the script used to keep the rcm
	  instance as a daemon in the system where is installed; the
	  installer uses update-rc.d tool to insert into the system
	  init structure so that the daemon will be run at start time

-------------
Configuration
-------------

The configuration process prompts questions to the user,
proposes available responses within brackets in case of
selection queries and shows a default response within square
brackets (used in case the user presses only Enter key):

::

	~/rcm_platform_pkg$ ./install
	Is this RCM platform node the master ([Y]es/no)?

As said before we have two kind of agents in the platform so
this question provide the selection between an rcm master
agent that should be installed on the master and an rcm robot
agent that should be installed on the robots.
Rcm master doesn't have the standard configuration file
init.cfg but uses an hidden file .init.cfg that looks the
same but with only the list of the ports opened for inbound
communication; this instance directly controls the robotics
data so further creates and uses the sqlite database
file named robotics_data.db.
Rcm robot uses the standard configuration file and need a name
for the platform instance and the ip address where to find rcm
master too.

Don't forget that the list of ports for inbound communication
should be configured only if we need ports available from outside
the VPN so you have to open the ports on the NAT and firewall
to make them available to the platform. Moreover for what concern
the name and the ip address configured on the rcm robots is
important to remember that the ip address of the master must be
the one in the VPN (network interface 'tun0' is used by the
instances to communicate each other) and the name of the robot
must match the one provided during the provisioning phase (once
the rcm master is installed and running exposes RDAPI: to add
robots to the platform you have to call the provisioning web
service and provide the name there will be used for the robot).

Platform instance configuration files
=====================================

In the source repository is available under the folder cfg an
example of file configuration called init_template.cfg:

::

	# This is a template for the configuration file for an RCM platform node.
	# Remember that only robots and virtual machines must have this file specified and that must be renamed
	# with the name init.cfg.

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
	# ip_master=10.xx.yy.1

The file is a template of what is created with the wizard procedure
and can be used as base for the creation of the needed files from
scratch: before every option in the main (and only for now) section
there is a brief description of the meaning of that option.
The following examples are how the init.cfg file on the robot and
the .init.cfg file could look like:

::

	# init.cfg

	[main]
	name=robot1
	robot=yes
	ip_master=10.11.12.1

::

	# .init.cfg

	[main]
	inbound_open_ports=10100, 10101, 10102, 10103

Used ports
==========

The main port used by all the rcm instance in the platform is
9999 on TCP protocol to communicate each other: it is used
inside the VPN channel so there shouldn't be problems about
inbound/outbound access and firewall/NAT concerns.
Furthermore in case of the master we need the port 1194 on
UDP protocol used by OpenVPN server to provide the VPN channel
and the port 80 on TCP protocol used by rcm master to provide
the interaction with external users. These two ports require
inbound and outbound access so you have to open them on your
firewall/NAT.
Remember that all the ROS nodes in the ROS framework under the
RCM platform use a port to communicate with their master but
all these communications are inside the VPN channel. During
the configuration phase you will be asked for opened inbound
ports: these ports are needed only if you need that one ROS
node must be accessed from outside the platform. 
All inbound ports used by ROS nodes in these cases have to be
opened on your firewall/NAT: RCM suppose that all the opened
inbound ports are properly managed by the network manager in
your environment.

--------------
Uninstallation
--------------

The installation package rcm_platform.zip provides the tool
for uninstalling RCM. Enter into rcm_platform_pkg folder
extracted from the installation package and launch
uninstall.sh:

::

	~/rcm_platform_pkg$ ./uninstall.sh

this command removes all the files associated with RCM except
the `package dependences`_.

