Robotics_msgs
=============
Robotics_msgs is the rcm component that provide the communication language between the rcm driver
and firos to deal with the fiware world. Provides the messages types and the type of communication
used by rcm driver and firos to communicate under ROS environment.

Installing Robotics_msgs
========================

Requirements
------------

- RCM
- rcm_driver
- firos

Creation of the RCM distribution package
----------------------------------------

The creation of the distribution package of this component is integrated in the main
RCM distribution package creation. In any case you can do it separately using the
following command to create the file *robotics_msgs.zip*:
    
  - `RCM/scripts/create_rcm_platform_pkg robotics_msgs`

Remember that in order to use the created package outside the main process you have
to install it manually.

Installation
------------

The installation of this component is integrated in the main RCM installation. But
you can manually install it unzipping *robotics_msgs.zip* in the RCM workspace folder 
`rcmp_ws` under `src` as you do for custom ROS nodes.
 
Uninstallation
--------------

Delete the folder `rcmp_ws/src/robotics_msgs` from the RCM workspace.
