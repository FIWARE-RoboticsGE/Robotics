RCM driver
==========
RCM driver is the rcm component that provide the communication between the rcm platform instance
and firos to deal with the fiware world. It uses robotics_msgs as common language with firos under
ROS environment.

Installing RCM driver
=====================

Requirements
------------

- RCM
- robotics_msgs
- firos

Creation of the RCM distribution package
----------------------------------------

The creation of the distribution package of this component is integrated in the main
RCM distribution package creation. In any case you can do it separately using the
following command to create the file *rcm_driver.zip*:

  - `RCM/scripts/create_rcm_platform_pkg rcm_driver`

Remember that in order to use the created package outside the main process you have
to install it manually.

Installation
------------

The installation of this component is integrated in the main RCM installation. But
you can manually install it unzipping *rcm_driver.zip* in the RCM workspace folder 
`rcmp_ws` under `src` as you do for custom ROS nodes.
 
Uninstallation
--------------

Delete the folder `rcmp_ws/src/rcm` from the RCM workspace.
