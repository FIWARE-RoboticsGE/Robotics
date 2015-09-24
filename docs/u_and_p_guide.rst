=======================
User & Programmer Guide
=======================

This guide tries to show the user how to use Robotics.
Robotics is composed by two components:

	- ``RCM`` (Robot Clone Manager) the distributed platform which
	  manages robots

	- ``Firos`` the bridge which connects robots to the FIWARE world

This is important to remember because a user of Robotics can be from one of the
two worlds, the robot world managed by RCM and the fiware world accessed and
linked through firos.

-------------------
Enable the platform
-------------------

The first thing to explain is how to enable the platform. A machine become
the platform when you install the main rcm platform agent, i.e. the rcm master (look
at the :doc:`i_and_a_guide` to see how to
do that).
Being the platform means that this machine (or master, as we call it) will be the
center of all the user will create in its robotic world: we are speaking of
simulated robots for now because we still don't have added physical robot in
that world but can be physical robot or machines that works as robots in the
platform; machines working as robots only means that you run the rcm robot
agent into them: in any case we'll see later how to `add robot to the platform`_.
At this point the platform is enabled and you can see that calling the web
services exposed by the master:

::

	~$ curl http://public_ip_master

The response is a simple html page remembering that rcm is responding and so
is up and running.
The web server runs only in the master so you can check only the master
installation in this way, but you know that a minimal configuration of the
platform is available.
You can have a look at what you have in your platform contacting platform_instance
and see that your platform doesn't have robot for now:

::

	~$ curl http://public_ip_master/platform_instance/read

The response is a json formatted response with the list of robots available
in the platform so you have an empty list for now.

.. _add robot to the platform:

----------
Add robots
----------

Now that your platform is available you can add the robots: these could be
physical robots, physical computers or virtual machines, but the important
thing is that they have to run the rcm robot agent.
Before that an rcm agent would be recognized by the platform you have to do
the provisioning of the robot; you have to tell the platform the ``name`` of
your robot and what will be its role, what it can do: we call this
``service logic`` (see `Create brains`_ to better understand the concept
and how to create a service logic).
You'll provide a new robot calling platform_instance in the rcm master
instance:

::

	~$ curl -H 'Content-Type: application/json' -d '{"pi_name":"name_r","pi_service_logic":"name_s_l"}' http://public_ip_master/platform_instance/provisioning

You can do this before or after running the machine with the rcm robot
agent but only when this operation will be done the robot will be available
on the platform and can be used.
The name of the robot must be unique because this will be the identifier for
the robot and all the components generated for that robot; the service logic
is mandatory too otherwise the provisioning fails.
The response provides the result of the operation but in any case you can see
the new platform situation calling platform_instance in the rcm master
instance:

::

	~$ curl http://public_ip_master/platform_instance/read

This time you could see the newly added robot in the no more empty list of
available robots. In this case you can see the state of the rcm robot and
verify if it's up and running:

    - ``connected`` is true when you switch on your provisioned robot. The
      agent on the robot has contacted the master to advertise its presence
      in the platform and the master agent recognise him as one the the robot
      provisioned by the user

    - ``paired`` is true when the robot is associated with a server (as its
      brain) and its role is defined (the service logic used for the robot
      at provisioning time is running)

.. _Create brains:

-------------
Create brains
-------------

In this section will speak about ``service logic``, its meaning and how to
create a new service logic.
Often the robot is a machine less computationally equipped but that needs
more power that he has in order to do all what he has to do. The robot is
associated with an additional brain at the boot time (after the installation
of the rcm robot agent and the robot provisioning) and and has a role in the
platform which means what he can do in the platform: this is defined by
the service logic.
In terms of robotic world the logic units are tied with the underlying
`ROS <http://wiki.ros.org/>`_ and can be of two types:

    - ``Node`` a process that perform computation and does one of the task
      of what the user want to let the robot do. Every task can involve more
      other nodes as sub task: to find more information about ROS nodes see the
      `documentation <http://wiki.ros.org/Nodes>`_. We define this underlying
      component as ``service node`` and can be accessed in the service logic
      as one of the two types of service items

    - ``Launcher`` a sort of aggregator, something that provide a common
      starting point for nodes in the ROS point of view. We define this as
      ``service launcher`` and can be accessed in the service logic as one of
      the two types of service items

The service logic provides a context in which the underlying elements can
run and live to give the feature or brain the robot needs.
There are many service items already available through the full installation
of the underlying layer but are not listed through the platform in the fiware
version: you can see them looking in the ROS environment or documentation to
see what the installation deploys into the machine.

The service logic is composed by 3 parts:

    - ``name`` the name that identifies the service logic into the platform

    - ``context`` the environment in which the elements of the service logic
      run and live. We call them ``service spaces`` and are a sort of containers
      in which the service items work. This is not configurable but every service
      logic instance and so every robot create and use one service space identified
      by the same name used to identify the rcm robot

    - ``list of service items`` the elements that define the behaviour of the
      robot and in terms of ROS point of view what is launched in the service
      logic context

Manage service logic
====================

There are three operations that can be done on a service logic:

    - create a new service logic

    - delete an old and no more used service logic

    - retrieve the set of available service logics

You can create a new service logic calling service_logic in the rcm master
instance:

::

	~$ curl -H 'Content-Type: application/json' -d '{"slg_name":"name_s",...}' http://public_ip_master/service_logic/provisioning

The important thing to remember in this operation is that you have to provide
2 of the 3 parts of the service logic we listed before at the end of the section
`Create brains`_.
The ``slg_name`` parameter is mandatory and as we said before identifies the
service logic in the platform: when you do the provisioning of the robot you have
to provide this name in ``pi_service_logic`` parameter and the platform starts an
instance of that service logic when the robot is turned on. Starting an instance
means only that in the context (what we called service space in the previous
section) will be launched all the nodes and launchers defined for this service
logic.
In the provisioning of the service logic you have to provide a complete list
of items you need so ``sn_list`` and ``sl_list`` should be added in json format.
You could have only nodes or only launchers so you can use an empty list for
the parameter you don't use.

``sn_list`` and ``sl_list`` stand for service node list and service launcher list.
All their elements follow the form of their type of element in the underlying layer
so

    - ``sn_package`` and ``sl_package`` will be the names of the service node
      (sn) or service launcher (sl) packages, the packages of the ROS nodes
      or launchers

    - ``sn_type`` will be the type of the service node, the name of the
      executable or python script representing the type of the node in ROS

    - ``sl_file_launcher`` will be the file name of the service launcher, the
      name of the scripting file representing the launcher in ROS

    - ``sn_name`` and ``sl_name`` will be the names assigned to the service
      node or the service launcher

    - ``sn_params`` will be the parameters passed to the service node or
      launcher

The only parameters that are specific to rcm platform are:

    - ``sn_side`` and ``sl_side``: they represent the side where a node or
      a launcher will be run. The meaning of this field is tied to the meaning
      of the context or service space: the service space is a logical container
      which represents the link between two machines, a server and a robot, and
      has a sort of manager or main component that in the underlying ROS is
      called ``roscore``. This component will be on server side by default but
      all the other node and launcher can run on server side or robot side.
      You have to decide where to launch the elements but remember that the
      additional brain and the machine more powerful should be the server and
      should be the preferred side where to launch more resource greedy
      processes

The information you pass to the platform is not verified so if you put
a not existing node into the service logic the result will be that the
platform will be unable to correctly start the robot using that service logic.
In any case the result of the service logic provisioning will be OK if the
syntax of the operation was right so be careful when you create a service
logic to provide the available items and correct parameters.

At any moment you can see the service logic that are available in the platform
and how is composed what you created looking at service_logic in the rcm master
instance:

::

    ~$ curl http://public_ip_master/service_logic/read
    ~$ curl http://public_ip_master/service_logic/read?slg_name=name_s_l

The first give you an overview of the service logic in the platform (those
created by default and those created by you) and the second give a more
detailed overview about a specified service logic referred by name.

The last operation you can do is the deletion of the service logic you created
if you are not satisfied or you want change something (no changes can be done,
so if you want add some nodes or change a launcher you used, you have to remove
the service logic and repeat the provisioning with the same name but with the
newly designed structure).
The deletion can be done calling service_logic in the rcm master instance:

::

    ~$ curl -H 'Content-Type: application/json' -d '{"slg_name":"name_s"}' http://public_ip_master/service_logic/r_provisioning

---------------------
Give the brain a body
---------------------

When you finished to design the brain for your robot you have to provide a
body to that brain and you do that when you do the provisioning of the
robot. All that you set to run in the service logic will be launched where you
asked when the robot is switched on after the robot provisioning.
You can check if all went well looking at platform_instance in the rcm master
instance:

::

	~$ curl http://public_ip_master/platform_instance/read

The service logic was good and the provisioning went well if the ``paired``
field becomes true. This change of state require some time so wait before considering
the operation a failure.
Even in the case of robot as it happens in service logic case, if you want to
change something about the robot you have to remove the robot and provide
again with the new values. If you change the name you have to change the name
in the configuration file of the rcm robot to match the name you newly provide.
In order to remove the robot you can call platform_instance in the rcm master
instance:

::

	~$ curl -H 'Content-Type: application/json' -d '{"pi_name":"name_r"}' http://public_ip_master/platform_instance/r_provisioning

------------------------------
Connecting to the Fiware world
------------------------------

In order to understand and provide the connection to the fiware world you
have to know that this link is done through firos and you need to put that
part in your custom service logic to do it.
During the master installation the wizard ask you if you want
to enter the fiware world and install the firos package (see
:doc:`i_and_a_guide`). If you require
that, firos, rcm_driver and robotics_msgs will be deployed in the ROS
workspace used by the rcm platform to run the underlying nodes and launchers.
You can see those 3 elements as service nodes needed to exchange information
between the robotic world and fiware world.
Rcm platform speaks to rcm_driver to tell firos what's happening in the
rcm platform and firos communicates those information to the fiware context
broker. Rcm_driver speaks to firos in the ROS environment using the language
specified in robotics_msgs.
All this explanation is intended to let you know that if you want to connect
with fiware in your custom made service logic you have to put those 3 nodes
in it. Moreover those 3 service nodes are deployed in the master and are
available only there, so when you create your service logic you must tell it
to run them on the server side.
If you do that when you turn on your robots they are notified in fiware
world and and an entity of each robot will be automatically available there.

------------------------
Robots in context broker
------------------------

Firos will perform a mapping between ROS and context broker following these rules:
    - Each robot will be an entity, its type will be ROBOT and its id the name of the robot.
    - The topics from the robot will be translated into attributes, its type will be
      the type of the topic and the name will be a slightly modified version of the
      name of the topic.
    - Each topic will have a timestamp named firostimestamp.
    - If you want to send a command from context broker to a robot, you must list the
      name of the attribute to be sent into the value of the attribute COMMAND (type COMMAND).
    
::

        {
            "type": "COMMAND",
            "name": "COMMAND",
            "value": [
                "pose"
            ]
        }

Firos whitelist
===============
Firos selects which topics will be mapped into context broker throught its whitelist, which
can be configured in the whitelist.json file. This file has the following format:

::

    "name_of_the_robot": {
        "publisher": ["list_of_topics_to_be_received_from_context_broker"],
        "subscriber": ["list_of_topics_to_be_sent_to_context_broker"],
    }

For example:

::

    "turtle\\w+": {
        "publisher": ["cmd_vel"],
        "subscriber": ["pose"]
    },
    "robot\\w+": {
        "publisher": ["cmd_vel.*teleop", ".*move_base/goal", ".*move_base/cancel"],
        "subscriber": [".*move_base/result"]
    }

Both the name of the robot and the name of the topics can be regular expresions.

Robots descriptions
===================

Robots may have some public files so users can understand some characteristics or even
use their devices. All the references contained in this file can be published on the
context broker; to do so, just configure the robotdescriptions.json file following this example:

::

    "turtle1": {
        "descriptions": [
            "http://wiki.ros.org/ROS/Tutorials/UsingRxconsoleRoslaunch",
            "http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes"
        ]
    }

This data will be inserted in the entity as follows:

::

            {
                "type": "DescriptionData",
                "name": "descriptions",
                "value": "http://wiki.ros.org/ROS/Tutorials/UsingRxconsoleRoslaunch||http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes"
            },
