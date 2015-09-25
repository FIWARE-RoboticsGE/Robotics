#<a name="top"></a>Robotics

* [Introduction](#introduction)
* [GEi overall description](#gei-overall-description)
* [Install and run](#install-and-run)
* [API Reference Documentation](#api-reference-documentation)
		  
## Introduction

Considering the architecture of this platform we think that Robotics cannot be distributed
as a [Docker](https://www.docker.com/) image as required by [FIWARE](http://www.fiware.org).
The main reason for this kind of choice is because docker is a platform for distributed 
application and is useless put a platform of distributed application related to robot
into a container of something else that does the same although for a wider range of 
applications. Furthermore having our components in docker would mean change the 
architecture of our product to provide the management of docker containers instead of
ROS components and put those ROS elements with their infrastructure in a container for
each of them. This is out of the scope of our project and so we provide this kind of 
distribution packages.

[Top](#top)

## Distribution structure

This document is a simple note to explain how the dist folder is composed. Really follow 
the same idea at the base of the `create_robotics_pkg` script that creates the distribution 
packages.

-   `all` contains a single package that is composed by a master rcm platform with all
    the components embedded into it
-   `platform` contains both the packages of rcm platform (2 types of rcm platform exist: 
    master and robot) but they are without the components
-   `components` contains all the components packages

[Top](#top)

## Install and run

Install documentation for Robotics can be found at [the Installation and Administration Guide](../docs/i_and_a_guide.rst).
How to run Robotics can be found at [the Installation and Administration Guide](../docs/i_and_a_guide.rst).

[Top](#top)

## API Reference Documentation

* [RCM](http://docs.rdapi.apiary.io) (Apiary)
* [Firos](http://docs.firos.apiary.io) (Apiary)

[Top](#top)

