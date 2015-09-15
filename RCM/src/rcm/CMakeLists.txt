cmake_minimum_required(VERSION 2.8.3)
project(rcm)

find_package(catkin REQUIRED COMPONENTS
    rospy
    robotics_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime
)
