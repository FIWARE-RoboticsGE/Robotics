cmake_minimum_required(VERSION 2.8.3)
project(rcm)

find_package(catkin REQUIRED COMPONENTS
    rospy
#    std_msgs
#    message_generation
    robotics_msgs
)

#add_message_files(
#    FILES
#    CB_Event.msg
#    Robot_Event.msg
#)

#add_service_files(
#    FILES
#    FIROS_Info.srv
#)

#generate_messages(
#    DEPENDENCIES
#    std_msgs
#)

catkin_package(
    CATKIN_DEPENDS message_runtime
)
