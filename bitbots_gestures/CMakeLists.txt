cmake_minimum_required(VERSION 3.1)
project(bitbots_gestures)

find_package(catkin REQUIRED COMPONENTS
        rospy
        sensor_msgs
        std_msgs
        nav_msgs
        geometry_msgs
        roscpp
        rosconsole
        bitbots_docs
        message_generation
        )
catkin_python_setup()

find_package(OpenCV REQUIRED)

catkin_package(
        CATKIN_DEPENDS geometry_msgs rospy sensor_msgs std_msgs message_runtime
)


set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall")


enable_bitbots_docs()

