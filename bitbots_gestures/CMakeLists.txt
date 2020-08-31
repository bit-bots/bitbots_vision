cmake_minimum_required(VERSION 3.1)
project(bitbots_gestures)

find_package(catkin REQUIRED COMPONENTS
        image_transport
        rospy
        sensor_msgs
        std_msgs
        nav_msgs
        geometry_msgs
        roscpp
        rosconsole
        bitbots_docs
        message_generation
        cv_bridge
        )
catkin_python_setup()

add_message_files(
        FILES
        HumanPoseEstimation.msg
        HumanPoseArray.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
)

find_package(InferenceEngine REQUIRED)


find_package(OpenCV REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS geometry_msgs image_transport rospy sensor_msgs std_msgs roscpp message_runtime cv_bridge
)

set(SOURCES
        src/human_pose.cpp
        src/human_pose_estimator.cpp
        src/node.cpp
        src/peak.cpp
        )

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${InferenceEngine_INCLUDE_DIRS}
)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall")

add_executable(node ${SOURCES})
target_link_libraries(node ${catkin_LIBRARIES} ${InferenceEngine_LIBRARIES})

enable_bitbots_docs()

