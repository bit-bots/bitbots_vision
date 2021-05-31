cmake_minimum_required(VERSION 3.1)
project(bitbots_vision)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

find_package(catkin REQUIRED COMPONENTS
  image_transport
  rospy
  sensor_msgs
  std_msgs
  nav_msgs
  geometry_msgs
  humanoid_league_msgs
  bitbots_msgs
  trajectory_msgs
  message_generation
  dynamic_reconfigure
)

find_package(catkin COMPONENTS bitbots_docs)

catkin_python_setup()

FIND_PACKAGE (OpenCV REQUIRED)

#add dynamic reconfigure api
#find_package(catkin REQUIRED dynamic_reconfigure)
generate_dynamic_reconfigure_options(
  cfg/Vision.cfg
  #...
)


catkin_package(
  CATKIN_DEPENDS geometry_msgs humanoid_league_msgs image_transport message_runtime rospy sensor_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

set (CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall")


add_custom_target(
    extensions
    ALL
)

add_custom_command(
    TARGET extensions
    COMMAND python3 ${CMAKE_SOURCE_DIR}/src/python_extensions/setup.py ARGS build install "--user"
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/src/python_extensions/
)

enable_bitbots_docs()

if (CATKIN_ENABLE_TESTING)
    catkin_download_test_data(
      81-test_nagoya_game_d_00095.png
      http://data.bit-bots.de/bitbots_test/images/81-test_nagoya_game_d_00095.png
      MD5 6ce76435a0db5074550abe3b1bd32449
    )
    catkin_download_test_data(
      img_fake_cam_000800.PNG
      http://data.bit-bots.de/bitbots_test/images/img_fake_cam_000800.PNG
      MD5 3c0785e02afd7b1ac3da35cf78717c6f
    )
    set(ENV{CATKIN_BUILD_BITBOTS_VISION} ${CMAKE_CURRENT_BINARY_DIR})
    find_package(catkin REQUIRED COMPONENTS bitbots_test)
    enable_bitbots_tests()
endif()
