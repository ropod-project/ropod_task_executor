cmake_minimum_required(VERSION 2.8.6)

project(ropod_task_executor)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    roscpp
    ropod_ros_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    ropod_ros_msgs
)

add_executable(ropod_task_executor
    src/task_executor.cpp
)

target_link_libraries(ropod_task_executor
    ${catkin_LIBRARIES}
)
