cmake_minimum_required(VERSION 2.8.6)

project(ropod_task_executor)

set(CMAKE_CXX_STANDARD 11)
include(/opt/ropod/ropod_common/ropodcpp/ftsm/CMakeLists.txt)

find_package(PkgConfig)
include(FindPkgConfig)
pkg_check_modules(LIBMONGOCXX REQUIRED libmongocxx)
pkg_check_modules(LIBBSONCXX REQUIRED libbsoncxx)


find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    roscpp
    ropod_ros_msgs
    actionlib
    elevator_navigation
    route_navigation
)

include_directories(
  include
  ${LIBMONGOCXX_INCLUDE_DIRS}
  ${LIBBSONCXX_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${LIBMONGOCXX_LIBRARY_DIRS}
  ${LIBBSONCXX_LIBRARY_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    ropod_ros_msgs
    elevator_navigation
    route_navigation
)

add_executable(ropod_task_executor
    src/task_executor.cpp
    src/task_executor_sim.cpp
    src/goto_recovery.cpp
    src/dock_recovery.cpp
    src/elevator_recovery.cpp
    src/action_recovery.cpp
    /opt/ropod/ropod_common/ropodcpp/ftsm/src/ftsm_base.cpp
    /opt/ropod/ftsm/cppftsm/src/ftsm.cpp
)
add_dependencies(ropod_task_executor ropod_ros_msgs_generate_messages_cpp)

target_link_libraries(ropod_task_executor
  ${LIBMONGOCXX_LIBRARIES}
  ${LIBBSONCXX_LIBRARIES}
  ${catkin_LIBRARIES}
  ${FTSM_LIBRARIES}
)

add_executable(execution_mockup
    mockup/execution_mockup.cpp
)
add_dependencies(execution_mockup ropod_ros_msgs_gencpp)

target_link_libraries(execution_mockup
  ${catkin_LIBRARIES}
)
