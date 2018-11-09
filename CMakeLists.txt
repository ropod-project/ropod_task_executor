cmake_minimum_required(VERSION 2.8.6)

project(ropod_task_executor)

set(CMAKE_CXX_STANDARD 11)

find_package(PkgConfig)
include(FindPkgConfig)
pkg_check_modules(LIBMONGOCXX REQUIRED libmongocxx)
pkg_check_modules(LIBBSONCXX REQUIRED libbsoncxx)


find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    roscpp
    ropod_ros_msgs
)

include_directories(
  include
  /opt/ropod/ftsm/cppftsm/include
  /opt/ropod/ropod_common/ropodcpp/ftsm/include
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
)

add_executable(ropod_task_executor
    src/task_executor.cpp
    /opt/ropod/ropod_common/ropodcpp/ftsm/src/ftsm_base.cpp
    /opt/ropod/ftsm/cppftsm/src/ftsm.cpp
)
add_dependencies(ropod_task_executor ropod_ros_msgs_gencpp)

target_link_libraries(ropod_task_executor
  ${LIBMONGOCXX_LIBRARIES}
  ${LIBBSONCXX_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(execution_mockup
    mockup/execution_mockup.cpp
)
add_dependencies(execution_mockup ropod_ros_msgs_gencpp)

target_link_libraries(execution_mockup
  ${catkin_LIBRARIES}
)
