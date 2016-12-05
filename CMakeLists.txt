cmake_minimum_required(VERSION 2.8.3)
project(arl_controllers)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  controller_interface
  roscpp
  arl_interfaces
  arl_hw_msgs
  hardware_interface
  cmake_modules
  pluginlib
  control_toolbox
)

catkin_package(
  INCLUDE_DIRS
        include
#  LIBRARIES arl_controllers
#  CATKIN_DEPENDS controller_interface roscpp
  DEPENDS
        pluginlib
        roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(muscle_controller src/muscle_controller.cpp)

target_link_libraries(muscle_controller ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

