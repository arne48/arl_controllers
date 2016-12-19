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
  INCLUDE_DIRS include
  LIBRARIES muscle_controller
  CATKIN_DEPENDS controller_interface roscpp
  DEPENDS pluginlib roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(muscle_controller src/muscle_controller.cpp)
add_dependencies(muscle_controller arl_hw_msgs_gencpp) 

target_link_libraries(muscle_controller ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)

    add_rostest_gtest(muscle_controller_test
            test/muscle_controller.test
            test/muscle_controller_test.cpp)
    target_link_libraries(muscle_controller_test muscle_controller ${catkin_LIBRARIES})
endif()

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

