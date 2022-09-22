cmake_minimum_required(VERSION 2.8.3)
project(kimm_phri_panda_husky)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
LIST( APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}" )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
  franka_hw
  franka_gripper
  geometry_msgs
  hardware_interface
  pluginlib
  realtime_tools
  roscpp
  rospy
  roslib
  tf
)

find_package(Eigen3 REQUIRED)
find_package(Franka 0.5.0 REQUIRED)
find_package(kimm_hqp_controller REQUIRED)
find_package(kimm_joint_planner_ros_interface REQUIRED)
find_package(kimm_se3_planner_ros_interface REQUIRED)
find_package(kimm_path_planner_ros_interface REQUIRED)
find_package(kimm_object_estimation REQUIRED)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    controller_interface
    dynamic_reconfigure
    franka_hw
    geometry_msgs
    hardware_interface
    pluginlib
    realtime_tools
    tf
    roscpp
  DEPENDS Franka
)
###########
## Build ##
###########

## Franka hqp ctrl
SET(LIBRARY_NAME phri_hqp)
SET(${LIBRARY_NAME}_HEADERS
  include/kimm_phri_panda_husky/panda_husky_hqp.h  
)
SET(${LIBRARY_NAME}_SOURCES
  src/panda_husky_hqp.cpp  
)
ADD_HEADER_GROUP(${LIBRARY_NAME}_HEADERS)
ADD_SOURCE_GROUP(${LIBRARY_NAME}_SOURCES)
ADD_LIBRARY(${LIBRARY_NAME} SHARED ${${LIBRARY_NAME}_SOURCES} ${${LIBRARY_NAME}_HEADERS})
TARGET_INCLUDE_DIRECTORIES(${LIBRARY_NAME} PUBLIC $<INSTALL_INTERFACE:include> ${kimm_hqp_controller_INCLUDE_DIRS} ${kimm_joint_planner_ros_interface_INCLUDE_DIRS} ${kimm_se3_planner_ros_interface_INCLUDE_DIRS} ${kimm_path_planner_ros_interface_INCLUDE_DIRS} ${kimm_object_estimation_INCLUDE_DIRS}) 
TARGET_LINK_LIBRARIES(${LIBRARY_NAME} PUBLIC ${kimm_hqp_controller_LIBRARIES} ${kimm_object_estimation_LIBRARIES})

## Franka Controller for Real Robot
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Franka_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/phri_real.cpp
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  ${PROJECT_NAME}_gencpp
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
  ${LIBRARY_NAME}
)

## Franka Controller for Simulation Robot
add_executable(${PROJECT_NAME}_simulation src/phri_simul.cpp)
target_link_libraries(${PROJECT_NAME}_simulation
  ${catkin_LIBRARIES}
  ${LIBRARY_NAME}
  -lpthread
)
add_dependencies(${PROJECT_NAME}_simulation ${PROJECT_NAME}_gencpp)


#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES ${PROJECT_NAME}_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
catkin_install_python(
  PROGRAMS scripts/interactive_marker.py scripts/move_to_start.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_dyros_mobile_manipulator_controller.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
