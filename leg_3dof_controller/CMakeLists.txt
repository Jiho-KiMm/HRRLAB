cmake_minimum_required(VERSION 3.0.2)
project(leg_3dof_controller)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  roscpp
  roslib
  rospy
  sensor_msgs
  urdf
  tf
  xacro
  tf_conversions
  gazebo_ros
  controller_manager
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

# Pinocchio

find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

find_package(gazebo REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS eigen_conversions geometry_msgs sensor_msgs tf tf_conversions
 DEPENDS gazebo_ros roscpp rospy std_msgs Boost pinocchio
)

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
  ${Boost_INCLUDE_DIR} 
  ${catkin_INCLUDE_DIRS} 
  ${GAZEBO_INCLUDE_DIRS} 
  ${PCL_LIBRARY_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
  ${pinocchio_INCLUDE_DIRS}
  )

link_directories(
    ${pinocchio_LIBRARY_DIRS}
)

include_directories(
  include
  include/Tools
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(include/Tools)

add_library(${PROJECT_NAME}
  include/leg_3dof_controller.h
  src/leg_3dof_controller.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${pinocchio_LIBRARIES})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_executable(leg_3dof_node src/leg_3dof_controller_node.cpp)

add_dependencies(leg_3dof_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} Tools)

target_link_libraries(leg_3dof_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

