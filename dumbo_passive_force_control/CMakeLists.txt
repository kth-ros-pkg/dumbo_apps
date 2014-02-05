cmake_minimum_required(VERSION 2.8.3)
project(dumbo_passive_force_control)

find_package(catkin REQUIRED COMPONENTS roscpp brics_actuator control_msgs kdl_wrapper geometry_msgs tf)
find_package(Eigen REQUIRED)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS roscpp brics_actuator control_msgs kdl_wrapper geometry_msgs tf
  INCLUDE_DIRS
  LIBRARIES ${PROJECT_NAME} 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

add_executable(${PROJECT_NAME} src/dumbo_passive_force_control.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)