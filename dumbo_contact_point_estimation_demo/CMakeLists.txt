cmake_minimum_required(VERSION 2.8.3)
project(dumbo_contact_point_estimation_demo)

find_package(catkin REQUIRED COMPONENTS )

catkin_python_setup()

catkin_package(
)


install(PROGRAMS scripts/dumbo_contact_point_estimation_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)