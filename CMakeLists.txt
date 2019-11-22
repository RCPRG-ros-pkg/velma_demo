cmake_minimum_required(VERSION 2.8)
project(velma_demo)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
)

catkin_package()

#catkin_install_python(PROGRAMS
#    scripts/hello_world
#    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
