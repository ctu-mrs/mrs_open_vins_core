cmake_minimum_required(VERSION 3.5)
project(mrs_open_vins_core)

set(CATKIN_DEPENDENCIES
    cmake_modules
  )

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
  )

catkin_package(
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  )
