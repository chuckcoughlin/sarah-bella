cmake_minimum_required(VERSION 2.8.3)
project(common)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
)

catkin_package(
  CATKIN_DEPENDS
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

