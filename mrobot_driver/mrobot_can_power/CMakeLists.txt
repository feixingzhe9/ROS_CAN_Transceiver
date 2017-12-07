cmake_minimum_required(VERSION 2.8.3)
project(mrobot_can_power)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  mrobot_driver_msgs
  roscan
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs 
)

###########
## Build ##
###########

set (CMAKE_CXX_FLAGS "-std=c++11")

include_directories(
  ${catkin_INCLUDE_DIRS}
  include/mrobot_can_power
)

add_executable(mrobot_can_power_node
    src/can_powerboard.cpp
)

target_link_libraries(mrobot_can_power_node
  ${catkin_LIBRARIES} 
)

#############
## Install ##
#############


#############
## Testing ##
#############

