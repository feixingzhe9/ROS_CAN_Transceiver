cmake_minimum_required(VERSION 2.8.3)
project(can_sub_powerboard)

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
  include/can_sub_powerboard
)

add_executable(can_sub_powerboard_node 
    src/can_sub_powerboard.cpp 
)
target_link_libraries(can_sub_powerboard_node
  ${catkin_LIBRARIES} 
)

#############
## Install ##
#############


#############
## Testing ##
#############

