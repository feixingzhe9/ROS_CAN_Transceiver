cmake_minimum_required(VERSION 2.8.3)
project(roscan)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  mrobot_msgs
  can_interface
)

catkin_package(
   CATKIN_DEPENDS roscpp rospy std_msgs can_interface mrobot_msgs
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########
set (CMAKE_CXX_FLAGS "-std=c++11")

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_library(${PROJECT_NAME} src/can_transceiver.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(can_transceiver src/can_transceiver.cpp)
add_dependencies(can_transceiver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(can_transceiver
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS can_transceiver
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

#############
## Testing ##
#############

#add_executable(long_frame_test src/can_long_frame_test.cpp)
