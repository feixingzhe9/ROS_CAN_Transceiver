cmake_minimum_required(VERSION 2.8.3)
project(can_interface)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
   CATKIN_DEPENDS roscpp
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########

set (CMAKE_CXX_FLAGS "-std=c++11")
#set (CMAKE_CXX_STANDARD 11)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_library(${PROJECT_NAME} src/CanVCI.cpp)

set(EXTRALIB_BIN ${PROJECT_SOURCE_DIR}/lib/libcontrolcan.so)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${EXTRALIB_BIN}
  controlcan
)

#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY lib/
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
#############
## Testing ##
#############

