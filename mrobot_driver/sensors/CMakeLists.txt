cmake_minimum_required(VERSION 2.8.3)
project(sensors)

find_package(catkin REQUIRED COMPONENTS
  roslaunch
  rospy
  std_msgs
  mrobot_driver_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs mrobot_driver_msgs
  LIBRARIES ${PROJECT_NAME}
)

roslaunch_add_file_check(launch)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(
    PROGRAMS script/canMsg.py script/rfid.py script/sensor.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch"
)

