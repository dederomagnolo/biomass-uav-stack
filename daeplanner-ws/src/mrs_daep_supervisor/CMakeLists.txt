cmake_minimum_required(VERSION 3.0.2)
project(mrs_daep_supervisor)

find_package(catkin REQUIRED COMPONENTS
  rospy
  nav_msgs
  geometry_msgs
  octomap_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Instala os scripts Python (rosrun/roslaunch)
catkin_install_python(PROGRAMS
  scripts/pose_relay.py
  scripts/octomap_relay.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Instala arquivos de launch
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
