cmake_minimum_required(VERSION 3.0.2)

# EDIT: name of package
project(nxt_connector)

# EDIT: list all <build_depend> and <depend> package dependencies
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  template_msg_pkg
  template_rospy_pkg
)

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# EDIT: list all executable python and shell scripts for installation
install(PROGRAMS
   scripts/*
   nodes/*
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
