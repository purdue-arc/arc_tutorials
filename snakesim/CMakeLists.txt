cmake_minimum_required(VERSION 3.0.2)
project(snakesim)

# find catkin so we can use our macros
find_package(catkin REQUIRED dynamic_reconfigure)

# install python module(s)
catkin_python_setup()

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  cfg/SnakeNode.cfg
)

# generate variables for installation
catkin_package()

#############
## Install ##
#############

# Mark executable python scripts and nodes for installation
catkin_install_python(PROGRAMS
  nodes/snake_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
