cmake_minimum_required(VERSION 2.8.3)
project(example)

# Compile as C++11, supported in ROS Kinetic and newer
# alternative is C++03
# add_compile_options(-std=c++11)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  example_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
)

# System dependencies are found with CMake's conventions
# remove if not using boost
find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
# The catkin_package macro generates cmake config files for your package
# Declare things to be passed to dependent projects
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    example_msgs
    geometry_msgs
    visualization_msgs
    std_msgs
)

###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
include
${Boost_INCLUDE_DIRS}
${catkin_INCLUDE_DIRS}
)

# Declare a C++ executable
add_executable(state_publisher_node
  src/state_publisher_node.cpp
)
# This line is required since we depend upon messages in same ws
add_dependencies(state_publisher_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(state_publisher_node ${catkin_LIBRARIES})

# Declare a C++ executable
add_executable(clock_face_node
  src/clock_face_node.cpp
  src/ClockFace.cpp
)
# This line is required since we depend upon messages in same ws
add_dependencies(clock_face_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(clock_face_node ${catkin_LIBRARIES})

# Declare a C++ executable
add_executable(clock_face_converter_node
  src/clock_face_converter_node.cpp
  src/ClockFaceConverter.cpp
)
# This line is required since we depend upon messages in same ws
add_dependencies(clock_face_converter_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(clock_face_converter_node ${catkin_LIBRARIES})

# Declare a C++ executable
add_executable(clock_face_decoder_node
  src/clock_face_decoder_node.cpp
  src/ClockFaceDecoder.cpp
)
# This line is required since we depend upon messages in same ws
add_dependencies(clock_face_decoder_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(clock_face_decoder_node ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# Mark executables for installation
# See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS state_publisher_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark executables for installation
install(TARGETS clock_face_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark executables for installation
install(TARGETS clock_face_converter_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark executables for installation
install(TARGETS clock_face_decoder_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark launch files for installation
install(FILES
  launch/state_publisher.launch
  launch/clock_face.launch
  launch/clock_face_converter.launch
  launch/combined_clock.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## Mark param files for installation
install(FILES
  params/clock_face_converter_params.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/params
)

## Mark rviz files for installation
install(FILES
  rviz/clock.rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)
