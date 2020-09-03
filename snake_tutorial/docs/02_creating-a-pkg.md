# Creating the Controller Package
Up to this point, we have explored high-level ROS concepts and ran the
Snake game example controller. We will now go through practical concepts
related to development within ROS.

## ROS File Structure
Let's take a deeper dive into our project file structure.

Software in ROS is organized in `packages`. Packages contain our nodes, 
launch files, messages, services and more. When working on a project, you may have multiple
packages all responsible for various tasks. For instance, `snakesim/` and
`snake_tutorial/` are both examples of packages.

Packages are often managed within a catkin workspace, which is what you 
setup as part of your ARC development environment.

The simplified file structure for a workspace is as follows:
```
catkin_ws/
│
└───src/
│   └───example_package/
│   └───another_package/
│
└───devel/
|   └─setup.bash
|
└───build/
|
└───logs/
|
└───install/

```
`src/` is where the source code for all packages are stored. This commonly
contains repositories tracked by Git.

`build/` and `logs/` are created when building your code. We won't go into any
detail on these.

`devel/` and `install/` contain executables and bash files for setting up your
environment. It is optional to have an `install/` directory; we won't be using
one in this tutorial. There are a few distinctions between the two that we won't
get into.

Each time you open your workspace in a new terminal you will need to source this
setup file by running:
```bash
source devel/setup.bash
```

There is a way to do that automatically, which is covered at the end of this
document.

Here is a simplified file structure for Python ROS packages (C++ is different):
```
example_package/
│
└───nodes/
│
└───launch/
│
└───CMakeLists.txt
│
└───package.xml
```

`nodes/` contains node Python files for running ROS nodes. You'll end up making
several of these when following the tutorial.

`launch/` contains launch files. These can launch one or more nodes and contain
logic.

`CMakeLists.txt` is a build file used by your catkin workspace.

`package.xml` is a manifest that contains general package information.

## Creating a New Package
We will start by moving into the `src/` directory of our 
catkin workspace. As a reminder, you should still be within the container environment and 
have the following file path:

`~/catkin_ws/src/`

Now we will use this command to create our new package:

```bash
catkin_create_pkg snake_controller rospy geometry_msgs std_msgs snakesim tf python-angles
```
Looking at this command, `snake_controller` is the name of package and
all the parameters that follow are other packages that `snake_controller` 
depends on. Don't worry about what those packages are right now, you'll learn
about them when completing the tutorials.

Lets now move into our newly created package:

```bash
cd snake_controller/
```
Upon printing the directory contents, you can see have three items within
our package:
- `CMakeLists.txt`
- `package.xml`
- `src/`

In order to make things better set up for a purely Python ROS package, we're
going to modify the `CMakeLists.txt` and `package.xml`. We will also be making
a few directories. If you aren't comfortable making directories on the
commandline, you can do these steps in an IDE like VS Code.

## Modifying Package.xml
The `package.xml` file exists to capture some basic information about your
package, such as the author, maintainer, license, and any dependencies that your
package has. If we open the existing one, you can see it has a lot of
information, and some comments about how to edit it:
```xml
<?xml version="1.0"?>
<package format="2">
  <name>snake_controller</name>
  <version>0.0.0</version>
  <description>The snake_controller package</description>

  <!-- One maintainer tag required, multiple allowed, one person per tag -->
  <!-- Example:  -->
  <!-- <maintainer email="jane.doe@example.com">Jane Doe</maintainer> -->
  <maintainer email="jamesb@todo.todo">jamesb</maintainer>


  <!-- One license tag required, multiple allowed, one license per tag -->
  <!-- Commonly used license strings: -->
  <!--   BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, LGPLv3 -->
  <license>TODO</license>


  <!-- Url tags are optional, but multiple are allowed, one per tag -->
  <!-- Optional attribute type can be: website, bugtracker, or repository -->
  <!-- Example: -->
  <!-- <url type="website">http://wiki.ros.org/snake_controller</url> -->


  <!-- Author tags are optional, multiple are allowed, one per tag -->
  <!-- Authors do not have to be maintainers, but could be -->
  <!-- Example: -->
  <!-- <author email="jane.doe@example.com">Jane Doe</author> -->


  <!-- The *depend tags are used to specify dependencies -->
  <!-- Dependencies can be catkin packages or system dependencies -->
  <!-- Examples: -->
  <!-- Use depend as a shortcut for packages that are both build and exec dependencies -->
  <!--   <depend>roscpp</depend> -->
  <!--   Note that this is equivalent to the following: -->
  <!--   <build_depend>roscpp</build_depend> -->
  <!--   <exec_depend>roscpp</exec_depend> -->
  <!-- Use build_depend for packages you need at compile time: -->
  <!--   <build_depend>message_generation</build_depend> -->
  <!-- Use build_export_depend for packages you need in order to build against this package: -->
  <!--   <build_export_depend>message_generation</build_export_depend> -->
  <!-- Use buildtool_depend for build tool packages: -->
  <!--   <buildtool_depend>catkin</buildtool_depend> -->
  <!-- Use exec_depend for packages you need at runtime: -->
  <!--   <exec_depend>message_runtime</exec_depend> -->
  <!-- Use test_depend for packages you need only for testing: -->
  <!--   <test_depend>gtest</test_depend> -->
  <!-- Use doc_depend for packages you need only for building documentation: -->
  <!--   <doc_depend>doxygen</doc_depend> -->
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>geometry_msgs</build_depend>
  <build_depend>python-angles</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>snakesim</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>tf</build_depend>
  <build_export_depend>geometry_msgs</build_export_depend>
  <build_export_depend>python-angles</build_export_depend>
  <build_export_depend>rospy</build_export_depend>
  <build_export_depend>snakesim</build_export_depend>
  <build_export_depend>std_msgs</build_export_depend>
  <build_export_depend>tf</build_export_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>python-angles</exec_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>snakesim</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>tf</exec_depend>


  <!-- The export tag contains other, unspecified, tags -->
  <export>
    <!-- Other tools can request additional information be placed here -->

  </export>
</package>
```

We're going to go ahead and remove a lot of the stuff that we don't need:
```xml
<?xml version="1.0"?>
<package format="2">
  <name>snake_controller</name>
  <version>0.0.0</version>
  <description>The snake_controller package</description>

  <!-- One maintainer tag required, multiple allowed, one person per tag -->
  <!-- Example:  -->
  <!-- <maintainer email="jane.doe@example.com">Jane Doe</maintainer> -->
  <maintainer email="jamesb@todo.todo">jamesb</maintainer>


  <!-- One license tag required, multiple allowed, one license per tag -->
  <!-- Commonly used license strings: -->
  <!--   BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, LGPLv3 -->
  <license>TODO</license>

  <!-- Author tags are optional, multiple are allowed, one per tag -->
  <!-- Authors do not have to be maintainers, but could be -->
  <!-- Example: -->
  <!-- <author email="jane.doe@example.com">Jane Doe</author> -->
  
  <buildtool_depend>catkin</buildtool_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>python-angles</exec_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>snakesim</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>tf</exec_depend>
</package>
```

Go ahead and make a few changes to personalize it:
- add a version number if you want
- add yourself as the maintainer
- add a license if you want
- add yourself as an author

You should end up with something like this:
```xml
<?xml version="1.0"?>
<package format="2">
  <name>snake_controller</name>
  <version>1.0.0</version>
  <description>A basic controller for the snakesim package</description>

  <maintainer email="pete@purdue.edu">Purdue Pete</maintainer>
  <license>BSD 3 Clause</license>
  <author email="pete@purdue.edu">Purdue Pete</author>
  
  <buildtool_depend>catkin</buildtool_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>python-angles</exec_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>snakesim</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>tf</exec_depend>
</package>
```

## Modifying CMakeLists.txt
The `CMakeLists.txt` is similar to `package.xml` in that it contains a lot of
templated information for you to go in and edit. For a C++ package, it is very
important, rather complicated, and will be used to build the code. For a purely
Python package, it is actually really simple. It is only needed to make your
code compatible with the catkin build system.

If you open the existing one, you should see something like this:
```cmake
cmake_minimum_required(VERSION 3.0.2)
project(snake_controller)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  python-angles
  rospy
  snakesim
  std_msgs
  tf
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES snake_controller
#  CATKIN_DEPENDS geometry_msgs python-angles rospy snakesim std_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/snake_controller.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/snake_controller_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_snake_controller.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
```

Once we remove everything we don't need, it is pretty short:
```cmake
cmake_minimum_required(VERSION 3.0.2)
project(snake_controller)

# Find catkin macros
find_package(catkin REQUIRED)

# generates cmake config files and set variables for installation
catkin_package()
```

Optionally, you can leave the install and test section in there, but we won't be
using it for this tutorial.

## Creating the Package Directory Structure
We're going to remove the existing `src/` directory, which is commonly used for
C++ code or Python modules (we are making Python nodes, which are different).

Instead, make the directory structure that we talked about earlier:
```
snake_controller/
│
└───nodes/
│
└───launch/
│
└───CMakeLists.txt
│
└───package.xml
```

## Sourcing the Workspace Automatically
Remember how we said that you need to source the workspace for every new
terminal window that you open? The command looks like this:
```bash
source devel/setup.bash
```

Thankfully, there is a way to make that happen automatically! If you set up WSL2
by following our guide, you may remember modifying a script called the `bashrc`.
This is a script that is run every time you open a new terminal. If we put that
source command at the end of it, then we don't need to worry about running it
manually for every new terminal we open. You can add that command by running
the following in your shell:
```bash
echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc
```

We can now run that to make it take effect in our current terminal:
```bash
source ~/.bashrc
```

However, if you are using the ARC development environment, this edit was done
inside a volatile Docker container. This means that as soon as you exit the
container, this edit will be lost permenantly. We made a note about committing
changes in Docker, and this is a good time to do that.

This is the command, which needs to be run on your _host_ machine. You'll want
to open up a new terminal, rather than close the one you are working in.
```bash
docker commit arc-dev arc-dev
```

Essentially, we've just taken the current running version of our Docker
container, with our edits, and saved it as the Docker image that we will run
everytime we restart it. You can safely exit the container now, and every time
you restart it through the `docker-run.sh`, it will source our workspace
automatically in every new terminal.

## Final Notes
Congratulations, you just finished creating a ROS package! In the following
documents, we'll build three nodes that will allow your new package to control
the snake game that you ran earlier.
