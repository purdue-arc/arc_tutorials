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

Here is a simplified file structure for packages:
```
example_package/
│
└───src/
│   │
│   └───example_package/
│
└───nodes/
│
└───launch/
│
└───msg/
│
└───srv/
│
└───scripts/
│
└───CMakeLists.txt
│
└───package.xml
```

`src/example_package/` is where a majority of the magic happens.
This is where source files will be stored.

`nodes/` contains node types, which we will cover in the next section.

`launch/` contains launch files.

`msg/` contains message types.

`src/` contains service types.

`scripts/` contains executable scripts.

`CMakeLists.txt` is a build file used by your catkin workspace.

`package.xml` is a manifest that contains general package information.

## Creating a New Package
We will start by moving into the `src/arc_tutorials/` directory of our 
catkin workspace. As a reminder, you should still be within the container environment and 
have the following file path:

`~/catkin_ws/src/arc_tutorials/`

Now we will use this command to create our new package:

```
catkin_create_pkg snake_controller rospy
```
Looking at this command, `snake_controller` is the name of package and
all the parameters that follow are other packages that `snake_controller` 
depends on.

Lets now move into our newly created package:

```
cd snake_controller/
```
Upon printing the directory contents, you can see have three items within
our package:
- CMakeLists.txt
- package.xml
- `src/`








