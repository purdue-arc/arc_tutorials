# General Information
This package contains example rosnodes and implements custom messages defined in
example_msgs
## Nodes
### state_publisher_node
This is a very basic node that exists entirely inside a main function. It
periodically sends out example_msgs/State messages

### clock_face_node
This is a slightly more complex node that exists inside a main function and uses
another class to offload some functionality. It's functionality is pretty
useless, it publishes example_msgs/ClockFace messages that show the current EST
time using the hands on an analog clock

### clock_face_converter_node
This node has a very bare main function, and the functionality exists within a
separate class. It converts the example_msgs/ClockFace messages to a
visualization_msgs/Marker that can be viewed within Rviz

## Launch Files
### state_publisher.launch
Launches state_publisher_node

### clock_face.launch
Launches clock_face_node

### clock_face_converter.launch
Launches clock_face_converter_node

### combined_clock.launch
Launches clock_face_node, clock_face_converter_node, and rviz

## Parameter Files
### clock_face_converter_params.yaml
Contains parameters for clock_face_converter_node

## Rviz Files
### clock.rviz
Set up to view output of clock_face_converter_node

# Setup Instructions
1. Move into the directory where you want to have your workspace  
ex: `cd ~/src`

2. Create a workspace and source directory  
ex: `mkdir -p example_ws/src`  
_Note: the name of the workspace directory does not matter (*_ws is typical),
but you must have a folder named src inside. You can also add these packages
into an existing workspace, but creating a new one may be cleaner._

3. Clone the necessary repos  
ex: `git clone https://github.com/purdue-arc/example.git && git clone
https://github.com/purdue-arc/example_msgs.git`

4. Build the packages
ex: `cd .. && catkin build`  
_Note: `catkin_make` can also be used, but catkin build provides a more
descriptive command line interface and also builds packages in parallel._

5. Source the packages  
ex: `source devel/setup.bash`  
_Note: This must be done in every new terminal window that you want to use these
packages in. It may be convenient to add it to your bashrc, which is a script
called every time you open a new one. You can add it via the following command:
`echo 'source ~/src/example_ws_2/devel/setup.bash --extend' >> ~/.bashrc`
The `--extend` flag is used so multiple workspaces can be sourced simultaneously._

# Operation Guide
## Launching Nodes
You can launch nodes either via rosrun or roslaunch. Rosrun starts a single node
from a specified package, while roslaunch  can launch multiple nodes simultaneously.
This can be incredibly useful for repeatedly launching a whole system of nodes,
but you must have a launch file set up in order to use it. Launch files can also
load parameters, remap topics, and perform basic if-else logic. Rosrun can do
many of the same things via the command line interface. Roslaunch will also start
a ROS master if one is not already running. Rosrun requires you to have one
already running.

### Rosrun Syntax
Start a ROS master if not already running: `roscore`  
Run a node: `rosrun <pkg name> <node name>`  
ex: `rosrun example state_publisher_node`

### Roslaunch Syntax
Launch a launch file: `roslaunch <pdk name> <launch file name>`  
ex: `roslaunch example combined_clock.launch`

## Debugging
There are several command line and GUI tools to help in debugging a network of
ROS nodes

### Roswtf
This is a command line tool that can do some basic automated debugging. Sometimes its useful, other times it isn't.  
ex: `roswtf`

### Rostopic
This is a command line tool used to interface with ROS topics and messages.  
ex: `rostopic list`
ex: `rostopic echo /<topic name>`

### Rosnode
This is a command line to interface with ROS nodes.  
ex: `rostopic list`  
ex: `rostopic info <node name>`

### Rqt_graph
This is a GUI tool that displays the network of ROS nodes and topics. Be sure to
check out the various visibility options in the top left.  
ex: `rosrun rqt_graph rqt_graph`

# Coding Challenges
To get better at working in ROS, there are a few coding challenges you can do
with this package. Create a new branch for each one. In the future, sample
solutions will be available in branches named `challenge-*`, where the text is
the title of the challenge from the below list. They are roughly arranged in
order of increasing difficulty.

1. Combined Launch File  
Create a launch file that combines the functionality of state_publisher.launch
and combined_clock.launch in a single file, using include tags.

2. Timer Conversion  
Modify state_publisher_node to operate off a Timer instead of a Rate object.
Parameterize the publish rate with a parameter you pick, and set this parameter
in the launch file. Ensure it is robust to the parameter not being set. Measure
the output rate using `rostopic hz /state` for when the parameter is set and for
when it is defaulted to.

3. Cyclical State Publisher  
Modify the state publisher node to cycle through the various states defined within
the State message. The rate at which it cycles should be parameterized via a
parameter you pick and set in the launch file.

4. State Publisher Timeout
Modify the state publisher node to slowly worsen its published state in response
to a subscribed Empty message. For example, it receives the Empty message (on a
topic you pick) and is GOOD for 1 second, BAD for the next second ... stopping at
ABYSMAL. Make the time duration between transitions parameterized, and publish
the Empty message via the command line rostopic interface.

5. Clock Face Numerals  
Create a new node that publishes Marker messages on topic(s) you pick. These
messages must have the proper text and position in order to look like a clock
face when viewed in Rviz. Modify the Rviz file to display these messages and
modify the combined_clock.launch file to launch it. For extra difficulty,
include a parameter to set if the numbers are roman numerals or not.

6. ClockFace Message Decoding  
The /face topic publishes ClockFace messages with the current time (in a mostly
useless format). Create a new node that subscribes to this and publishes the time
(calculated through the ClockFace message) in a useful format in a String message
on a topic you pick. Do not modify combined_clock.launch, but instead launch
this node directly from the command line with rosrun.
