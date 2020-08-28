# snakesim
This package contains a snake game written in Python with ROS bindings.
It will be used as an example in order to teach ROS by writing a control
algorithm for the snake. It has been designed to have a somewhat similar
interface to the [turtlesim](http://wiki.ros.org/turtlesim) package.

## Snake Game
Consider familiarizing yourself with the traditional game of snake that this is
based off of. You can play it on [Google](https://www.google.com/search?q=play+snake).

In the classic game, you control a snake that travels on a discrete grid and
can chose to turn left, turn right, or continue straight on each new square.
The goal is to eat as many apples as possible, and with each apple the length of
the snake will increase. The only ways to lose are by hitting the edge of the
environment or colliding with yourself.

The game included in this package is very similar but operates in a continuous
environment. The position of the snake, the heading of the snake, and the timing
of commands are executed are all continuous.\* It is controlled with a linear
velocity (must be positive) and angular velocity. Just as with the classic game,
you lose by colliding with the edge of the world or self intersecting. You also
increase your score and number of segments by reaching the goal.

\*or at least high enough resolution that it appears continuous :)

## Interface
### Subscribed Topics
`snake/cmd_vel` (geometry_msgs/Twist)

The linear and angular command velocity for the head of the snake. The snake
will execute a velocity command for 1 second then time out. `Twist.linear.x` is
the forward velocity and `Twist.angular.z` is the angular velocity.

### Published Topics
`snake/pose` (geometry_msgs/PoseArray)

An array of the poses of the segments of the snake. The XY positions correspond
directly to the position of the segment. The Z position can be ignored. The
orientation is a quaternion representing the yaw of the segment. For the head,
this is the current heading. For body segments, this points to the segment
ahead. Yaw is 0 pointing along the X axis and increases counter-clockwise.

`snake/goal` (geometry_msgs/PointStamped)

The XY position of the goal. The Z value can be ignored.

`snake/score` (std_msgs/Int32)

The current score in the game. In the current implementation, this is simply
the number of segments.

`snake/active` (std_msgs/Bool)

A Boolean value describing if the snake is still running.
It is true (active) initially, but will turn false (inactive) if a loss
condition like self collision occurs.

### Services
`snake/reset` (std_srvs/Empty)

Resets the game to the start configuration. This includes the pose of the
snake, position of the goal, score, and active flag.

## Instructions
### Building and Sourcing the Package
Even though this is written in Python, not C++, you still need to use catkin
in order to build the package so that you can source it. If you have a catkin
workspace set up as described in the [ros tutorials](http://wiki.ros.org/catkin/Tutorials/create_a_workspace),
simply clone this repository into `~/catkin_ws/src/` and build it. Building may
be done by calling `catkin_make` or `catkin build` from the root workspace
directory. After the package is built, source it by running `source devel/setup.bash`
from the root workspace directory. Note that you must source this file from
every new terminal you intend to run this code from.

### Launching
There are several launch files included. The most basic one to get up and
running can be executed with `roslaunch snakesim snakesim.launch`.

You can also run `snake_rviz.launch` if you'd prefer to use rviz to visualize
and control the snake instead of pygame and rqt_robot_steering.

The file `snake.launch` can be used to run the game headless.
