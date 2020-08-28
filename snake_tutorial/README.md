# snake_tutorial
This package contains a very simple controller for the snakesim package. The
snakesim package emulates the traditional game of [snake](https://www.google.com/search?q=play+snake),
except it occurs in a continuous environment. It has ROS bindings, which is how
this package will interact with it. For more information, refer to the `README.md`
file included in the snakesim pacakge.

This package also includes a detailed tutorial on how to get started with ROS by
running the snakesim package with both manual control and autonomous control. It
provides line by line explanation on how to write the controllers included in
this package and also has resources for an open-ended project to improve upon
the basic controllers. Refer to the `docs` subfolder for this content.

## Nodes
### snake_heading_controller
This node uses a [bang-bang controller](https://en.wikipedia.org/wiki/Bang%E2%80%93bang_control)
to modulate the snake's input angular velocity such that the heading tracks a goal.
It also passes through a linear velocity in order to create the Twist message.

#### Subscribed Topics
`controller/heading` (std_msgs/Float64)

The desired heading for the snake. This is given as positive rotation about the
Z axis in radians.

`controller/velocity` (std_msgs/Float64)

The desired linear velocity value to be passed through into the output Twist
message. This is given in length units per second.

`snake/pose` (geometry_msgs/PoseArray)

The pose of the snake. This is used to determine the heading.

#### Published Topics
`snake/cmd_vel` (geometry_msgs/Twist)

The commanded velocity of the snake. The linear X value is passed through from
the subscriber, and the linear Z value is determined by the controller.

#### Services
None.

### snake_position_controller
This node uses a simple geometric calculation in order to determine the heading
to face a goal at the current position. It works well enough to control the
snake to reach a desired position, but it does not factor in the turning radius,
self-collisions, or wall-collisions at all.

#### Subscribed Topics
`controller/goal` (geometry_msgs/Point)

The desired goal position for the snake.

`snake/pose` (geometry_msgs/PoseArray)

The pose of the snake. This is used to determine the head position.

#### Published Topics
`controller/heading` (geometry_msgs/Float64)

The required heading to face the goal at the snake's current position. This is given as positive rotation about the Z axis in radians.

#### Services
None.

### snake_goal relay
This node republishes the goal topic from the game to the controller. This is
required due to the difference in topic types.

#### Subscribed Topics
`snake/goal` (geometry_msgs/PointStamped)

The current goal location as reported by the game.

#### Published Topics
`controller/goal` (geometry_msgs/Point)

The goal location data to be fed into the controller.

#### Services
None.

## Instructions
See the the `docs` subfolder for detailed instructions on how to get started.
