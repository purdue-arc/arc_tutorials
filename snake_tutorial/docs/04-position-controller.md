# Position Controller
## Overview
Our snake is now controlled by heading. Why don't we try to extend it another
layer and control with with position? That seems useful if we want to tell it
to chase the goal or give it a series of waypoints.

We'll create a closed loop controller like last time. Our inputs will be the
current position of the snake's head and the commanded position. Our output will
be the required heading to reach that positition.

## Creating the Program
This will be just like the last program. Let's call it
`snake_position_controller` and put it in the `nodes` folder like last time.
Make sure to make it executable!

Again, we'll start the program by writing a shebang and docstring. The basic
groundwork for the file will be really similar to last time too. See if you can
write out the `__main__` check, declare a class, and prototype all the methods
our class will need by writing a docstring then using `pass`.

You should have gotten something like this:
```python
#!/usr/bin/env python
"""Node to control the position of the snake.

License removed for brevity
"""

class SnakePositionController(object):
    """Simple position controller for the snake."""
    def __init__(self):
        pass

    def position_cb(self, msg):
        """Callback for position."""
        pass

    def snake_cb(self, msg):
        """Callback for poses from the snake."""
        pass

if __name__ == "__main__":
    SnakePositionController()
```

## ROS Setup
This section will also be very similar to last time. We're going to initilize
the node, create our subscribers, and create our publishers. Take a look on the
ROS wiki, at both the [std_msgs](http://wiki.ros.org/std_msgs) and
[geometry_msgs](http://wiki.ros.org/geometry_msgs) packages and see if you can
pick out a good message type for the subscribers and publishers. Remember that
the message types need to match if we're recieving data from or sending data to
nodes that have already been written.

Once you've got a handle on that, go ahead and write the `__init__` method. You
can also update the argument names in the callbacks to be more explicit. Don't
forget any imports too!

You should have gotten something like this:
```python
#!/usr/bin/env python
"""Node to control the position of the snake.

License removed for brevity
"""

# ROS
import rospy
from geometry_msgs.msg import PoseArray, Point
from std_msgs.msg import Float64

class SnakePositionController(object):
    """Simple position controller for the snake."""
    def __init__(self):
        rospy.init_node('snake_position_controller')

        # Publishers
        self.heading_pub = rospy.Publisher('controller/heading', Float64, queue_size=1)

        # Subscribers
        rospy.Subscriber('snake/pose', PoseArray, self.snake_cb)
        rospy.Subscriber('controller/position', Point, self.position_cb)

        rospy.spin()

    def position_cb(self, point_msg):
        """Callback for position."""
        pass

    def snake_cb(self, pose_msg):
        """Callback for poses from the snake."""
        pass

if __name__ == "__main__":
    SnakePositionController()
```

You can see the `Point` message was selected for the `controller/position`
topic. Other notable options were `Pose`, `Pose2D`, and time-stamped variants of
those messages (`PointStamped` and `PoseStamped`, there is no `Pose2DStamped`).
Since our goal is just a position and doesn't need any orientation data, we
won't use a Pose type. Additionally, Pose2D is depreciated and shouldn't be used
anyways. We could have time-stamped the message, but a timestamp isn't needed.
We'll always just chase the latest position command.

There are many other messages worth looking at on the [ROS wiki](http://wiki.ros.org/common_msgs)
if you have a specific need in the future.

## Implementing the Controller
This controller can be implemented much like the last one. Create a variable to
track the desired position, then modify the positon callback in order to set it.

Try two write out the logic in the snake callback. You'll need a quick check,
then somehow you'll need to calculate the heading command. This can simply be
the heading from the point you are currently at, to the point you want to be at.
Once you have that, publish it as a ROS message. Don't forget any imports!

If you need a hint, the `atan2` function will help you out.

You should have come up with something like this:
```python
#!/usr/bin/env python
"""Node to control the position of the snake.

License removed for brevity
"""

# Python
import math

# ROS
import rospy
from geometry_msgs.msg import PoseArray, Point
from std_msgs.msg import Float64

class SnakePositionController(object):
    """Simple position controller for the snake."""
    def __init__(self):
        rospy.init_node('snake_position_controller')

        self.position = None

        # Publishers
        self.heading_pub = rospy.Publisher('controller/heading', Float64, queue_size=1)

        # Subscribers
        rospy.Subscriber('snake/pose', PoseArray, self.snake_cb)
        rospy.Subscriber('controller/position', Point, self.position_cb)

        rospy.spin()

    def position_cb(self, point_msg):
        """Callback for position."""
        self.position = (point_msg.x, point_msg.y)

    def snake_cb(self, pose_msg):
        """Callback for poses from the snake."""
        if self.position is not None:
            pose = (pose_msg.poses[0].position.x, pose_msg.poses[0].position.y)

            # make a local copy to avoid threading issues
            position = self.position
            heading = math.atan2(position[1] - pose[1], position[0] - pose[0])
            self.heading_pub.publish(heading)

if __name__ == "__main__":
    SnakePositionController()
```
Something you'll notice is how a local copy of position is made. Look at these
two lines:
```python
            # make a local copy to avoid threading issues
            position = self.position
```

In rospy, the callbacks all happen in different threads. A thread is a separate,
simultaneously running, task. In Python, they aren't actually simultaneous, but
instead the computer will jump back and forth between different threads at a
high rate. A bytecode command, which is the smallest chunk that Python commands
can be split into can get interleaved between the two threads from it jumping
back and forth.

Essentially, you're not guaranteed that a callback will run all the way through
before a different callback will get started. If we were to remove the two lines
that were referenced earlier, it is possible that the call to `atan2` will be
started and the program will run a few bytecode commands in order to calculate
the value of the first argument. Then, the execution could go to a different
thread, where the first callback is being run. This could changes the
`self.position` tuple to a completely different number while the earlier thread
calculating the `atan2` is 'paused'. Execution jumps back to the earlier thread
and keeps working on the `atan2` call. The second argument is now calculated
with the new point. When `atan2` is actually called, it would receive the Y
value from the first point, and the X value from the second. It would then be
calculating the heading to a location that isn't either of the two points we
wanted it to go to!

For this specific case, it wouldn't be a critical issue since it can't lead to a
program crash (as far as I know ...). The pose callback would also be running
at a much higher rate than the position callback gets new data, so any weirdness
would be quickly fixed in the next iteration.

Regardless of how big an issue it could potentially be, it is still good to
write threadsafe code. This is discussed further in **06_next-steps**, along
with how making a local copy can resolves that issue. Making a local copy will
not always resolve the issue, but it does for this specific case. It also
discusses another technique to write threadsafe code using `Locks`.

## Updating Launch Files and Testing
Updating the launch file is relatively simple. Put this line in
`snake_controller.launch` right below the other node:
```xml
<node type="snake_position_controller" pkg="snake_tutorial" name="snake_position_controller"/>
```

Running the code follows the same procedure as before. Make sure to `catkin
build` and source!

For testing, you can use either `rostopic pub` or `rqt_publisher`. See if you
can figure out the command for `rostopic pub`. Tab completion will be your
friend :)

## Final Notes
Congratulations, you just finished your second node and are almost done!
This tutorial was much more hands off, so hopefully you were able to apply a lot
of the knowledge you gained while writing the first node in order to write this
one.

Below, you'll see the final file we developed. Feel free to experiment with
making changes to this node, or move on to the next one.

### Full File for Reference:
```python
#!/usr/bin/env python
"""Node to control the position of the snake.

License removed for brevity
"""

# Python
import math

# ROS
import rospy
from geometry_msgs.msg import PoseArray, Point
from std_msgs.msg import Float64

class SnakePositionController(object):
    """Simple position controller for the snake."""
    def __init__(self):
        rospy.init_node('snake_position_controller')

        self.position = None

        # Publishers
        self.heading_pub = rospy.Publisher('controller/heading', Float64, queue_size=1)

        # Subscribers
        rospy.Subscriber('snake/pose', PoseArray, self.snake_cb)
        rospy.Subscriber('controller/position', Point, self.position_cb)

        rospy.spin()

    def position_cb(self, point_msg):
        """Callback for position."""
        self.position = (point_msg.x, point_msg.y)

    def snake_cb(self, pose_msg):
        """Callback for poses from the snake."""
        if self.position is not None:
            pose = (pose_msg.poses[0].position.x, pose_msg.poses[0].position.y)

            # make a local copy to avoid threading issues
            position = self.position
            heading = math.atan2(position[1] - pose[1], position[0] - pose[0])
            self.heading_pub.publish(heading)

if __name__ == "__main__":
    SnakePositionController()
```
