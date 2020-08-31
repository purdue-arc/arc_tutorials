# Heading Controller
## Overview
Since our snake recieves linear and angular velocity inputs, making a heading
controller seems like a good place to start. This will allow us to control the
heading of the snake using feedback control.

Feedback control (also called closed-loop control) means that we are using
sensor readings (in this case the output pose of the snake) in order to create
our control signal that we send to the snake. As a block diagram, it may look
like this:
![heading controller](images/heading-controller.png)

## Creating the Program
Let's go ahead and implement this controller in Python now.

We need to start by creating a new file. In the last tutorial, we were left with
the proper directory structure to start writing code. We need to make a new file
in the `nodes` directory to house our code. We'll call this file
`snake_position_controller`. 

Note that it is general practice not to add a `.py` file extension to nodes.
This is because the file name becomes the name of the node when building our
package with catkin. Ex: `roslaunch snake_controller snake_position_controller`
is cleaner than `roslaunch snake_controller snake_position_controller.py`.

Let's start the file by writing a shebang and docstring.
```python
#!/usr/bin/env python
"""Node to control the heading of the snake.

License removed for brevity
"""
```
A shebang is a one line comment at the start of the file that tells the
computer how to run it. We will always use `#!/usr/bin/env python` for Python
files. You can read more about them [here](https://en.wikipedia.org/wiki/Shebang_(Unix))

A docstring is a comment in triple double quotes (`"""`) that serves as a
comment for that file, class, function, or method. They are part of the PEP 8
style guide for Python, which we'll be following for our Python code. You can
read more about docstrings [here](https://www.python.org/dev/peps/pep-0257/#what-is-a-docstring),
and more about PEP 8 [here](https://www.python.org/dev/peps/pep-0008/#introduction).

Now, let's move on and start laying the foundation for our file by writing our
main function and creating a class to hold our logic.
```python
class SnakeHeadingController(object):
    """Simple heading controller for the snake."""
    def __init__(self):
        pass

if __name__ == "__main__":
    SnakeHeadingController()
```
What we've done in the first part of this program is create a class called
`SnakeHeadingController`. A class lets you group methods and variables together
inside an object. It is a key part of Object Oriented Programming (OOP). It is
useful for us, because we're going to have several variables holding data that
we'll need to access from different methods.

This class has a docstring like before, and it only has one method defined,
`__init__`. This function is called when you create a new instance of that class
and is used for initialization. Right now, we've put `pass` inside, which is a
Python keyword meaning "do nothing." It is a handy placeholder because leaving
the body of that function blank would result in an error.

In the second part of the program, we've told the program what to do if it is
executed. This piece gets called when we tell ROS to launch the node. In this
case, it is creating an instance of our `SnakeHeadingController` class and
doing nothing else.

If you'd like, you can run this piece of code from your terminal with the
command:
```bash
./snake_heading_controller
```
(this command works because of the shebang :) )

A keen observer will notice that nothing happened, and our program ended
immediately. This is because our code doesn't do anything yet. We call the
second part of our code, which creates an instance of a `SnakeHeadingController`,
which gets initialized through the `__init__` method, which does nothing. You
can put in print commands in various locations if you are confused about the
order in which that all happens:
```python
print "I am here #1"
```

## Creating the Node
Let's start our ROS specific setup now.

Let's modify the body of the `__init__` function to be the following. This is
replacing the `pass` keyword.
```python
        rospy.init_node('snake_heading_controller')

        rospy.spin()
```

Very importantly, we also need to tell Python to import the `rospy` module, so
that we have access to these functions. Add the following just below the
docstring:
```python
import rospy
```
An import command lets you pull in functionality from other Python files. This
is super useful to be able to re-use code and write small, module files. If you
look at the source of the `snakesim` package, you will see it is full of imports.

Here is our current file for reference:
```python
#!/usr/bin/env python
"""Node to control the heading of the snake.

License removed for brevity
"""
import rospy

class SnakeHeadingController(object):
    """Simple heading controller for the snake."""
    def __init__(self):
        rospy.init_node('snake_heading_controller')

        rospy.spin()

if __name__ == "__main__":
    SnakeHeadingController()
```

Let's run the program again to see what happens. Note that the commands are a
little bit more involved since we need the ROS core to be running.

Terminal 1:
```bash
roscore
```

Terminal 2:
```bash
./snake_heading_controller
```
Compared to the first time we ran this program, it may not seem like much has
changed. However, notice that the program will run forever. We need to manually
kill it with `ctrl+C`. This is due to the `spin` command. Essentially, this
command tells the node to wait indefinitely and process message subscriptions.
We don't have any current subscriptions, so the program isn't doing anything.

We can see the effect of the `init_node` command by running this in a third
terminal:
```bash
rosnode list
```
You will see your node, `snake-heading-controller` in the list of running nodes!

Feel free to try changing the name of the node or removing the `spin` command
to see what it does.

## Creating Subscriptions
In the last section, we learned that `spin` tells the program to wait
indefinitely and process message subscriptions. Let's create some of those
subscriptions now.

Before our `spin` command, add the following lines:
```python
        # Subscribers
        rospy.Subscriber('snake/pose', PoseArray, self.pose_cb)
        rospy.Subscriber('controller/heading', Float64, self.heading_cb)
```
The `Subscriber` command tells ROS to subscribe to a topic (argument 1), of a
specific type (argument 2), and call a function (argument 3) when it recieves a
new message. We need to define those callback functions now.

Put these lines after `init`:
```python
    def heading_cb(self, heading_msg):
        """Callback for heading goal."""
        pass

    def pose_cb(self, pose_msg):
        """Callback for poses from the snake."""
        pass
```
We are defining methods for our `SnakeHeadingController` and simply using `pass`
so that we can hold off on the implementation for them. If you remember our
block diagram from earlier, we are subscribing to the desired heading and true
heading respectively.

The callbacks have one argument each (ignoring self), which is the contents of
the ROS message recieved. We'll talk more about how to interpret those shortly.

Lastly, we need to tell Python where to find these message types. Put this right
after our existing `import` command:
```python
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float64
```

Here is our current file for reference:
```python
#!/usr/bin/env python
"""Node to control the heading of the snake.

License removed for brevity
"""
import rospy
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float64

class SnakeHeadingController(object):
    """Simple heading controller for the snake."""
    def __init__(self):
        rospy.init_node('snake_heading_controller')

        # Subscribers
        rospy.Subscriber('snake/pose', PoseArray, self.pose_cb)
        rospy.Subscriber('controller/heading', Float64, self.heading_cb)

        rospy.spin()

    def heading_cb(self, heading_msg):
        """Callback for heading goal."""
        pass

    def pose_cb(self, pose_msg):
        """Callback for poses from the snake."""
        pass

if __name__ == "__main__":
    SnakeHeadingController()
```

If we run the file now, we should see the two new callbacks. We'll use the same
first two commands, but we'll use a slightly different third command in order
to inspect the node.

Terminal 1:
```bash
roscore
```

Terminal 2:
```bash
./snake_heading_controller
```

Terminal 3:
```bash
rosnode info snake_heading_controller
```
You will see it is subscribed to `snake/pose` and `controller/heading` like we
intended!

## Creating Publishers
Thinking back to our block diagram, our program needs to output the commanded
linear velocity. We will need a publisher in order to do this.

Put this by the subscribers code. It can go before or after, but I like to put
them before.
```python
        # Publishers
        self.twist_pub = rospy.Publisher('snake/cmd_vel', Twist, queue_size=1)
```
This `Publisher` command is similar to the `Subscriber` command. It will create
a ROS publisher on a given topic (argument 1), of a given type (argument 2),
with a specific queue size (argument 3). A queue size of 1 means that the most
recent message will always be sent and any older messages waiting to be sent
will get dropped. This is good for our application, since we don't want a delay
caused by old messages stuck in a buffer. You can read more about queue sizes
on the [ROS wiki](http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers#queue_size:_publish.28.29_behavior_and_queuing).

Another difference is that we are keeping a reference to the `Publisher` object.
This is important so that we can publish messages via that reference in the
future.

Again, we need to tell Python where to find this new message type. Replace one
of the exising `import` commands with the following:
```python
from geometry_msgs.msg import Twist, PoseArray
```

Here is our current file for reference:
```python
#!/usr/bin/env python
"""Node to control the heading of the snake.

License removed for brevity
"""
import rospy
from geometry_msgs.msg import Twist, PoseArray
from std_msgs.msg import Float64

class SnakeHeadingController(object):
    """Simple heading controller for the snake."""
    def __init__(self):
        rospy.init_node('snake_heading_controller')

        # Publishers
        self.twist_pub = rospy.Publisher('snake/cmd_vel', Twist, queue_size=1)

        # Subscribers
        rospy.Subscriber('snake/pose', PoseArray, self.pose_cb)
        rospy.Subscriber('controller/heading', Float64, self.heading_cb)

        rospy.spin()

    def heading_cb(self, heading_msg):
        """Callback for heading goal."""
        pass

    def pose_cb(self, pose_msg):
        """Callback for poses from the snake."""
        pass

if __name__ == "__main__":
    SnakeHeadingController()
```

We can test with the same commands as earlier:

Terminal 1:
```bash
roscore
```

Terminal 2:
```bash
./snake_heading_controller
```

Terminal 3:
```bash
rosnode info snake_heading_controller
```
You will see it is now publishing to `snake/cmd_vel` as we hoped!

## Dealing with ROS Message Definitions
Let's quickly talk about how to pull data out of the ROS messages that our
callbacks are recieving. The first thing you'll want to do is determine the
layout of the message you're receiving. This can be done by browsing the API
online or with a terminal command.

Online:
- [std_msgs/Float64](http://docs.ros.org/api/std_msgs/html/msg/Float64.html)
- [geometry_msgs/PoseArray](http://docs.ros.org/api/geometry_msgs/html/msg/PoseArray.html)

Terminal:
```bash
rosmsg info std_msgs/Float64
rosmsg info geometry_msgs/PoseArray
```

Note that message definitions can be nested. Our `PoseArray` message holds an
array of `Pose` messages, which is a different message type you can also find
on the [online API](http://docs.ros.org/api/geometry_msgs/html/msg/Pose.html).

Let's look at the heading callback (`heading_cb`) first since it has a simpler
message. There is only one field, `data`, which holds a 64 bit floating point
number. This is the same as a `float` type for most Python implementations.

We'll save that to a local variable and figure out what to do with it later.
Replace `pass` with the following command:
```python
heading_command = heading_msg.data
```

Let's look at the pose callback (`pose_cb`) now. You can see that it contains an
array of `Poses`. From the `README.md` file included in the snakesim package, we
know that this is an array with the pose of each element of the snake starting
at the head. We are only interested in the heading of the first segment, which
corresponds to the yaw of the pose at index 0.

We know we'll be looking at something like this:
```python
orientation = pose_msg.poses[0].orientation
```

To make things a little bit trickier, this orientation is encoded as a
quaternion, not an Euler angle. There are many good reasons ROS uses quaternions
instead of Euler angles. It avoids singularities, is compact, and there is no
ambiguity about what convention is in use. For humans though, Euler angles are
normally easier to understand, so we'll convert to them with the help of a
module included with ROS.

First, we'll import the module. Put this with your other `import` commands:
```python
from tf.transformations import euler_from_quaternion
```
You can read the documentation on this function [online](http://docs.ros.org/melodic/api/tf/html/python/transformations.html#tf.transformations.euler_from_quaternion).

Next, we need to put the quaternion data into a tuple in (x,y,z,w) order, then
call the function to get the Euler angles as a tuple in (roll, pitch, yaw)
order. Replace `pass` with the following:
```python
            quat = (pose_msg.poses[0].orientation.x,
                    pose_msg.poses[0].orientation.y,
                    pose_msg.poses[0].orientation.z,
                    pose_msg.poses[0].orientation.w)

            __, __, heading = euler_from_quaternion(quat)
```

Here is our current file for reference:
```python
#!/usr/bin/env python
"""Node to control the heading of the snake.

License removed for brevity
"""
import rospy
from geometry_msgs.msg import Twist, PoseArray
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion

class SnakeHeadingController(object):
    """Simple heading controller for the snake."""
    def __init__(self):
        rospy.init_node('snake_heading_controller')

        # Publishers
        self.twist_pub = rospy.Publisher('snake/cmd_vel', Twist, queue_size=1)

        # Subscribers
        rospy.Subscriber('snake/pose', PoseArray, self.pose_cb)
        rospy.Subscriber('controller/heading', Float64, self.heading_cb)

        rospy.spin()

    def heading_cb(self, heading_msg):
        """Callback for heading goal."""
        orientation = pose_msg.poses[0].orientation

    def pose_cb(self, pose_msg):
        """Callback for poses from the snake."""
        quat = (pose_msg.poses[0].orientation.x,
                pose_msg.poses[0].orientation.y,
                pose_msg.poses[0].orientation.z,
                pose_msg.poses[0].orientation.w)

        __, __, heading = euler_from_quaternion(quat)

if __name__ == "__main__":
    SnakeHeadingController()
```

You can run it again if you want, but there shouldn't be any difference from the
last time we ran it.

## Implementing the Controller
Now that everything is nicely laid out, let's get to the actual logic behind
the controller.

The first thing we want to do is figure out where the main loop is going to take
place. Since the pose callback is going to happen at a reasonably high rate, it
seems safe to put our code in there. If we wanted to be more careful, we could
look into using a `timer` or `rate` object.

In order to keep things simple, we're going to implement what is called a
[bang-bang controller](https://en.wikipedia.org/wiki/Bang%E2%80%93bang_control).
Essentially, it will output a fixed magnitude, positive or negative command
depending on the sign of the error. If we are too far left, it will shoot us
right. If we are too far right, it will shoot us left. If we wanted to be
fancier, we could look at something like a PID controller, but this will be 
eaiser to implement and work just fine for our puproses.

On a basic level, our logic is going to look something like this:
```python
if have_heading_command:
    error = heading - heading_command
    angular_velocity = sign(error) * ANGULAR_VELOCITY_MAG
```

From this pseudocode, we know a few things:
 - we need a variable to hold the magnitude of the angular velocity output
 - we need to get the heading command data from the other callback
 - we need a good way to subtract angles that can handle +/- pi being the same
 - we need a math library to copy the sign of the error

Let's handle these in order:

First, we'll create an `ANGULAR_VELOCITY_MAG` variable. Put this line in the
`init` function after `init_node`:
```python
self.ANGULAR_VELOCITY_MAG = 2.0
```

Next, we'll make a variable for tracking the heading command. Put this by the
previous line:
```python
self.heading_command = None
```

Update the heading callback to use this variable rather than a local variable:
```python
self.heading_command = heading_msg.data
```

In order to handle the difference of angles, we'll use the `angles` module. Put
this line with your imports:
```python
from angles import shortest_angular_distance
```

In order to handle the sign of the error, we'll use the `math module. Put this
line with your imports:
```python
from math import copysign
```

Now that we have all that out of the way, let's actually write the logic for the
loop. Modify the body of the pose callback to be the following:

```python
        if (self.heading_command is not None):
            quat = (pose_msg.poses[0].orientation.x,
                    pose_msg.poses[0].orientation.y,
                    pose_msg.poses[0].orientation.z,
                    pose_msg.poses[0].orientation.w)

            __, __, heading = euler_from_quaternion(quat)
            error = shortest_angular_distance(heading, self.heading_command)
            angular_velocity_command = copysign(self.ANGULAR_VELOCITY, error)
```

Here's the current file for reference:
```python
#!/usr/bin/env python
"""Node to control the heading of the snake.

License removed for brevity
"""

# Python
from math import copysign
from angles import shortest_angular_distance

# ROS
import rospy
from geometry_msgs.msg import Twist, PoseArray
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion

class SnakeHeadingController(object):
    """Simple heading controller for the snake."""
    def __init__(self):
        rospy.init_node('snake_heading_controller')

        self.heading_command = None
        self.ANGULAR_VELOCITY = 6.28

        # Publishers
        self.twist_pub = rospy.Publisher('snake/cmd_vel', Twist, queue_size=1)

        # Subscribers
        rospy.Subscriber('snake/pose', PoseArray, self.pose_cb)
        rospy.Subscriber('controller/heading', Float64, self.heading_cb)

        rospy.spin()

    def heading_cb(self, heading_msg):
        """Callback for heading goal."""
        self.heading_command = heading_msg.data

    def pose_cb(self, pose_msg):
        """Callback for poses from the snake."""
        if (self.heading_command is not None):
            quat = (pose_msg.poses[0].orientation.x,
                    pose_msg.poses[0].orientation.y,
                    pose_msg.poses[0].orientation.z,
                    pose_msg.poses[0].orientation.w)

            __, __, heading = euler_from_quaternion(quat)
            error = shortest_angular_distance(heading, self.heading_command)
            angular_velocity_command = copysign(self.ANGULAR_VELOCITY, error)

if __name__ == "__main__":
    SnakeHeadingController()
```

We'll be able to test this shortly, but it isn't ready just yet.

## Publishing the Ouput
Earlier we made a publisher and we wrote the logic to get the value to publish.
One of the last things we need is actually publishing the value.

Here's the current file for reference:
```python
#!/usr/bin/env python
"""Node to control the heading of the snake.

License removed for brevity
"""

# Python
from math import copysign
from angles import shortest_angular_distance

# ROS
import rospy
from geometry_msgs.msg import Twist, PoseArray
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion

class SnakeHeadingController(object):
    """Simple heading controller for the snake."""
    def __init__(self):
        rospy.init_node('snake_heading_controller')

        self.heading_command = None
        self.ANGULAR_VELOCITY = 6.28
        self.LINEAR_VELOCITY = 2.0

        # Publishers
        self.twist_pub = rospy.Publisher('snake/cmd_vel', Twist, queue_size=1)

        # Subscribers
        rospy.Subscriber('snake/pose', PoseArray, self.pose_cb)
        rospy.Subscriber('controller/heading', Float64, self.heading_cb)

        rospy.spin()

    def heading_cb(self, heading_msg):
        """Callback for heading goal."""
        self.heading_command = heading_msg.data

    def pose_cb(self, pose_msg):
        """Callback for poses from the snake."""
        if (self.heading_command is not None):
            quat = (pose_msg.poses[0].orientation.x,
                    pose_msg.poses[0].orientation.y,
                    pose_msg.poses[0].orientation.z,
                    pose_msg.poses[0].orientation.w)

            __, __, heading = euler_from_quaternion(quat)
            error = shortest_angular_distance(heading, self.heading_command)
            angular_velocity_command = copysign(self.ANGULAR_VELOCITY, error)

            twist_msg = Twist()
            twist_msg.linear.x = self.LINEAR_VELOCITY
            twist_msg.angular.z = angular_velocity_command
            self.twist_pub.publish(twist_msg)

if __name__ == "__main__":
    SnakeHeadingController()
```

## Using Parameters
Here's the current file for reference:
```python
#!/usr/bin/env python
"""Node to control the heading of the snake.

License removed for brevity
"""

# Python
from math import copysign
from angles import shortest_angular_distance

# ROS
import rospy
from geometry_msgs.msg import Twist, PoseArray
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion

class SnakeHeadingController(object):
    """Simple heading controller for the snake."""
    def __init__(self):
        rospy.init_node('snake_heading_controller')

        self.heading_command = None
        self.ANGULAR_VELOCITY = rospy.get_param('~angular_velocity', 6.28)
        self.LINEAR_VELOCITY = rospy.get_param('~linear_velocity', 2.0)

        # Publishers
        self.twist_pub = rospy.Publisher('snake/cmd_vel', Twist, queue_size=1)

        # Subscribers
        rospy.Subscriber('snake/pose', PoseArray, self.pose_cb)
        rospy.Subscriber('controller/heading', Float64, self.heading_cb)

        rospy.spin()

    def heading_cb(self, heading_msg):
        """Callback for heading goal."""
        self.heading_command = heading_msg.data

    def pose_cb(self, pose_msg):
        """Callback for poses from the snake."""
        if (self.heading_command is not None):
            quat = (pose_msg.poses[0].orientation.x,
                    pose_msg.poses[0].orientation.y,
                    pose_msg.poses[0].orientation.z,
                    pose_msg.poses[0].orientation.w)

            __, __, heading = euler_from_quaternion(quat)
            error = shortest_angular_distance(heading, self.heading_command)
            angular_velocity_command = copysign(self.ANGULAR_VELOCITY, error)

            twist_msg = Twist()
            twist_msg.linear.x = self.LINEAR_VELOCITY
            twist_msg.angular.z = angular_velocity_command
            self.twist_pub.publish(twist_msg)

if __name__ == "__main__":
    SnakeHeadingController()
```

## Creating a Launch File


## Final Notes
Full file for reference:
```python
#!/usr/bin/env python
"""Node to control the heading of the snake.

License removed for brevity
"""

# Python
from math import copysign
from angles import shortest_angular_distance

# ROS
import rospy
from geometry_msgs.msg import Twist, PoseArray
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion

class SnakeHeadingController(object):
    """Simple heading controller for the snake."""
    def __init__(self):
        rospy.init_node('snake_heading_controller')

        self.heading_command = None
        self.ANGULAR_VELOCITY = rospy.get_param('~angular_velocity', 6.28)
        self.LINEAR_VELOCITY = rospy.get_param('~linear_velocity', 2.0)

        # Publishers
        self.twist_pub = rospy.Publisher('snake/cmd_vel', Twist, queue_size=1)

        # Subscribers
        rospy.Subscriber('snake/pose', PoseArray, self.pose_cb)
        rospy.Subscriber('controller/heading', Float64, self.heading_cb)

        rospy.spin()

    def heading_cb(self, heading_msg):
        """Callback for heading goal."""
        self.heading_command = heading_msg.data

    def pose_cb(self, pose_msg):
        """Callback for poses from the snake."""
        if (self.heading_command is not None):
            quat = (pose_msg.poses[0].orientation.x,
                    pose_msg.poses[0].orientation.y,
                    pose_msg.poses[0].orientation.z,
                    pose_msg.poses[0].orientation.w)

            __, __, heading = euler_from_quaternion(quat)
            error = shortest_angular_distance(heading, self.heading_command)
            angular_velocity_command = copysign(self.ANGULAR_VELOCITY, error)

            twist_msg = Twist()
            twist_msg.linear.x = self.LINEAR_VELOCITY
            twist_msg.angular.z = angular_velocity_command
            self.twist_pub.publish(twist_msg)

if __name__ == "__main__":
    SnakeHeadingController()
```

